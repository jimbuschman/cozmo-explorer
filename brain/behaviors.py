"""
Robot Behaviors

Low-level behaviors that can be composed into higher-level actions.
Each behavior is an async generator that yields control periodically.
"""
import asyncio
import random
import math
import logging
from typing import Optional, AsyncGenerator, TYPE_CHECKING
from dataclasses import dataclass
from enum import Enum, auto
from datetime import datetime
from pathlib import Path

import config
from cozmo_interface.robot import CozmoRobot, RobotPose

if TYPE_CHECKING:
    from memory.experience_logger import ExperienceLogger
    from memory.learned_rules import LearnedRulesStore

logger = logging.getLogger(__name__)


class BehaviorStatus(Enum):
    """Status of a running behavior"""
    RUNNING = auto()
    COMPLETED = auto()
    FAILED = auto()
    INTERRUPTED = auto()


@dataclass
class BehaviorResult:
    """Result from a completed behavior"""
    status: BehaviorStatus
    message: str = ""
    data: dict = None


class Behavior:
    """Base class for behaviors"""

    def __init__(self, robot: CozmoRobot):
        self.robot = robot
        self._cancelled = False

    def cancel(self):
        """Request behavior cancellation"""
        self._cancelled = True

    @property
    def is_cancelled(self) -> bool:
        return self._cancelled

    async def run(self) -> BehaviorResult:
        """Execute the behavior - override in subclasses"""
        raise NotImplementedError


class StopBehavior(Behavior):
    """Immediately stop all movement"""

    async def run(self) -> BehaviorResult:
        await self.robot.stop()
        return BehaviorResult(BehaviorStatus.COMPLETED, "Stopped")


class DriveDistanceBehavior(Behavior):
    """Drive a specific distance"""

    def __init__(self, robot: CozmoRobot, distance_mm: float, speed: float = 50.0):
        super().__init__(robot)
        self.distance = distance_mm
        self.speed = speed

    async def run(self) -> BehaviorResult:
        if self.distance == 0:
            return BehaviorResult(BehaviorStatus.COMPLETED, "Zero distance")

        # Calculate duration based on distance and speed
        duration = abs(self.distance) / self.speed
        actual_speed = self.speed if self.distance > 0 else -self.speed

        start_pose = RobotPose(
            self.robot.pose.x,
            self.robot.pose.y,
            self.robot.pose.angle
        )

        await self.robot.drive(actual_speed)

        # Monitor progress
        elapsed = 0.0
        check_interval = 0.1

        while elapsed < duration and not self.is_cancelled:
            await asyncio.sleep(check_interval)
            elapsed += check_interval

            # Check for cliff
            if self.robot.sensors.cliff_detected:
                await self.robot.stop()
                return BehaviorResult(
                    BehaviorStatus.INTERRUPTED,
                    "Cliff detected"
                )

            # Check if picked up
            if self.robot.sensors.is_picked_up:
                await self.robot.stop()
                return BehaviorResult(
                    BehaviorStatus.INTERRUPTED,
                    "Robot picked up"
                )

        await self.robot.stop()

        if self.is_cancelled:
            return BehaviorResult(BehaviorStatus.INTERRUPTED, "Cancelled")

        return BehaviorResult(
            BehaviorStatus.COMPLETED,
            f"Drove {self.distance}mm",
            {"distance": self.distance, "duration": duration}
        )


class TurnAngleBehavior(Behavior):
    """Turn by a specific angle"""

    def __init__(self, robot: CozmoRobot, angle_deg: float, speed: float = 45.0):
        super().__init__(robot)
        self.angle = angle_deg
        self.speed = speed

    async def run(self) -> BehaviorResult:
        if self.angle == 0:
            return BehaviorResult(BehaviorStatus.COMPLETED, "Zero angle")

        await self.robot.turn(self.angle, self.speed)

        if self.is_cancelled:
            await self.robot.stop()
            return BehaviorResult(BehaviorStatus.INTERRUPTED, "Cancelled")

        return BehaviorResult(
            BehaviorStatus.COMPLETED,
            f"Turned {self.angle} degrees"
        )


class WanderBehavior(Behavior):
    """
    Random wandering exploration with proactive obstacle avoidance.

    Uses external distance sensors (if available) for proactive avoidance,
    falls back to reactive collision detection if not.

    Optionally integrates with the learning system to:
    - Log sensor data, actions, and outcomes
    - Apply learned rules to modify recovery behavior
    """

    # Distance thresholds (mm)
    DANGER_DISTANCE = 80      # Emergency stop
    SLOW_DISTANCE = 150       # Slow down
    CAUTION_DISTANCE = 250    # Start looking for alternatives

    # Default recovery angles (data shows 120°+ works, 90° never succeeds with trailer)
    DEFAULT_STALL_ANGLES = [-120, -135, 120, 135]
    DEFAULT_CLIFF_ANGLES = [-135, -150, 135, 150]

    def __init__(
        self,
        robot: CozmoRobot,
        duration: float = 30.0,
        speed: float = 50.0,
        turn_probability: float = 0.1,
        experience_logger: "ExperienceLogger" = None,
        rules_store: "LearnedRulesStore" = None
    ):
        super().__init__(robot)
        self.duration = duration
        self.speed = speed
        self.turn_probability = turn_probability
        self.experience_logger = experience_logger
        self.rules_store = rules_store

        # Track current action for outcome logging
        self._current_action_id: Optional[int] = None
        self._current_action_type: Optional[str] = None

    async def run(self) -> BehaviorResult:
        elapsed = 0.0
        check_interval = 0.15  # Faster checks for obstacle response
        distance_traveled = 0.0
        turns_made = 0
        stall_check_count = 0
        stall_imu_count = 0

        logger.info(f"Starting wander for {self.duration}s")
        has_distance_sensors = self.robot.sensors.ext_connected

        while elapsed < self.duration and not self.is_cancelled:
            sensors = self.robot.sensors

            # === SAFETY CHECKS ===
            if sensors.cliff_detected:
                logger.info("Cliff detected, backing up and turning")
                await self._escape_cliff()
                turns_made += 1
                elapsed += 1.5
                last_pose_x, last_pose_y = self.robot.pose.x, self.robot.pose.y
                continue

            if sensors.is_picked_up:
                await self.robot.stop()
                return BehaviorResult(BehaviorStatus.INTERRUPTED, "Robot picked up")

            # Check tilt from external IMU
            if has_distance_sensors and (abs(sensors.ext_pitch) > 35 or abs(sensors.ext_roll) > 35):
                logger.warning(f"Tilt detected: pitch={sensors.ext_pitch:.1f}° roll={sensors.ext_roll:.1f}°")
                await self.robot.stop()
                await asyncio.sleep(0.5)
                continue

            # === PROACTIVE OBSTACLE AVOIDANCE ===
            if has_distance_sensors:
                front_dist = sensors.get_front_obstacle_distance()
                left_dist = sensors.ext_ultra_l_mm
                right_dist = sensors.ext_ultra_r_mm

                # Emergency stop
                if 0 < front_dist < self.DANGER_DISTANCE:
                    logger.warning(f"Obstacle at {front_dist}mm, emergency reverse")
                    await self.robot.stop()
                    escape_speed = config.ESCAPE_SPEED
                    backup_time = 1.0 if config.TRAILER_MODE else 0.5
                    await self.robot.drive(-escape_speed, duration=backup_time)
                    await asyncio.sleep(backup_time)
                    turn_angle = self._pick_turn_direction(left_dist, right_dist, 90)
                    await self.robot.turn(turn_angle)
                    sensors.collision_detected = False  # Clear flag from maneuver
                    turns_made += 1
                    elapsed += backup_time + 1.0
                    last_pose_x, last_pose_y = self.robot.pose.x, self.robot.pose.y
                    stall_check_count = 0
                    continue

                # Caution zone - turn away
                if 0 < front_dist < self.CAUTION_DISTANCE:
                    turn_angle = self._pick_turn_direction(left_dist, right_dist, 45)
                    logger.info(f"Obstacle at {front_dist}mm, turning {turn_angle}°")
                    await self.robot.stop()
                    await self.robot.turn(turn_angle)
                    turns_made += 1
                    elapsed += abs(turn_angle) / 45
                    continue

                # Speed adjustment
                current_speed = self.speed * 0.5 if 0 < front_dist < self.SLOW_DISTANCE else self.speed
            else:
                current_speed = self.speed

            # === STALL DETECTION (IMU-based) ===
            is_stalled = False
            if sensors.collision_detected:
                is_stalled = True
                logger.info("Collision detected")
                sensors.collision_detected = False
            elif has_distance_sensors and stall_check_count > 8:
                # Use MPU gyro + accelerometer to detect actual movement
                # If we're commanding drive but IMU shows no motion, we're stuck
                gyro_activity = abs(sensors.ext_gz_dps)  # Yaw rotation
                accel_activity = math.sqrt(sensors.ext_ax_g**2 + sensors.ext_ay_g**2)
                # Moving robot has gyro > 2 dps or horizontal accel > 0.05g
                if gyro_activity < 2.0 and accel_activity < 0.05:
                    stall_imu_count += 1
                    if stall_imu_count > 6:  # ~1 second of no movement
                        is_stalled = True
                        logger.info(f"Stall detected (IMU): gyro={gyro_activity:.1f}dps accel={accel_activity:.3f}g over {stall_imu_count} checks")
                else:
                    stall_imu_count = 0

            if is_stalled:
                await self._escape_stall()
                turns_made += 1
                elapsed += 1.5
                stall_check_count = 0
                stall_imu_count = 0
                continue

            stall_check_count += 1

            # === RANDOM EXPLORATION ===
            if random.random() < self.turn_probability:
                await self.robot.stop()
                turn_angle = random.uniform(-60, 60)
                await self.robot.turn(turn_angle)
                turns_made += 1
                elapsed += abs(turn_angle) / 45
                last_pose_x, last_pose_y = self.robot.pose.x, self.robot.pose.y
                stall_check_count = 0

            # === DRIVE ===
            await self.robot.drive(current_speed)
            await asyncio.sleep(check_interval)
            distance_traveled += current_speed * check_interval
            elapsed += check_interval

        await self.robot.stop()

        return BehaviorResult(
            BehaviorStatus.COMPLETED if not self.is_cancelled else BehaviorStatus.INTERRUPTED,
            f"Wandered for {elapsed:.1f}s",
            {"duration": elapsed, "distance_estimate": distance_traveled, "turns": turns_made}
        )

    def _pick_turn_direction(self, left_dist: int, right_dist: int, magnitude: int = 90) -> int:
        """Pick turn direction based on clearance."""
        if left_dist <= 0 and right_dist <= 0:
            return random.choice([-magnitude, magnitude])
        if left_dist <= 0:
            return -magnitude
        if right_dist <= 0:
            return magnitude
        return magnitude if left_dist > right_dist else -magnitude

    async def _escape_cliff(self):
        """Escape from cliff detection - back up and turn away"""
        # Skip escape images to save battery (lift cycling is expensive)
        # Sensor snapshots still capture all the important data
        before_image_path = None

        # Log sensor snapshot before action
        snapshot_id = None
        if self.experience_logger:
            snapshot_id = self.experience_logger.log_sensor_snapshot_from_robot(
                self.robot, image_path=before_image_path
            )

        # Get sensor context for rule matching
        sensor_context = self._get_sensor_context()

        # Determine turn angles - apply learned rules if available
        base_action = {"angles": self.DEFAULT_CLIFF_ANGLES, "backup_duration": 0.5, "arc_ratio": "gentle"}
        applied_rule_id = None
        if self.rules_store:
            modified_action, applied_rule_id = self.rules_store.apply_rules_to_action(base_action, sensor_context)
            angles = modified_action.get("angles", self.DEFAULT_CLIFF_ANGLES)
            backup_duration = modified_action.get("backup_duration", 0.5)
            arc_ratio_name = modified_action.get("arc_ratio", "gentle")
        else:
            angles = self.DEFAULT_CLIFF_ANGLES
            backup_duration = 0.5
            arc_ratio_name = "gentle"

        # Choose angle
        angle = random.choice(angles)

        # Log action
        action_id = None
        if self.experience_logger:
            action_id = self.experience_logger.log_action(
                action_type="escape_cliff",
                parameters={
                    "angle": angle,
                    "backup_duration": backup_duration,
                    "rule_id": applied_rule_id,
                    "trailer_mode": config.TRAILER_MODE,
                    "arc_ratio": arc_ratio_name if config.TRAILER_MODE else None
                },
                trigger="cliff",
                context_snapshot_id=snapshot_id
            )
            self._current_action_id = action_id
            self._current_action_type = "escape_cliff"

        # Execute the escape maneuver
        await self.robot.stop()

        if config.TRAILER_MODE:
            # Trailer escape: back up STRAIGHT first to clear cliff,
            # then forward arc turn to change direction (trailer-safe)
            escape_speed = config.ESCAPE_SPEED
            straight_backup = backup_duration + 0.8  # Back up extra far from cliff

            # Step 1: Straight reverse away from cliff
            await self.robot.drive(-escape_speed, duration=straight_backup)
            await asyncio.sleep(straight_backup)
            await self.robot.stop()

            # Step 2: Forward arc turn to new direction (safe for trailer)
            arc_ratio = config.ARC_RATIOS.get(arc_ratio_name, config.TRAILER_ARC_RATIO)
            arc_duration = abs(angle) / 30.0 * 1.5
            if angle > 0:
                await self.robot.arc_turn_left(escape_speed, arc_ratio, arc_duration)
            else:
                await self.robot.arc_turn_right(escape_speed, arc_ratio, arc_duration)
        else:
            # Normal mode: back up straight then turn
            await self.robot.drive(-self.speed, duration=backup_duration)
            await asyncio.sleep(backup_duration)
            await self.robot.turn(angle)

        # Clear collision flag so escape maneuver itself doesn't re-trigger
        self.robot.sensors.collision_detected = False

        # Log outcome (assume success if we get here without another cliff)
        if self.experience_logger and action_id:
            after_image_path = None  # Skip to save battery

            post_snapshot_id = self.experience_logger.log_sensor_snapshot_from_robot(
                self.robot, image_path=after_image_path
            )
            outcome_type = "cliff" if self.robot.sensors.cliff_detected else "success"
            self.experience_logger.log_outcome(
                action_event_id=action_id,
                outcome_type=outcome_type,
                details={
                    "post_cliff_detected": self.robot.sensors.cliff_detected,
                    "before_image": before_image_path,
                    "after_image": after_image_path
                },
                sensor_snapshot_id=post_snapshot_id
            )

            # Record rule performance for conflict resolution
            if self.rules_store and applied_rule_id:
                self.rules_store.record_rule_application(applied_rule_id, outcome_type == "success")

            self._current_action_id = None
            self._current_action_type = None

    async def _escape_stall(self):
        """Escape from stall/collision - back up and turn away"""
        # Skip escape images to save battery (lift cycling is expensive)
        before_image_path = None

        # Log sensor snapshot before action
        snapshot_id = None
        if self.experience_logger:
            snapshot_id = self.experience_logger.log_sensor_snapshot_from_robot(
                self.robot, image_path=before_image_path
            )

        # Get sensor context for rule matching
        sensor_context = self._get_sensor_context()

        # Determine turn angles - apply learned rules if available
        base_action = {"angles": self.DEFAULT_STALL_ANGLES, "backup_duration": 0.7, "arc_ratio": "medium"}
        applied_rule_id = None
        if self.rules_store:
            modified_action, applied_rule_id = self.rules_store.apply_rules_to_action(base_action, sensor_context)
            angles = modified_action.get("angles", self.DEFAULT_STALL_ANGLES)
            backup_duration = modified_action.get("backup_duration", 0.7)
            arc_ratio_name = modified_action.get("arc_ratio", "medium")
        else:
            angles = self.DEFAULT_STALL_ANGLES
            backup_duration = 0.7
            arc_ratio_name = "medium"

        # Choose angle
        angle = random.choice(angles)

        # Log action
        action_id = None
        if self.experience_logger:
            action_id = self.experience_logger.log_action(
                action_type="escape_stall",
                parameters={
                    "angle": angle,
                    "backup_duration": backup_duration,
                    "rule_id": applied_rule_id,
                    "trailer_mode": config.TRAILER_MODE,
                    "arc_ratio": arc_ratio_name if config.TRAILER_MODE else None
                },
                trigger="collision" if self.robot.sensors.collision_detected else "stall",
                context_snapshot_id=snapshot_id
            )
            self._current_action_id = action_id
            self._current_action_type = "escape_stall"

        # Execute the escape maneuver
        await self.robot.stop()

        if config.TRAILER_MODE:
            # Trailer escape: back up STRAIGHT first to clear obstacle,
            # then forward arc turn to change direction (trailer-safe)
            escape_speed = config.ESCAPE_SPEED  # Use at least 80mm/s for escapes
            straight_backup = backup_duration + 0.5  # Back up longer to clear trailer

            # Step 1: Straight reverse to clear the obstacle
            await self.robot.drive(-escape_speed, duration=straight_backup)
            await asyncio.sleep(straight_backup)
            await self.robot.stop()

            # Step 2: Forward arc turn to change direction (safe for trailer)
            arc_ratio = config.ARC_RATIOS.get(arc_ratio_name, config.TRAILER_ARC_RATIO)
            arc_duration = abs(angle) / 30.0 * 1.5  # Longer arc for trailer
            if angle > 0:
                await self.robot.arc_turn_left(escape_speed, arc_ratio, arc_duration)
            else:
                await self.robot.arc_turn_right(escape_speed, arc_ratio, arc_duration)
        else:
            # Normal mode: back up straight then turn
            await self.robot.drive(-self.speed, duration=backup_duration)
            await asyncio.sleep(backup_duration)
            await self.robot.turn(angle)

        # Clear collision flag so escape maneuver itself doesn't re-trigger
        self.robot.sensors.collision_detected = False

        # Log outcome - check if we're still stuck after the maneuver
        if self.experience_logger and action_id:
            # Brief pause to check result
            await asyncio.sleep(0.2)

            after_image_path = None  # Skip to save battery

            post_snapshot_id = self.experience_logger.log_sensor_snapshot_from_robot(
                self.robot, image_path=after_image_path
            )

            # Determine outcome based on sensors
            if self.robot.sensors.collision_detected:
                outcome_type = "collision"
            elif self.robot.sensors.cliff_detected:
                outcome_type = "cliff"
            else:
                outcome_type = "success"

            self.experience_logger.log_outcome(
                action_event_id=action_id,
                outcome_type=outcome_type,
                details={
                    "angle_used": angle,
                    "backup_duration": backup_duration,
                    "before_image": before_image_path,
                    "after_image": after_image_path
                },
                sensor_snapshot_id=post_snapshot_id
            )

            # Record rule performance for conflict resolution
            if self.rules_store and applied_rule_id:
                self.rules_store.record_rule_application(applied_rule_id, outcome_type == "success")

            self._current_action_id = None
            self._current_action_type = None

    def _get_sensor_context(self) -> dict:
        """Get current sensor values as a dict for rule matching"""
        sensors = self.robot.sensors
        return {
            "ext_ultra_l_mm": sensors.ext_ultra_l_mm,
            "ext_ultra_c_mm": sensors.ext_ultra_c_mm,
            "ext_ultra_r_mm": sensors.ext_ultra_r_mm,
            "ext_tof_mm": sensors.ext_tof_mm,
            "ext_pitch": sensors.ext_pitch,
            "ext_roll": sensors.ext_roll,
            "ext_yaw": sensors.ext_yaw,
            "cliff_detected": sensors.cliff_detected,
            "is_picked_up": sensors.is_picked_up,
            "battery_voltage": sensors.battery_voltage
        }

    async def _capture_action_image(self, action_type: str, phase: str) -> Optional[str]:
        """
        Capture an image during an action for learning data.
        Raises lift to clear sensor pod from camera view, then lowers it.

        Args:
            action_type: Type of action (e.g., "escape_stall", "escape_cliff")
            phase: "before" or "after"

        Returns:
            Path to saved image, or None if capture failed
        """
        try:
            # Raise lift to clear sensor pod from camera view (if needed)
            if config.LIFT_FOR_CAMERA:
                await self.robot.set_lift_height(1.0)
                await asyncio.sleep(1.0)  # Wait for lift to fully raise

            image = await self.robot.capture_image()

            # Lower lift back down (if it was raised)
            if config.LIFT_FOR_CAMERA:
                await self.robot.set_lift_height(0.0)

            if image is None:
                return None

            # Create images directory for learning data
            images_dir = config.DATA_DIR / "learning_images"
            images_dir.mkdir(parents=True, exist_ok=True)

            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = f"{action_type}_{phase}_{timestamp}.jpg"
            filepath = images_dir / filename

            # Save image
            image.save(str(filepath), "JPEG", quality=85)
            logger.debug(f"Captured {phase} image for {action_type}: {filename}")

            return str(filepath)

        except Exception as e:
            logger.debug(f"Failed to capture action image: {e}")
            return None


class LookAroundBehavior(Behavior):
    """
    Look around by panning the camera/head.

    Useful for surveying the environment from a fixed position.
    """

    def __init__(self, robot: CozmoRobot, capture_images: bool = True):
        super().__init__(robot)
        self.capture_images = capture_images
        self.images = []

    async def run(self) -> BehaviorResult:
        # Head angles to look at (radians)
        # Min: -0.44 (down), Max: 0.78 (up)
        head_angles = [-0.2, 0.0, 0.3, 0.6]

        # Body turn angles
        body_angles = [0, 45, 90, 135, 180, -135, -90, -45]

        self.images = []

        # Look straight first
        await self.robot.set_head_angle(0.1)
        await asyncio.sleep(0.3)

        for body_angle in body_angles:
            if self.is_cancelled:
                break

            # Turn to angle
            if body_angle != 0:
                await self.robot.turn(body_angle)

            # Capture at different head angles
            for head_angle in head_angles:
                if self.is_cancelled:
                    break

                await self.robot.set_head_angle(head_angle)
                await asyncio.sleep(0.2)

                if self.capture_images:
                    img = await self.robot.capture_image()
                    if img:
                        self.images.append({
                            'image': img,
                            'head_angle': head_angle,
                            'body_angle': body_angle
                        })

            # Turn back to center for next iteration
            if body_angle != 0:
                await self.robot.turn(-body_angle)

        # Return to neutral
        await self.robot.set_head_angle(0.0)

        return BehaviorResult(
            BehaviorStatus.COMPLETED if not self.is_cancelled else BehaviorStatus.INTERRUPTED,
            f"Captured {len(self.images)} views",
            {"images": self.images}
        )


class GoToPoseBehavior(Behavior):
    """
    Navigate to a target pose using simple dead-reckoning.

    Note: This is basic and will accumulate error. For real navigation,
    you'd want visual odometry or SLAM.
    """

    def __init__(
        self,
        robot: CozmoRobot,
        target: RobotPose,
        position_tolerance: float = 20.0,
        angle_tolerance: float = 10.0
    ):
        super().__init__(robot)
        self.target = target
        self.position_tolerance = position_tolerance
        self.angle_tolerance = math.radians(angle_tolerance)

    async def run(self) -> BehaviorResult:
        max_iterations = 20
        iteration = 0

        while iteration < max_iterations and not self.is_cancelled:
            current = self.robot.pose
            dx = self.target.x - current.x
            dy = self.target.y - current.y
            distance = math.sqrt(dx*dx + dy*dy)

            # Check if we're close enough
            if distance < self.position_tolerance:
                # Final angle adjustment
                angle_diff = self.target.angle - current.angle
                if abs(angle_diff) > self.angle_tolerance:
                    await self.robot.turn(math.degrees(angle_diff))

                return BehaviorResult(
                    BehaviorStatus.COMPLETED,
                    f"Reached target within {distance:.1f}mm"
                )

            # Calculate required heading
            target_heading = math.atan2(dy, dx)
            heading_error = target_heading - current.angle

            # Normalize to [-pi, pi]
            while heading_error > math.pi:
                heading_error -= 2 * math.pi
            while heading_error < -math.pi:
                heading_error += 2 * math.pi

            # Turn if needed
            if abs(heading_error) > self.angle_tolerance:
                await self.robot.turn(math.degrees(heading_error))
                await asyncio.sleep(0.2)

            # Drive toward target
            drive_distance = min(distance, 100)  # Max 100mm per iteration
            drive_behavior = DriveDistanceBehavior(self.robot, drive_distance)
            result = await drive_behavior.run()

            if result.status == BehaviorStatus.INTERRUPTED:
                return result

            iteration += 1
            await asyncio.sleep(0.1)

        if self.is_cancelled:
            return BehaviorResult(BehaviorStatus.INTERRUPTED, "Cancelled")

        return BehaviorResult(
            BehaviorStatus.FAILED,
            "Max iterations reached"
        )


class AvoidObstacleBehavior(Behavior):
    """
    React to an obstacle by backing up and turning away.
    """

    def __init__(self, robot: CozmoRobot, turn_direction: str = "random"):
        super().__init__(robot)
        self.turn_direction = turn_direction

    async def run(self) -> BehaviorResult:
        # Stop first
        await self.robot.stop()

        # Back up
        await self.robot.drive(-50, duration=0.5)
        await asyncio.sleep(0.5)

        # Turn away
        if self.turn_direction == "random":
            angle = random.choice([-90, 90])
        elif self.turn_direction == "left":
            angle = 90
        else:
            angle = -90

        await self.robot.turn(angle)

        return BehaviorResult(
            BehaviorStatus.COMPLETED,
            f"Avoided obstacle, turned {angle} degrees"
        )


# ==================== Behavior Factory ====================

class BehaviorFactory:
    """Factory for creating behaviors with consistent robot reference"""

    def __init__(
        self,
        robot: CozmoRobot,
        experience_logger: "ExperienceLogger" = None,
        rules_store: "LearnedRulesStore" = None
    ):
        self.robot = robot
        self.experience_logger = experience_logger
        self.rules_store = rules_store

    def stop(self) -> StopBehavior:
        return StopBehavior(self.robot)

    def drive(self, distance_mm: float, speed: float = 50.0) -> DriveDistanceBehavior:
        return DriveDistanceBehavior(self.robot, distance_mm, speed)

    def turn(self, angle_deg: float, speed: float = 45.0) -> TurnAngleBehavior:
        return TurnAngleBehavior(self.robot, angle_deg, speed)

    def wander(
        self,
        duration: float = 30.0,
        speed: float = 50.0
    ) -> WanderBehavior:
        return WanderBehavior(
            self.robot,
            duration,
            speed,
            experience_logger=self.experience_logger,
            rules_store=self.rules_store
        )

    def look_around(self, capture_images: bool = True) -> LookAroundBehavior:
        return LookAroundBehavior(self.robot, capture_images)

    def go_to(
        self,
        target: RobotPose,
        tolerance: float = 20.0
    ) -> GoToPoseBehavior:
        return GoToPoseBehavior(self.robot, target, tolerance)

    def avoid_obstacle(self, direction: str = "random") -> AvoidObstacleBehavior:
        return AvoidObstacleBehavior(self.robot, direction)
