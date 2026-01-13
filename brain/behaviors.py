"""
Robot Behaviors

Low-level behaviors that can be composed into higher-level actions.
Each behavior is an async generator that yields control periodically.
"""
import asyncio
import random
import math
import logging
from typing import Optional, AsyncGenerator
from dataclasses import dataclass
from enum import Enum, auto

from cozmo_interface.robot import CozmoRobot, RobotPose

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
    Random wandering exploration.

    Drives forward, occasionally turns randomly, avoids cliffs.
    """

    def __init__(
        self,
        robot: CozmoRobot,
        duration: float = 30.0,
        speed: float = 50.0,
        turn_probability: float = 0.1
    ):
        super().__init__(robot)
        self.duration = duration
        self.speed = speed
        self.turn_probability = turn_probability

    async def run(self) -> BehaviorResult:
        elapsed = 0.0
        check_interval = 0.3
        distance_traveled = 0.0
        turns_made = 0
        stall_check_count = 0
        last_pose_x = self.robot.pose.x
        last_pose_y = self.robot.pose.y

        logger.info(f"Starting wander for {self.duration}s")

        while elapsed < self.duration and not self.is_cancelled:
            # Check sensors
            if self.robot.sensors.cliff_detected:
                logger.info("Cliff detected, backing up and turning")
                await self.robot.stop()
                await self.robot.drive(-self.speed, duration=0.5)
                await asyncio.sleep(0.5)
                turn_angle = random.choice([-90, -135, 90, 135])
                await self.robot.turn(turn_angle)
                turns_made += 1
                elapsed += 1.5
                last_pose_x = self.robot.pose.x
                last_pose_y = self.robot.pose.y
                continue

            if self.robot.sensors.is_picked_up:
                await self.robot.stop()
                return BehaviorResult(
                    BehaviorStatus.INTERRUPTED,
                    "Robot picked up"
                )

            # Stall detection - check if we're actually moving
            current_x = self.robot.pose.x
            current_y = self.robot.pose.y
            movement = math.sqrt((current_x - last_pose_x)**2 + (current_y - last_pose_y)**2)

            # If we've been driving but haven't moved much, we're probably stuck
            # Need 5+ checks (~1.5s) with less than 5mm movement to trigger
            if stall_check_count > 5 and movement < 5.0:  # Less than 5mm movement over ~1.5s
                logger.info("Stall detected! Backing up and turning")
                await self.robot.stop()
                await self.robot.drive(-self.speed, duration=0.7)
                await asyncio.sleep(0.7)
                turn_angle = random.choice([-120, -90, 90, 120])
                await self.robot.turn(turn_angle)
                turns_made += 1
                elapsed += 1.5
                stall_check_count = 0
                last_pose_x = self.robot.pose.x
                last_pose_y = self.robot.pose.y
                continue

            stall_check_count += 1
            if stall_check_count > 8:  # Reset periodically
                last_pose_x = current_x
                last_pose_y = current_y
                stall_check_count = 0

            # Random turn decision
            if random.random() < self.turn_probability:
                await self.robot.stop()
                turn_angle = random.uniform(-60, 60)
                await self.robot.turn(turn_angle)
                turns_made += 1
                elapsed += abs(turn_angle) / 45  # rough estimate
                last_pose_x = self.robot.pose.x
                last_pose_y = self.robot.pose.y
                stall_check_count = 0

            # Drive forward
            await self.robot.drive(self.speed)
            await asyncio.sleep(check_interval)
            distance_traveled += self.speed * check_interval
            elapsed += check_interval

        await self.robot.stop()

        return BehaviorResult(
            BehaviorStatus.COMPLETED if not self.is_cancelled else BehaviorStatus.INTERRUPTED,
            f"Wandered for {elapsed:.1f}s",
            {
                "duration": elapsed,
                "distance_estimate": distance_traveled,
                "turns": turns_made
            }
        )


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

    def __init__(self, robot: CozmoRobot):
        self.robot = robot

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
        return WanderBehavior(self.robot, duration, speed)

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
