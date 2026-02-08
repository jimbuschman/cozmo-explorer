"""
Frontier Navigator

Simple frontier-based navigation: find nearest unexplored area, drive toward it,
update map continuously, escape if stuck, repeat. No rules, no random wandering.

Extracts proven working code from WanderBehavior (map update, stall detection,
tilt detection, escape logic) and wraps it in frontier-directed steering.
"""
import asyncio
import math
import logging
from enum import Enum, auto
from typing import Optional, TYPE_CHECKING

import config

if TYPE_CHECKING:
    from memory.spatial_map import SpatialMap
    from memory.experience_logger import ExperienceLogger

logger = logging.getLogger(__name__)


class NavResult(Enum):
    """Result of a navigation run."""
    MAPPING = auto()       # Still mapping (time ran out but frontiers remain)
    DONE = auto()          # No more frontiers
    PICKED_UP = auto()     # Robot was picked up
    CANCELLED = auto()     # Externally cancelled
    ERROR = auto()


class FrontierNavigator:
    """
    Drives toward the nearest frontier on the map, updating the map continuously.

    Core loop:
        update_map(sensors)
        if obstacle_too_close: escape()
        if stall_detected: escape()
        if no target or time to recheck: target = find_nearest_frontier()
        if heading off: turn toward target
        drive forward
    """

    # Distance thresholds (mm)
    DANGER_DISTANCE = 80
    CAUTION_DISTANCE = 250

    # Escape parameters (deterministic - data shows 135 works at ~100%)
    ESCAPE_ANGLE = 135
    ESCAPE_REVERSE_ARC_DURATION = 3.0  # seconds of reverse-arc (backup while turning)
    ESCAPE_ARC_RATIO = 0.3             # tight arc for escape turns

    # Stall detection
    STALL_WARMUP_CHECKS = 4        # ignore first N checks after start/escape
    STALL_THRESHOLD_CHECKS = 5     # consecutive no-movement checks before stall
    STALL_YAW_THRESHOLD = 1.0      # degrees
    STALL_DIST_THRESHOLD = 10      # mm

    # Tilt detection
    TILT_THRESHOLD_DEG = 25
    TILT_CONSECUTIVE_REQUIRED = 3

    # Frontier recheck
    FRONTIER_RECHECK_INTERVAL = 3.0   # seconds between frontier rechecks
    TARGET_REACHED_DISTANCE = 200     # mm — recheck early if within this distance

    # Relocation — force the robot to move when scanning from one spot plateaus
    STAGNATION_CHECK_INTERVAL = 60.0  # seconds between coverage rate checks
    STAGNATION_MIN_GAIN = 0.005       # minimum coverage gain (0.5%) per interval to not trigger
    RELOCATE_MIN_DISTANCE = 1500      # mm — minimum frontier distance when relocating
    HEADING_TOLERANCE_DEG = 20       # don't turn if heading is close enough
    MAX_TURN_DEG = 60                # cap per-turn correction

    def __init__(
        self,
        robot,
        spatial_map: "SpatialMap",
        duration: float = 600.0,
        speed: float = None,
        experience_logger: "ExperienceLogger" = None,
    ):
        self.robot = robot
        self.spatial_map = spatial_map
        self.duration = duration
        self.speed = speed or config.WANDER_SPEED
        self.experience_logger = experience_logger

        self._cancelled = False

        # Stats
        self.escapes = 0
        self.frontier_targets = 0
        self.elapsed = 0.0

        # Relocation state (persists across run() calls)
        self._last_coverage_pct = 0.0
        self._cumulative_elapsed = 0.0   # total sim-seconds across all run() calls
        self._last_coverage_check_elapsed = 0.0  # cumulative elapsed at last check
        self._relocating = False

    def cancel(self):
        self._cancelled = True

    async def run(self) -> NavResult:
        """Run the frontier navigation loop for up to self.duration seconds."""
        elapsed = 0.0
        check_interval = 0.15
        stall_check_count = 0
        stall_imu_count = 0
        escape_cooldown = 0.0
        last_yaw = self.robot.sensors.ext_yaw
        last_front_dist = self.robot.sensors.get_front_obstacle_distance()

        # Tilt baseline - calibrated from first real reading
        baseline_pitch = None
        baseline_roll = None
        tilt_consecutive = 0

        # Frontier target
        target = None
        last_frontier_check = 0.0

        # Escape direction tracking - avoid picking frontiers toward recent obstacles
        escape_exclude_angle = None   # heading when obstacle was hit (radians)
        consecutive_escapes = 0       # escapes without successful forward driving
        last_escape_time = -999.0     # sim-time of last escape

        # Relocation uses self._last_coverage_pct / self._last_coverage_check / self._relocating
        # (persisted across run() calls since mapper restarts us every 30s)

        has_sensors = self.robot.sensors.ext_connected

        logger.info(f"FrontierNavigator: starting {self.duration}s run at {self.speed}mm/s")

        while elapsed < self.duration and not self._cancelled:
            sensors = self.robot.sensors

            # === SAFETY ===
            if sensors.cliff_detected:
                logger.info("Cliff detected - escaping")
                await self._escape_cliff()
                self.escapes += 1
                target = None
                elapsed += 2.0
                stall_check_count = 0
                stall_imu_count = 0
                last_yaw = sensors.ext_yaw
                last_front_dist = sensors.get_front_obstacle_distance()
                escape_cooldown = 3.0
                continue

            if sensors.is_picked_up:
                await self.robot.stop()
                self.elapsed = elapsed
                return NavResult.PICKED_UP

            # Update sensor availability
            if not has_sensors and sensors.ext_connected:
                has_sensors = True
                logger.info("Sensors connected mid-run")

            # Calibrate tilt baseline
            if has_sensors and baseline_pitch is None:
                if sensors.ext_pitch != 0.0 or sensors.ext_roll != 0.0:
                    baseline_pitch = sensors.ext_pitch
                    baseline_roll = sensors.ext_roll
                    logger.info(f"Tilt baseline: pitch={baseline_pitch:.1f} roll={baseline_roll:.1f}")

            # === TILT CHECK ===
            if has_sensors and baseline_pitch is not None:
                pitch_delta = abs(sensors.ext_pitch - baseline_pitch)
                roll_delta = abs(sensors.ext_roll - baseline_roll)
                if pitch_delta > self.TILT_THRESHOLD_DEG or roll_delta > self.TILT_THRESHOLD_DEG:
                    tilt_consecutive += 1
                    if tilt_consecutive >= self.TILT_CONSECUTIVE_REQUIRED:
                        logger.warning(f"Tilt: pitch_d={pitch_delta:.1f} roll_d={roll_delta:.1f}")
                        await self.robot.stop()
                        await asyncio.sleep(0.5)
                        tilt_consecutive = 0
                        continue
                else:
                    tilt_consecutive = 0

            # === MAP UPDATE ===
            if has_sensors:
                self._update_map(sensors)

            # Tick down escape cooldown
            if escape_cooldown > 0:
                escape_cooldown -= check_interval

            # === OBSTACLE AVOIDANCE ===
            if has_sensors:
                front_dist = sensors.get_front_obstacle_distance()
                left_dist = sensors.ext_ultra_l_mm
                right_dist = sensors.ext_ultra_r_mm

                if 0 < front_dist < self.CAUTION_DISTANCE and escape_cooldown <= 0:
                    logger.info(f"Obstacle at {front_dist}mm - escaping")
                    # Record heading toward obstacle so we avoid frontiers in that direction
                    escape_exclude_angle = self.robot.pose.angle
                    consecutive_escapes += 1
                    last_escape_time = elapsed
                    await self._escape_obstacle(left_dist, right_dist)
                    self.escapes += 1
                    target = None
                    elapsed += 4.0
                    stall_check_count = 0
                    stall_imu_count = 0
                    last_yaw = sensors.ext_yaw
                    last_front_dist = sensors.get_front_obstacle_distance()
                    escape_cooldown = 3.0
                    continue

            # === STALL DETECTION ===
            # Skip stall detection during escape cooldown — the robot just
            # finished an escape and hasn't had time to build up motion yet.
            is_stalled = False
            if escape_cooldown > 0:
                pass  # cooldown active, skip stall checks
            elif sensors.collision_detected:
                is_stalled = True
                logger.info("Collision detected")
                sensors.collision_detected = False
            elif has_sensors and stall_check_count > self.STALL_WARMUP_CHECKS:
                current_yaw = sensors.ext_yaw
                yaw_change = abs(current_yaw - last_yaw)
                if yaw_change > 180:
                    yaw_change = 360 - yaw_change
                current_front = sensors.get_front_obstacle_distance()
                dist_change = abs(current_front - last_front_dist) if last_front_dist > 0 and current_front > 0 else 999

                if yaw_change < self.STALL_YAW_THRESHOLD and dist_change < self.STALL_DIST_THRESHOLD:
                    stall_imu_count += 1
                    if stall_imu_count > self.STALL_THRESHOLD_CHECKS:
                        is_stalled = True
                        logger.info(f"Stall: yaw_d={yaw_change:.1f} dist_d={dist_change:.0f}")
                else:
                    stall_imu_count = 0
                last_yaw = current_yaw
                last_front_dist = current_front

            if is_stalled:
                left_dist = sensors.ext_ultra_l_mm if has_sensors else -1
                right_dist = sensors.ext_ultra_r_mm if has_sensors else -1
                escape_exclude_angle = self.robot.pose.angle
                consecutive_escapes += 1
                last_escape_time = elapsed
                await self._escape_obstacle(left_dist, right_dist)
                self.escapes += 1
                target = None
                elapsed += 2.0
                stall_check_count = 0
                stall_imu_count = 0
                last_yaw = sensors.ext_yaw
                last_front_dist = sensors.get_front_obstacle_distance()
                escape_cooldown = 3.0
                continue

            stall_check_count += 1

            # Clear escape exclusion after driving successfully for a while
            # (~4.5s = 30 checks at 0.15s without triggering any escape)
            if consecutive_escapes > 0 and stall_check_count > 30:
                logger.debug(f"Clearing escape exclusion after {stall_check_count} clean checks")
                escape_exclude_angle = None
                consecutive_escapes = 0

            # === FRONTIER TARGETING ===
            # Check if we've reached the current target (triggers early recheck)
            x, y = self.robot.pose.x, self.robot.pose.y
            reached_target = False
            if target is not None:
                dist_to_target = math.sqrt((target[0] - x) ** 2 + (target[1] - y) ** 2)
                if dist_to_target < self.TARGET_REACHED_DISTANCE:
                    reached_target = True

            # Stagnation detection — if coverage rate has plateaued, force relocation
            # Uses cumulative sim-elapsed (persists across run() calls)
            cum_elapsed = self._cumulative_elapsed + elapsed
            if (cum_elapsed - self._last_coverage_check_elapsed) >= self.STAGNATION_CHECK_INTERVAL:
                current_pct = self.spatial_map.get_exploration_progress()
                coverage_gain = current_pct - self._last_coverage_pct
                if coverage_gain < self.STAGNATION_MIN_GAIN and self._last_coverage_pct > 0 and not self._relocating:
                    logger.info(
                        f"Coverage stagnant: {coverage_gain*100:.2f}% gain in {self.STAGNATION_CHECK_INTERVAL:.0f}s "
                        f"(total {current_pct*100:.1f}%) — relocating"
                    )
                    self._relocating = True
                    target = None  # force recheck below
                elif self._relocating and coverage_gain >= self.STAGNATION_MIN_GAIN:
                    logger.info(f"Coverage improving again ({coverage_gain*100:.2f}%) — resuming normal")
                    self._relocating = False
                self._last_coverage_pct = current_pct
                self._last_coverage_check_elapsed = cum_elapsed

            time_to_recheck = (elapsed - last_frontier_check) >= self.FRONTIER_RECHECK_INTERVAL
            if target is None or time_to_recheck or reached_target:
                # After escapes, avoid picking frontiers in the direction we just fled from.
                exc_angle = escape_exclude_angle if consecutive_escapes > 0 else None
                exc_cone = math.radians(min(45 + 15 * (consecutive_escapes - 1), 120)) if exc_angle is not None else math.radians(45)
                if self._relocating:
                    # Relocating: use cluster selection to find big unexplored regions
                    new_target = self.spatial_map.find_best_frontier_cluster(
                        x, y,
                        min_cluster_size=3,
                        exclude_angle=exc_angle,
                        exclude_cone=exc_cone,
                    )
                    if new_target:
                        logger.debug(f"Relocation target: ({new_target[0]:.0f}, {new_target[1]:.0f})")
                    else:
                        # Fallback to nearest frontier with min distance
                        new_target = self.spatial_map.find_nearest_frontier(
                            x, y, min_distance=self.RELOCATE_MIN_DISTANCE,
                        )
                    if new_target is None:
                        new_target = self.spatial_map.find_nearest_frontier(x, y, min_distance=0)
                else:
                    # Normal: nearest frontier for efficient scanning
                    new_target = self.spatial_map.find_nearest_frontier(
                        x, y, min_distance=0,
                        exclude_angle=exc_angle,
                        exclude_cone=exc_cone,
                    )
                if new_target:
                    target = new_target
                    self.frontier_targets += 1
                    last_frontier_check = elapsed
                elif target is None:
                    # Check if the map is basically empty (just started)
                    known_pct = self.spatial_map.get_exploration_progress()
                    if known_pct < 0.001:
                        # Map is empty - just drive forward to build initial data
                        pass  # Will fall through to DRIVE section below
                    else:
                        # Map has data but no frontiers - we're done
                        await self.robot.stop()
                        self.elapsed = elapsed
                        logger.info("No frontiers remaining - mapping complete")
                        return NavResult.DONE

            # === STEERING ===
            if target and escape_cooldown <= 0:
                x, y = self.robot.pose.x, self.robot.pose.y
                target_angle = math.atan2(target[1] - y, target[0] - x)
                heading_error = target_angle - self.robot.pose.angle
                heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

                if abs(heading_error) > math.radians(self.HEADING_TOLERANCE_DEG):
                    turn_angle = max(-self.MAX_TURN_DEG, min(self.MAX_TURN_DEG, math.degrees(heading_error)))
                    await self.robot.stop()
                    await self._turn_with_mapping(turn_angle)
                    elapsed += abs(turn_angle) / 45
                    stall_check_count = 0

            # === DRIVE ===
            await self.robot.drive(self.speed)
            await asyncio.sleep(check_interval)
            elapsed += check_interval

        await self.robot.stop()
        self.elapsed = elapsed
        self._cumulative_elapsed += elapsed

        if self._cancelled:
            return NavResult.CANCELLED

        # Time ran out - check if there are still frontiers
        x, y = self.robot.pose.x, self.robot.pose.y
        remaining = self.spatial_map.find_nearest_frontier(x, y, min_distance=0)
        if remaining:
            return NavResult.MAPPING
        return NavResult.DONE

    def _update_map(self, sensors):
        """Update the spatial map from current sensor readings."""
        x, y = self.robot.pose.x, self.robot.pose.y
        heading = self.robot.pose.angle
        self.spatial_map.mark_visited(x, y)

        for name, reading, max_range, angle_offset in [
            ("tof", sensors.ext_tof_mm, 2000, 0),
            ("ultra_c", sensors.ext_ultra_c_mm, 4000, 0),
            ("ultra_l", sensors.ext_ultra_l_mm, 4000, math.radians(15)),
            ("ultra_r", sensors.ext_ultra_r_mm, 4000, math.radians(-15)),
        ]:
            if 0 < reading < max_range:
                self.spatial_map.update_from_sensor(
                    x, y, heading, angle_offset, reading, max_range
                )
            elif reading >= max_range:
                self.spatial_map.update_from_sensor(
                    x, y, heading, angle_offset, max_range, max_range
                )

    async def _turn_with_mapping(self, angle: float, speed: float = 30.0):
        """Turn while updating the map from sensor sweeps."""
        if config.TRAILER_MODE:
            ratio = config.TRAILER_ARC_RATIO
            total_duration = abs(angle) / speed * 2.0
        else:
            ratio = 0.0
            total_duration = abs(angle) / speed
        step = 0.15

        if config.TRAILER_MODE:
            if angle > 0:
                await self.robot.set_wheels(speed * ratio, speed)
            else:
                await self.robot.set_wheels(speed, speed * ratio)
        else:
            if angle > 0:
                await self.robot.set_wheels(-speed, speed)
            else:
                await self.robot.set_wheels(speed, -speed)

        t = 0.0
        while t < total_duration:
            await asyncio.sleep(step)
            t += step
            if self.robot.sensors.ext_connected:
                self._update_map(self.robot.sensors)

        await self.robot.stop()

    def _pick_escape_direction(self, left_dist: int, right_dist: int) -> int:
        """Pick escape turn direction: turn away from the closer side.

        Returns signed angle (positive = left, negative = right).
        """
        from cozmo_interface.robot import SensorData
        left_valid = SensorData._valid_distance(left_dist) if left_dist > 0 else False
        right_valid = SensorData._valid_distance(right_dist) if right_dist > 0 else False

        angle = self.ESCAPE_ANGLE
        if not left_valid and not right_valid:
            # No side data - use map if available
            return self._pick_escape_from_map(angle)
        if not left_valid:
            return angle   # Turn left (away from unknown right)
        if not right_valid:
            return -angle  # Turn right (away from unknown left)
        # Turn toward the more open side
        return angle if left_dist > right_dist else -angle

    def _pick_escape_from_map(self, magnitude: int) -> int:
        """Use map frontiers to choose escape direction."""
        x, y = self.robot.pose.x, self.robot.pose.y
        heading = self.robot.pose.angle
        target = self.spatial_map.find_nearest_frontier(
            x, y, min_distance=0, exclude_angle=heading
        )
        if target:
            target_angle = math.atan2(target[1] - y, target[0] - x)
            heading_error = target_angle - heading
            heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi
            return magnitude if heading_error > 0 else -magnitude
        # No frontier info - random
        import random
        return random.choice([-magnitude, magnitude])

    async def _escape_obstacle(self, left_dist: int, right_dist: int):
        """Escape from obstacle/stall: reverse arc (backs up while turning).

        Uses a reverse arc instead of straight backup + forward turn.
        This sweeps the robot away from the obstacle in one motion,
        avoiding backing straight into walls behind.
        """
        self.robot._escape_in_progress = True
        await self.robot.stop()

        try:
            escape_speed = config.ESCAPE_SPEED
            turn_angle = self._pick_escape_direction(left_dist, right_dist)
            duration = self.ESCAPE_REVERSE_ARC_DURATION

            # Reverse arc: back up while turning simultaneously
            # Use a step loop (not single sleep) so wheel speeds are re-set
            # each tick — protects against sim collision handler zeroing them.
            ratio = self.ESCAPE_ARC_RATIO
            if turn_angle > 0:
                left_spd, right_spd = -escape_speed * ratio, -escape_speed
            else:
                left_spd, right_spd = -escape_speed, -escape_speed * ratio

            step = 0.05
            t = 0.0
            while t < duration:
                await self.robot.set_wheels(left_spd, right_spd)
                await asyncio.sleep(step)
                t += step
                if self.robot.sensors.ext_connected:
                    self._update_map(self.robot.sensors)

            await self.robot.stop()

            logger.info(f"Escape: reverse-arc {duration}s turn_dir={'L' if turn_angle > 0 else 'R'}")
        finally:
            self.robot._escape_in_progress = False
            self.robot.sensors.collision_detected = False

    async def _escape_cliff(self):
        """Escape cliff: longer reverse arc to get well clear."""
        self.robot._escape_in_progress = True
        await self.robot.stop()

        try:
            escape_speed = config.ESCAPE_SPEED
            duration = 4.0  # Extra long for cliff safety

            # Reverse arc with random direction (cliff doesn't indicate side)
            import random
            turn_left = random.choice([True, False])
            ratio = self.ESCAPE_ARC_RATIO
            if turn_left:
                left_spd, right_spd = -escape_speed * ratio, -escape_speed
            else:
                left_spd, right_spd = -escape_speed, -escape_speed * ratio

            step = 0.05
            t = 0.0
            while t < duration:
                await self.robot.set_wheels(left_spd, right_spd)
                await asyncio.sleep(step)
                t += step
                if self.robot.sensors.ext_connected:
                    self._update_map(self.robot.sensors)

            await self.robot.stop()

            logger.info(f"Cliff escape: reverse-arc {duration}s dir={'L' if turn_left else 'R'}")
        finally:
            self.robot._escape_in_progress = False
            self.robot.sensors.collision_detected = False
