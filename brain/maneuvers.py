"""
Maneuvers - sensor-aware escape patterns.

These work against a duck-typed robot interface (set_wheels, stop, sensors)
so they run identically on real CozmoRobot and SimRobot.
"""
import asyncio
import logging
import time
from enum import Enum, auto
from dataclasses import dataclass

import config

logger = logging.getLogger(__name__)


class ManeuverStatus(Enum):
    SUCCESS = auto()
    FAILED = auto()
    ABORTED = auto()


@dataclass
class ManeuverResult:
    status: ManeuverStatus
    message: str = ""
    cycles_used: int = 0
    duration: float = 0.0


class ZigzagManeuver:
    """
    Sensor-aware zigzag escape: reverse arc away from obstacle, then forward
    arc to reorient, repeat until front is clear or max cycles exhausted.

    Reverse-first because the obstacle is ahead - backing up creates space
    before trying to turn. Each reverse-forward cycle ratchets the heading
    sideways, effective with a trailer where single arcs barely change heading.

    Uses set_wheels() for non-blocking wheel control so sensors can be
    polled every ZIGZAG_SENSOR_POLL seconds during arcs.
    """

    # Clear threshold: well above caution distance so we don't immediately
    # re-enter the caution zone and loop forever
    CLEAR_DISTANCE = 350

    def __init__(self, robot, sensors):
        """
        Args:
            robot: CozmoRobot or SimRobot (needs set_wheels, stop)
            sensors: SensorData-compatible object (needs get_front_obstacle_distance)
        """
        self.robot = robot
        self.sensors = sensors

    async def execute(self, direction: str = "left") -> ManeuverResult:
        """
        Run the zigzag escape maneuver.

        Args:
            direction: "left" or "right" - which way to turn away

        Returns:
            ManeuverResult with status and details
        """
        start_time = time.monotonic()
        speed = config.ESCAPE_SPEED
        ratio = config.ZIGZAG_ARC_RATIO
        poll = config.ZIGZAG_SENSOR_POLL
        arc_duration = config.ZIGZAG_ARC_DURATION
        clear_dist = self.CLEAR_DISTANCE
        danger_dist = 80    # WanderBehavior.DANGER_DISTANCE

        sign = 1.0 if direction == "left" else -1.0

        logger.info(f"Zigzag escape starting, direction={direction}")

        try:
            for cycle in range(config.ZIGZAG_MAX_CYCLES):
                logger.debug(f"Zigzag cycle {cycle+1}/{config.ZIGZAG_MAX_CYCLES}, "
                             f"arc_duration={arc_duration:.1f}s")

                # === Phase 1: Reverse arc AWAY from obstacle ===
                # Back up while turning - creates space and starts heading change
                if sign > 0:
                    # Turn left: reverse-right (right wheel faster backward)
                    left_speed = -speed * ratio
                    right_speed = -speed
                else:
                    # Turn right: reverse-left (left wheel faster backward)
                    left_speed = -speed
                    right_speed = -speed * ratio

                await self.robot.set_wheels(left_speed, right_speed)

                phase_elapsed = 0.0
                while phase_elapsed < arc_duration:
                    await asyncio.sleep(poll)
                    phase_elapsed += poll

                    front = self.sensors.get_front_obstacle_distance()
                    if front > clear_dist or front < 0:
                        await self.robot.stop()
                        elapsed = time.monotonic() - start_time
                        logger.info(f"Zigzag SUCCESS: clear after cycle {cycle+1} "
                                    f"rev phase, front={front}mm, {elapsed:.1f}s")
                        return ManeuverResult(
                            ManeuverStatus.SUCCESS,
                            f"Clear after {cycle+1} cycles (rev), front={front}mm",
                            cycles_used=cycle + 1,
                            duration=elapsed,
                        )

                await self.robot.stop()
                await asyncio.sleep(0.05)  # Brief pause between phases

                # === Phase 2: Forward arc to reorient ===
                # Drive forward in the turn direction to complete heading change
                if sign > 0:
                    left_speed = speed * ratio
                    right_speed = speed
                else:
                    left_speed = speed
                    right_speed = speed * ratio

                await self.robot.set_wheels(left_speed, right_speed)

                phase_elapsed = 0.0
                while phase_elapsed < arc_duration:
                    await asyncio.sleep(poll)
                    phase_elapsed += poll

                    front = self.sensors.get_front_obstacle_distance()
                    if front > clear_dist or front < 0:
                        await self.robot.stop()
                        elapsed = time.monotonic() - start_time
                        logger.info(f"Zigzag SUCCESS: clear after cycle {cycle+1} "
                                    f"fwd phase, front={front}mm, {elapsed:.1f}s")
                        return ManeuverResult(
                            ManeuverStatus.SUCCESS,
                            f"Clear after {cycle+1} cycles (fwd), front={front}mm",
                            cycles_used=cycle + 1,
                            duration=elapsed,
                        )
                    if 0 < front < danger_dist:
                        # Getting too close - abort forward phase early
                        logger.debug(f"Zigzag: front={front}mm < danger, "
                                     f"aborting fwd phase early")
                        break

                await self.robot.stop()
                await asyncio.sleep(0.05)

                # Escalate: longer arcs each cycle
                arc_duration += config.ZIGZAG_ARC_ESCALATION

            # Exhausted all cycles
            elapsed = time.monotonic() - start_time
            logger.warning(f"Zigzag FAILED after {config.ZIGZAG_MAX_CYCLES} cycles, "
                           f"{elapsed:.1f}s")
            return ManeuverResult(
                ManeuverStatus.FAILED,
                f"Failed after {config.ZIGZAG_MAX_CYCLES} cycles",
                cycles_used=config.ZIGZAG_MAX_CYCLES,
                duration=elapsed,
            )

        except asyncio.CancelledError:
            await self.robot.stop()
            raise
        except Exception as e:
            await self.robot.stop()
            logger.error(f"Zigzag error: {e}")
            return ManeuverResult(
                ManeuverStatus.ABORTED,
                f"Error: {e}",
                duration=time.monotonic() - start_time,
            )
        finally:
            await self.robot.stop()
