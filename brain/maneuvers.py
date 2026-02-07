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
    Sensor-aware zigzag escape: forward arc one direction, reverse arc the
    other, repeat until front is clear or max cycles exhausted.

    Each forward-reverse cycle ratchets the heading sideways - effective
    with a trailer where single arcs barely change heading.

    Uses set_wheels() for non-blocking wheel control so sensors can be
    polled every ZIGZAG_SENSOR_POLL seconds during arcs.
    """

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
            direction: "left" or "right" - initial forward arc direction

        Returns:
            ManeuverResult with status and details
        """
        start_time = time.monotonic()
        speed = config.ESCAPE_SPEED
        ratio = config.ZIGZAG_ARC_RATIO
        poll = config.ZIGZAG_SENSOR_POLL
        arc_duration = config.ZIGZAG_ARC_DURATION
        caution_dist = 250  # WanderBehavior.CAUTION_DISTANCE
        danger_dist = 80    # WanderBehavior.DANGER_DISTANCE

        sign = 1.0 if direction == "left" else -1.0

        logger.info(f"Zigzag escape starting, direction={direction}")

        try:
            for cycle in range(config.ZIGZAG_MAX_CYCLES):
                logger.debug(f"Zigzag cycle {cycle+1}/{config.ZIGZAG_MAX_CYCLES}, "
                             f"arc_duration={arc_duration:.1f}s")

                # === Phase 1: Forward arc in chosen direction ===
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
                    if front > caution_dist or front < 0:
                        # Clear! (or no sensor data, treat as clear)
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
                        # Too close - abort this forward phase early
                        logger.debug(f"Zigzag: front={front}mm < danger, "
                                     f"aborting fwd phase early")
                        break

                await self.robot.stop()
                await asyncio.sleep(0.05)  # Brief pause between phases

                # === Phase 2: Reverse arc in OPPOSITE direction ===
                if sign > 0:
                    # Was going forward-left, now reverse-right
                    left_speed = -speed
                    right_speed = -speed * ratio
                else:
                    # Was going forward-right, now reverse-left
                    left_speed = -speed * ratio
                    right_speed = -speed

                await self.robot.set_wheels(left_speed, right_speed)

                phase_elapsed = 0.0
                while phase_elapsed < arc_duration:
                    await asyncio.sleep(poll)
                    phase_elapsed += poll

                    front = self.sensors.get_front_obstacle_distance()
                    if front > caution_dist or front < 0:
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
