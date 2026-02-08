"""
Robot Behaviors

Low-level behaviors that can be composed into higher-level actions.
Each behavior is an async generator that yields control periodically.

NOTE: WanderBehavior and BehaviorFactory have been removed.
Frontier-based navigation is now in brain/frontier_navigator.py.
The learning rule system has been archived (archive/).
"""
import asyncio
import random
import math
import logging
from typing import Optional
from dataclasses import dataclass
from enum import Enum, auto

import config
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

        duration = abs(self.distance) / self.speed
        actual_speed = self.speed if self.distance > 0 else -self.speed

        await self.robot.drive(actual_speed)

        elapsed = 0.0
        check_interval = 0.1

        while elapsed < duration and not self.is_cancelled:
            await asyncio.sleep(check_interval)
            elapsed += check_interval

            if self.robot.sensors.cliff_detected:
                await self.robot.stop()
                return BehaviorResult(BehaviorStatus.INTERRUPTED, "Cliff detected")

            if self.robot.sensors.is_picked_up:
                await self.robot.stop()
                return BehaviorResult(BehaviorStatus.INTERRUPTED, "Robot picked up")

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
        head_angles = [-0.2, 0.0, 0.3, 0.6]
        body_angles = [0, 45, 90, 135, 180, -135, -90, -45]

        self.images = []

        await self.robot.set_head_angle(0.1)
        await asyncio.sleep(0.3)

        for body_angle in body_angles:
            if self.is_cancelled:
                break

            if body_angle != 0:
                await self.robot.turn(body_angle)

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

            if body_angle != 0:
                await self.robot.turn(-body_angle)

        await self.robot.set_head_angle(0.0)

        return BehaviorResult(
            BehaviorStatus.COMPLETED if not self.is_cancelled else BehaviorStatus.INTERRUPTED,
            f"Captured {len(self.images)} views",
            {"images": self.images}
        )


class GoToPoseBehavior(Behavior):
    """
    Navigate to a target pose using simple dead-reckoning.
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

            if distance < self.position_tolerance:
                angle_diff = self.target.angle - current.angle
                if abs(angle_diff) > self.angle_tolerance:
                    await self.robot.turn(math.degrees(angle_diff))

                return BehaviorResult(
                    BehaviorStatus.COMPLETED,
                    f"Reached target within {distance:.1f}mm"
                )

            target_heading = math.atan2(dy, dx)
            heading_error = target_heading - current.angle

            while heading_error > math.pi:
                heading_error -= 2 * math.pi
            while heading_error < -math.pi:
                heading_error += 2 * math.pi

            if abs(heading_error) > self.angle_tolerance:
                await self.robot.turn(math.degrees(heading_error))
                await asyncio.sleep(0.2)

            drive_distance = min(distance, 100)
            drive_behavior = DriveDistanceBehavior(self.robot, drive_distance)
            result = await drive_behavior.run()

            if result.status == BehaviorStatus.INTERRUPTED:
                return result

            iteration += 1
            await asyncio.sleep(0.1)

        if self.is_cancelled:
            return BehaviorResult(BehaviorStatus.INTERRUPTED, "Cancelled")

        return BehaviorResult(BehaviorStatus.FAILED, "Max iterations reached")


class AvoidObstacleBehavior(Behavior):
    """React to an obstacle by backing up and turning away."""

    def __init__(self, robot: CozmoRobot, turn_direction: str = "random"):
        super().__init__(robot)
        self.turn_direction = turn_direction

    async def run(self) -> BehaviorResult:
        await self.robot.stop()

        await self.robot.drive(-50, duration=0.5)
        await asyncio.sleep(0.5)

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
