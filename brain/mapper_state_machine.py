"""
Mapper State Machine

Simplified state machine for the mapping platform architecture.
4 states: MAPPING, CAPTURING, REVIEWING, DONE.

The robot's job is to build the spatial map. The system (LLM + DB) learns
about the environment from the completed map.
"""
import asyncio
import logging
from enum import Enum, auto
from datetime import datetime
from typing import Optional, TYPE_CHECKING

import config

if TYPE_CHECKING:
    from memory.spatial_map import SpatialMap
    from memory.experience_logger import ExperienceLogger
    from memory.map_annotations import MapAnnotationStore
    from brain.session_reviewer import SessionReviewer

from brain.frontier_navigator import FrontierNavigator, NavResult

logger = logging.getLogger(__name__)


class MapperState(Enum):
    IDLE = auto()
    MAPPING = auto()
    CAPTURING = auto()
    REVIEWING = auto()
    DONE = auto()
    ERROR = auto()


class MapperStateMachine:
    """
    Runs the mapping cycle:
    1. MAPPING - FrontierNavigator drives toward unexplored areas
    2. CAPTURING - Take images at new areas (when robot has camera)
    3. REVIEWING - Post-session LLM reviews map + images
    4. DONE - No more frontiers

    Safety checks (picked up, low battery, cliff) handled by FrontierNavigator
    and by this machine's main loop.
    """

    # How long each mapping segment runs before checking state
    MAPPING_SEGMENT_DURATION = 30.0  # seconds per FrontierNavigator run

    # Image capture settings
    CAPTURE_INTERVAL = 120.0  # seconds between image captures
    CAPTURE_HEAD_ANGLE = 0.35  # radians - look forward

    def __init__(
        self,
        robot,
        spatial_map: "SpatialMap",
        experience_logger: "ExperienceLogger" = None,
        session_reviewer: "SessionReviewer" = None,
        annotation_store: "MapAnnotationStore" = None,
    ):
        self.robot = robot
        self.spatial_map = spatial_map
        self.experience_logger = experience_logger
        self.session_reviewer = session_reviewer
        self.annotation_store = annotation_store

        self.state = MapperState.IDLE
        self._running = False
        self._paused = False
        self._navigator: Optional[FrontierNavigator] = None

        # Stats
        self.total_escapes = 0
        self.total_frontier_targets = 0
        self.mapping_time = 0.0
        self.images_captured = 0
        self._last_capture_time = 0.0
        self._start_time: Optional[datetime] = None

    @property
    def is_running(self) -> bool:
        return self._running

    def pause(self):
        if not self._paused:
            logger.info("Mapper paused")
            self._paused = True
            if self._navigator:
                self._navigator.cancel()

    def resume(self):
        if self._paused:
            logger.info("Mapper resumed")
            self._paused = False

    @property
    def is_paused(self) -> bool:
        return self._paused

    async def start(self):
        """Start the mapping state machine."""
        if self._running:
            logger.warning("Mapper already running")
            return

        self._running = True
        self._start_time = datetime.now()
        logger.info("MapperStateMachine starting")

        try:
            await self._run_loop()
        except Exception as e:
            logger.error(f"Mapper error: {e}", exc_info=True)
            self.state = MapperState.ERROR
        finally:
            self._running = False

    async def stop(self):
        """Stop the mapping state machine."""
        logger.info("Stopping mapper")
        self._running = False
        if self._navigator:
            self._navigator.cancel()
        await self.robot.stop()

    async def _run_loop(self):
        """Main state machine loop."""
        self.state = MapperState.MAPPING

        while self._running:
            if self._paused:
                await asyncio.sleep(0.5)
                continue

            # Safety: picked up
            if self.robot.sensors.is_picked_up:
                await self.robot.stop()
                logger.info("Robot picked up - waiting")
                await asyncio.sleep(1.0)
                continue

            # Safety: low battery
            bv = self.robot.sensors.battery_voltage
            if bv > 0 and bv < config.LOW_BATTERY_VOLTAGE:
                await self.robot.stop()
                logger.warning(f"Low battery ({bv:.2f}V)")
                await asyncio.sleep(5.0)
                continue

            if self.state == MapperState.MAPPING:
                await self._do_mapping()
            elif self.state == MapperState.CAPTURING:
                await self._do_capturing()
            elif self.state == MapperState.REVIEWING:
                await self._do_reviewing()
            elif self.state == MapperState.DONE:
                break
            elif self.state == MapperState.ERROR:
                await self.robot.stop()
                await asyncio.sleep(1.0)
            else:
                await asyncio.sleep(0.5)

    async def _do_mapping(self):
        """Run one segment of frontier navigation."""
        # Reuse navigator to preserve stagnation/relocation state across runs
        if self._navigator is None:
            self._navigator = FrontierNavigator(
                robot=self.robot,
                spatial_map=self.spatial_map,
                duration=self.MAPPING_SEGMENT_DURATION,
                experience_logger=self.experience_logger,
            )

        result = await self._navigator.run()

        # Accumulate stats
        self.total_escapes = self._navigator.escapes
        self.total_frontier_targets = self._navigator.frontier_targets
        self.mapping_time += self._navigator.elapsed

        if result == NavResult.DONE:
            logger.info("All frontiers explored - transitioning to REVIEWING")
            self.state = MapperState.REVIEWING
        elif result == NavResult.PICKED_UP:
            # Will be caught by safety check next loop
            pass
        elif result == NavResult.CANCELLED:
            # Machine is stopping
            pass
        else:
            # MAPPING - still frontiers, check if we should capture images
            if self._should_capture():
                self.state = MapperState.CAPTURING
            # Otherwise continue mapping (state stays MAPPING)

    async def _do_capturing(self):
        """Capture an image at the current position for later review."""
        try:
            # Set head angle for good view
            await self.robot.set_head_angle(self.CAPTURE_HEAD_ANGLE)
            await asyncio.sleep(0.3)

            image = await self.robot.capture_image()
            if image:
                self.images_captured += 1
                self._last_capture_time = self.mapping_time

                # Save image
                images_dir = config.DATA_DIR / "mapping_images"
                images_dir.mkdir(parents=True, exist_ok=True)
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                x, y = self.robot.pose.x, self.robot.pose.y
                filepath = images_dir / f"map_{timestamp}_{x:.0f}_{y:.0f}.jpg"
                image.save(str(filepath), "JPEG", quality=85)
                logger.info(f"Captured image #{self.images_captured} at ({x:.0f}, {y:.0f})")

        except Exception as e:
            logger.debug(f"Image capture failed: {e}")

        # Back to mapping
        self.state = MapperState.MAPPING

    async def _do_reviewing(self):
        """Run post-session LLM review of the map."""
        if self.session_reviewer and self.annotation_store:
            try:
                logger.info("Running post-session map review...")
                annotations = await self.session_reviewer.review_session(
                    spatial_map=self.spatial_map,
                    robot_x=self.robot.pose.x,
                    robot_y=self.robot.pose.y,
                    escapes=self.total_escapes,
                    mapping_time=self.mapping_time,
                    images_captured=self.images_captured,
                )
                if annotations:
                    for ann in annotations:
                        self.annotation_store.add_annotation(
                            x=ann.get('x', 0),
                            y=ann.get('y', 0),
                            label=ann.get('label', ''),
                            annotation_type=ann.get('type', 'observation'),
                            details=ann.get('details', ''),
                        )
                    logger.info(f"Session review produced {len(annotations)} annotations")
            except Exception as e:
                logger.error(f"Session review failed: {e}")
        else:
            logger.info("No session reviewer configured - skipping review")

        self.state = MapperState.DONE

    def _should_capture(self) -> bool:
        """Check if it's time to capture an image."""
        if not hasattr(self.robot, 'capture_image'):
            return False
        return (self.mapping_time - self._last_capture_time) >= self.CAPTURE_INTERVAL

    def get_status(self) -> dict:
        """Get current status for display/logging."""
        visited_pct = self.spatial_map.get_visited_percentage() * 100
        explored_pct = self.spatial_map.get_exploration_progress() * 100
        return {
            "state": self.state.name,
            "mapping_time": self.mapping_time,
            "visited_pct": visited_pct,
            "explored_pct": explored_pct,
            "escapes": self.total_escapes,
            "frontier_targets": self.total_frontier_targets,
            "images_captured": self.images_captured,
        }
