"""
Vision Observer

Periodically captures images and uses LLaVA to describe what's seen.
Stores observations with location data.
"""
import asyncio
import logging
from typing import Optional, List
from dataclasses import dataclass
from datetime import datetime
import uuid

from perception.camera import CameraCapture
from llm.client import LLMClient
from memory.experience_db import ExperienceDB, Experience

logger = logging.getLogger(__name__)


@dataclass
class Observation:
    """A single visual observation"""
    id: str
    description: str
    location_x: float
    location_y: float
    timestamp: datetime
    image_base64: Optional[str] = None  # Keep image if needed


class VisionObserver:
    """
    Handles periodic visual observation of the environment.

    - Captures images from Cozmo's camera
    - Sends to LLaVA for description
    - Stores observations with location
    """

    def __init__(
        self,
        robot,
        llm_client: LLMClient,
        experience_db: ExperienceDB = None,
        capture_interval: float = 15.0,  # seconds between captures
        enabled: bool = True
    ):
        self.robot = robot
        self.llm = llm_client
        self.experiences = experience_db
        self.capture_interval = capture_interval
        self.enabled = enabled

        self.camera = CameraCapture(robot)
        self._running = False
        self._task: Optional[asyncio.Task] = None

        # Recent observations (in-memory)
        self.recent_observations: List[Observation] = []
        self.max_recent = 20

    async def start(self):
        """Start the vision observer background task"""
        if self._running:
            return

        self._running = True
        self._task = asyncio.create_task(self._observe_loop())
        logger.info(f"Vision observer started (interval: {self.capture_interval}s)")

    async def stop(self):
        """Stop the vision observer"""
        self._running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
        logger.info("Vision observer stopped")

    async def _observe_loop(self):
        """Main observation loop"""
        while self._running:
            if self.enabled:
                try:
                    await self._capture_and_analyze()
                except Exception as e:
                    logger.error(f"Vision observation failed: {e}")

            await asyncio.sleep(self.capture_interval)

    async def _capture_and_analyze(self):
        """Capture an image and analyze it with LLaVA"""
        # Capture image
        image_base64 = await self.camera.capture_for_llm(max_size=384)

        if not image_base64:
            logger.debug("No image captured")
            return

        # Get robot position
        pos_x = self.robot.pose.x
        pos_y = self.robot.pose.y

        # Ask LLaVA what it sees
        logger.info("Analyzing image with LLaVA...")
        description = await self.llm.describe_image(
            image_base64,
            context=f"Robot is at position ({pos_x:.0f}, {pos_y:.0f})"
        )

        if not description:
            logger.warning("No description from LLaVA")
            return

        # Create observation
        observation = Observation(
            id=str(uuid.uuid4()),
            description=description,
            location_x=pos_x,
            location_y=pos_y,
            timestamp=datetime.now()
        )

        # Store in recent list
        self.recent_observations.append(observation)
        if len(self.recent_observations) > self.max_recent:
            self.recent_observations.pop(0)

        # Store in experience DB if available
        if self.experiences:
            exp = Experience(
                id=observation.id,
                description=description,
                location_x=pos_x,
                location_y=pos_y,
                timestamp=observation.timestamp,
                experience_type="visual_observation",
                importance=0.5
            )
            await self.experiences.add_experience(exp)

        logger.info(f"Observation at ({pos_x:.0f}, {pos_y:.0f}): {description[:80]}...")

        return observation

    async def capture_now(self) -> Optional[Observation]:
        """
        Capture and analyze immediately (on-demand).

        Returns:
            Observation or None
        """
        return await self._capture_and_analyze()

    def get_recent_descriptions(self, n: int = 5) -> List[str]:
        """Get the N most recent observation descriptions"""
        return [obs.description for obs in self.recent_observations[-n:]]

    def get_observations_near(
        self,
        x: float,
        y: float,
        radius: float = 200.0
    ) -> List[Observation]:
        """Get observations within radius of a point"""
        nearby = []
        for obs in self.recent_observations:
            dist = ((obs.location_x - x)**2 + (obs.location_y - y)**2)**0.5
            if dist <= radius:
                nearby.append(obs)
        return nearby
