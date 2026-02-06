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
from pathlib import Path
import uuid

from perception.camera import CameraCapture
from llm.client import LLMClient
from memory.experience_db import ExperienceDB, Experience
from audio.voice import CozmoVoice
import config

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
        enabled: bool = True,
        save_images: bool = True,
        speak_descriptions: bool = True  # Have Cozmo speak what it sees
    ):
        self.robot = robot
        self.llm = llm_client
        self.experiences = experience_db
        self.capture_interval = capture_interval
        self.enabled = enabled
        self.save_images = save_images
        self.speak_descriptions = speak_descriptions

        self.camera = CameraCapture(robot)
        self.voice = CozmoVoice(robot) if speak_descriptions else None
        self._running = False
        self._task: Optional[asyncio.Task] = None

        # Recent observations (in-memory)
        self.recent_observations: List[Observation] = []
        self.max_recent = 20

        # Image save directory
        self.image_dir = config.DATA_DIR / "images"
        if save_images:
            self.image_dir.mkdir(parents=True, exist_ok=True)

        # Head angle for looking forward (radians)
        # Range: -0.44 (down) to 0.78 (up), 0.0 = straight ahead
        self.capture_head_angle = 0.65  # Look up to see past sensor pod wires

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
        # Raise lift to clear the sensor pod from camera view (if needed)
        if config.LIFT_FOR_CAMERA:
            try:
                await self.robot.set_lift_height(1.0)
                await asyncio.sleep(1.0)  # Wait for lift to fully raise (32mm -> 92mm)
            except Exception as e:
                logger.debug(f"Could not raise lift: {e}")

        # Headlight disabled to save battery
        # try:
        #     await self.robot.set_head_light(True)
        # except Exception as e:
        #     logger.debug(f"Could not turn on headlight: {e}")

        # Adjust head to look forward before capturing
        try:
            await self.robot.set_head_angle(self.capture_head_angle)
            await asyncio.sleep(0.5)  # Wait for head to settle
        except Exception as e:
            logger.debug(f"Could not set head angle: {e}")

        # Capture image (get both raw and base64)
        raw_image = await self.camera.capture()

        # Headlight disabled to save battery
        # try:
        #     await self.robot.set_head_light(False)
        # except Exception as e:
        #     logger.debug(f"Could not turn off headlight: {e}")

        # Lower lift back down before moving (if it was raised)
        if config.LIFT_FOR_CAMERA:
            try:
                await self.robot.set_lift_height(0.0)
                await asyncio.sleep(0.3)  # Wait for lift to lower
            except Exception as e:
                logger.debug(f"Could not lower lift: {e}")

        if not raw_image:
            logger.debug("No image captured")
            return

        image_base64 = self.camera.image_to_base64(
            self.camera.resize_for_llm(raw_image, max_size=384)
        )

        # Get robot position
        pos_x = self.robot.pose.x
        pos_y = self.robot.pose.y
        timestamp = datetime.now()

        # Save image to disk
        if self.save_images:
            await self._save_image(raw_image, pos_x, pos_y, timestamp)

        # Ask LLaVA what it sees
        logger.info("Analyzing image with LLaVA...")
        description = await self.llm.describe_image(
            image_base64,
            context=f"Robot is at position ({pos_x:.0f}, {pos_y:.0f})"
        )

        if not description:
            logger.warning("No description from LLaVA")
            return

        # Save description alongside image
        if self.save_images:
            await self._save_description(description, pos_x, pos_y, timestamp)

        # Have Cozmo speak the description (if voice is enabled)
        if self.voice and self.speak_descriptions and config.ENABLE_VOICE:
            try:
                # Shorten very long descriptions for speech
                speech_text = description[:150] if len(description) > 150 else description
                await self.voice.speak(speech_text)
            except Exception as e:
                logger.debug(f"Could not speak description: {e}")

        # Create observation
        observation = Observation(
            id=str(uuid.uuid4()),
            description=description,
            location_x=pos_x,
            location_y=pos_y,
            timestamp=timestamp
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

    async def _save_image(self, image, x: float, y: float, timestamp: datetime):
        """Save image to disk"""
        try:
            filename = f"{timestamp.strftime('%Y-%m-%d_%H-%M-%S')}_x{x:.0f}_y{y:.0f}.jpg"
            filepath = self.image_dir / filename
            image.save(filepath, "JPEG", quality=90)
            logger.debug(f"Saved image: {filepath}")
        except Exception as e:
            logger.error(f"Failed to save image: {e}")

    async def _save_description(self, description: str, x: float, y: float, timestamp: datetime):
        """Save description text file alongside image"""
        try:
            filename = f"{timestamp.strftime('%Y-%m-%d_%H-%M-%S')}_x{x:.0f}_y{y:.0f}.txt"
            filepath = self.image_dir / filename
            filepath.write_text(f"Position: ({x:.0f}, {y:.0f})\nTime: {timestamp}\n\n{description}")
            logger.debug(f"Saved description: {filepath}")
        except Exception as e:
            logger.error(f"Failed to save description: {e}")

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
