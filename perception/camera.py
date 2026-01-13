"""
Camera module for capturing and processing images from Cozmo
"""
import asyncio
import logging
import base64
import io
from typing import Optional
from datetime import datetime
from PIL import Image

logger = logging.getLogger(__name__)


class CameraCapture:
    """
    Handles camera capture from Cozmo and image processing.
    """

    def __init__(self, robot):
        self.robot = robot
        self._last_capture: Optional[Image.Image] = None
        self._last_capture_time: Optional[datetime] = None

    async def capture(self) -> Optional[Image.Image]:
        """
        Capture a single frame from Cozmo's camera.

        Returns:
            PIL Image or None if capture failed
        """
        try:
            image = await self.robot.capture_image()
            if image:
                self._last_capture = image
                self._last_capture_time = datetime.now()
                logger.debug(f"Captured image: {image.size}")
            return image
        except Exception as e:
            logger.error(f"Camera capture failed: {e}")
            return None

    @property
    def last_capture(self) -> Optional[Image.Image]:
        return self._last_capture

    @property
    def last_capture_time(self) -> Optional[datetime]:
        return self._last_capture_time

    @staticmethod
    def image_to_base64(image: Image.Image, format: str = "JPEG", quality: int = 85) -> str:
        """
        Convert PIL Image to base64 string for LLM.

        Args:
            image: PIL Image
            format: Image format (JPEG, PNG)
            quality: JPEG quality (1-100)

        Returns:
            Base64 encoded string
        """
        buffer = io.BytesIO()

        # Convert to RGB if necessary (for JPEG)
        if format == "JPEG" and image.mode != "RGB":
            image = image.convert("RGB")

        image.save(buffer, format=format, quality=quality)
        buffer.seek(0)

        return base64.b64encode(buffer.read()).decode('utf-8')

    @staticmethod
    def resize_for_llm(image: Image.Image, max_size: int = 512) -> Image.Image:
        """
        Resize image for LLM processing (smaller = faster).

        Args:
            image: PIL Image
            max_size: Maximum dimension

        Returns:
            Resized PIL Image
        """
        width, height = image.size

        if width <= max_size and height <= max_size:
            return image

        if width > height:
            new_width = max_size
            new_height = int(height * (max_size / width))
        else:
            new_height = max_size
            new_width = int(width * (max_size / height))

        return image.resize((new_width, new_height), Image.Resampling.LANCZOS)

    async def capture_for_llm(self, max_size: int = 512) -> Optional[str]:
        """
        Capture an image and prepare it for LLM (resized, base64 encoded).

        Args:
            max_size: Maximum image dimension

        Returns:
            Base64 encoded image string, or None if capture failed
        """
        image = await self.capture()

        if image is None:
            return None

        # Resize for faster LLM processing
        resized = self.resize_for_llm(image, max_size)

        # Convert to base64
        return self.image_to_base64(resized)
