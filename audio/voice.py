"""
Cozmo Voice Generator

Generates robot-like speech audio that can be played through Cozmo's speaker.
Based on code by Jim Buschman.
"""
import asyncio
import logging
import tempfile
import os
from pathlib import Path
from typing import Optional
import numpy as np

try:
    import pyttsx3
    import librosa
    import soundfile as sf
    HAS_AUDIO_DEPS = True
except ImportError:
    HAS_AUDIO_DEPS = False

import config

logger = logging.getLogger(__name__)


class CozmoVoice:
    """
    Generates Cozmo-like voice audio from text.

    Uses pyttsx3 for TTS, then applies pitch shifting and effects
    to make it sound more robotic.
    """

    def __init__(self, robot=None):
        self.robot = robot
        self._engine = None
        self._temp_dir = config.DATA_DIR / "audio_temp"
        self._temp_dir.mkdir(parents=True, exist_ok=True)

        # Voice settings
        self.pitch_shift_steps = 4  # Shift pitch up
        self.speech_rate = 150  # Words per minute

        if HAS_AUDIO_DEPS:
            self._init_engine()
        else:
            logger.warning("Audio dependencies not installed (pyttsx3, librosa, soundfile)")

    def _init_engine(self):
        """Initialize the TTS engine"""
        try:
            self._engine = pyttsx3.init()
            self._engine.setProperty('rate', self.speech_rate)

            # Try to use a more robotic voice if available
            voices = self._engine.getProperty('voices')
            if voices:
                # Prefer a male voice for more robotic sound
                for voice in voices:
                    if 'david' in voice.name.lower() or 'male' in voice.name.lower():
                        self._engine.setProperty('voice', voice.id)
                        break

            logger.info("Voice engine initialized")
        except Exception as e:
            logger.error(f"Failed to initialize voice engine: {e}")
            self._engine = None

    def _generate_raw_audio(self, text: str) -> Optional[str]:
        """Generate raw TTS audio to a temp file"""
        if not self._engine:
            return None

        temp_path = self._temp_dir / "raw_speech.wav"

        try:
            self._engine.save_to_file(text, str(temp_path))
            self._engine.runAndWait()
            return str(temp_path)
        except Exception as e:
            logger.error(f"TTS generation failed: {e}")
            return None

    def _apply_robot_effects(self, input_path: str) -> Optional[str]:
        """Apply pitch shifting and effects to make it sound robotic"""
        output_path = self._temp_dir / "robot_speech.wav"

        try:
            # Load audio
            y, sr = librosa.load(input_path, sr=None)

            # Pitch shift up to sound more robotic
            y_shifted = librosa.effects.pitch_shift(
                y, sr=sr, n_steps=self.pitch_shift_steps
            )

            # Add slight time stretch for robotic feel
            y_stretched = librosa.effects.time_stretch(y_shifted, rate=1.1)

            # Normalize
            y_normalized = librosa.util.normalize(y_stretched)

            # Save
            sf.write(str(output_path), y_normalized, sr)

            return str(output_path)
        except Exception as e:
            logger.error(f"Audio processing failed: {e}")
            return None

    def _convert_for_cozmo(self, input_path: str) -> Optional[str]:
        """Convert audio to Cozmo-compatible format (22kHz, 16-bit mono)"""
        output_path = self._temp_dir / "cozmo_speech.wav"

        try:
            # Load and resample to 22kHz
            y, sr = librosa.load(input_path, sr=22050, mono=True)

            # Clip to valid range and convert to 16-bit integer
            # librosa can return values slightly outside [-1.0, 1.0] after processing
            y_clipped = np.clip(y, -1.0, 1.0)
            y_int = (y_clipped * 32767).astype(np.int16)

            # Save as WAV
            sf.write(str(output_path), y_int, 22050, subtype='PCM_16')

            return str(output_path)
        except Exception as e:
            logger.error(f"Cozmo conversion failed: {e}")
            return None

    def generate_speech(self, text: str) -> Optional[str]:
        """
        Generate Cozmo-compatible speech audio from text.

        Args:
            text: Text to speak

        Returns:
            Path to WAV file, or None if generation failed
        """
        if not HAS_AUDIO_DEPS:
            logger.warning("Cannot generate speech - missing dependencies")
            return None

        # Step 1: Generate raw TTS
        raw_path = self._generate_raw_audio(text)
        if not raw_path or not os.path.exists(raw_path):
            return None

        # Step 2: Apply robot effects
        robot_path = self._apply_robot_effects(raw_path)
        if not robot_path:
            return None

        # Step 3: Convert for Cozmo
        cozmo_path = self._convert_for_cozmo(robot_path)

        return cozmo_path

    async def speak(self, text: str) -> bool:
        """
        Generate speech and play it through Cozmo's speaker.

        Args:
            text: Text to speak

        Returns:
            True if successful, False otherwise
        """
        if not self.robot:
            logger.warning("No robot connected - cannot play audio")
            return False

        # Generate the audio file
        logger.info(f"Generating speech: '{text[:50]}...'")
        audio_path = self.generate_speech(text)
        if not audio_path:
            logger.warning(f"Failed to generate speech for: {text[:50]}...")
            return False

        logger.info(f"Audio generated: {audio_path}")

        # Play through Cozmo
        try:
            await self.robot.play_audio(audio_path)
            logger.info(f"Spoke: {text[:50]}...")
            return True
        except Exception as e:
            logger.error(f"Failed to play audio on Cozmo: {e}")
            return False

    def cleanup(self):
        """Clean up temporary files"""
        try:
            for f in self._temp_dir.glob("*.wav"):
                f.unlink()
        except Exception as e:
            logger.debug(f"Cleanup error: {e}")


# Quick test
async def test_voice():
    """Test voice generation"""
    voice = CozmoVoice()

    test_texts = [
        "I see a table with some objects on it.",
        "Exploring the living room now.",
        "Battery is getting low.",
    ]

    for text in test_texts:
        print(f"Generating: {text}")
        path = voice.generate_speech(text)
        if path:
            print(f"  -> Saved to: {path}")
        else:
            print("  -> Failed!")

    voice.cleanup()


if __name__ == "__main__":
    asyncio.run(test_voice())
