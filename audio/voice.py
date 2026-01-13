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
            logger.error("TTS engine not initialized")
            return None

        temp_path = self._temp_dir / "raw_speech.wav"

        try:
            self._engine.save_to_file(text, str(temp_path))
            self._engine.runAndWait()
            logger.debug(f"Generated raw TTS: {temp_path}")
            return str(temp_path)
        except Exception as e:
            logger.error(f"TTS generation failed: {e}")
            try:
                from brain.personality import error_log
                error_log.log_error("voice", f"TTS generation failed for: {text[:50]}", e)
            except:
                pass
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

    def _convert_for_cozmo(self, input_path: str) -> list[str]:
        """
        Convert audio to Cozmo-compatible format (22kHz, 16-bit mono).

        Splits long audio into chunks since pycozmo buffer can't handle
        audio longer than ~2 seconds.

        Returns:
            List of paths to WAV file chunks, or empty list if failed
        """
        # Max duration per chunk in seconds - pycozmo buffer limitation
        # Using 1.5s instead of 2s for extra safety margin
        CHUNK_DURATION = 1.5
        sr = 22050

        try:
            # Load and resample to 22kHz
            y, orig_sr = librosa.load(input_path, sr=sr, mono=True)

            # Normalize the audio
            y = librosa.util.normalize(y)

            # Split into chunks
            chunk_samples = int(CHUNK_DURATION * sr)
            num_chunks = max(1, (len(y) + chunk_samples - 1) // chunk_samples)

            saved_files = []
            silence_threshold = 0.001

            for i in range(num_chunks):
                start = i * chunk_samples
                end = min(start + chunk_samples, len(y))
                chunk = y[start:end].copy()  # IMPORTANT: copy to avoid modifying original

                # Skip very quiet chunks
                if np.abs(chunk).mean() < silence_threshold:
                    continue

                # Add fade in/out to avoid clicks (only at chunk boundaries, not speech boundaries)
                fade_len = int(sr * 0.01)  # 10ms fade - shorter to preserve more audio
                if len(chunk) > fade_len * 2:
                    # Only fade in on first chunk
                    if i == 0:
                        chunk[:fade_len] *= np.linspace(0, 1, fade_len)
                    # Only fade out on last chunk
                    if i == num_chunks - 1:
                        chunk[-fade_len:] *= np.linspace(1, 0, fade_len)

                # Apply compression for more consistent volume (from user's test program)
                threshold = 0.2
                ratio = 2.0
                makeup_gain = 1.3

                gain_reduction = np.zeros_like(chunk)
                mask = np.abs(chunk) > threshold
                gain_reduction[mask] = (np.abs(chunk[mask]) - threshold) * (1 - 1/ratio)

                compressed = chunk.copy()
                compressed[mask] = np.sign(chunk[mask]) * (np.abs(chunk[mask]) - gain_reduction[mask])
                compressed = compressed * makeup_gain

                # Clip and convert to 16-bit
                chunk_clipped = np.clip(compressed, -0.98, 0.98)
                chunk_int16 = (chunk_clipped * 32767).astype(np.int16)

                # Save chunk with part number
                chunk_path = self._temp_dir / f"cozmo_speech_part{i+1}.wav"
                sf.write(str(chunk_path), chunk_int16, sr, subtype='PCM_16')
                saved_files.append(str(chunk_path))

                logger.debug(f"Saved chunk {i+1}: {chunk_path} ({len(chunk)/sr:.2f}s)")

            total_duration = len(y) / sr
            logger.info(f"Converted audio: {len(saved_files)} chunk(s), {total_duration:.1f}s total")
            return saved_files

        except Exception as e:
            logger.error(f"Cozmo conversion failed: {e}")
            try:
                from brain.personality import error_log
                error_log.log_error("voice", f"Cozmo conversion failed: {input_path}", e)
            except:
                pass
            return []

    def generate_speech(self, text: str) -> list[str]:
        """
        Generate Cozmo-compatible speech audio from text.

        Args:
            text: Text to speak

        Returns:
            List of paths to WAV file chunks, or empty list if failed
        """
        if not HAS_AUDIO_DEPS:
            logger.warning("Cannot generate speech - missing dependencies")
            return []

        # Step 1: Generate raw TTS
        raw_path = self._generate_raw_audio(text)
        if not raw_path or not os.path.exists(raw_path):
            return []

        # Step 2: Apply robot effects
        robot_path = self._apply_robot_effects(raw_path)
        if not robot_path:
            return []

        # Step 3: Convert for Cozmo (returns list of chunk paths)
        chunk_paths = self._convert_for_cozmo(robot_path)

        return chunk_paths

    async def speak(self, text: str) -> bool:
        """
        Generate speech and play it through Cozmo's speaker.

        Handles chunked audio by playing each chunk sequentially.

        Args:
            text: Text to speak

        Returns:
            True if successful, False otherwise
        """
        if not self.robot:
            logger.warning("No robot connected - cannot play audio")
            return False

        # Generate the audio files (may be multiple chunks)
        logger.info(f"Generating speech: '{text[:50]}...'")
        audio_paths = self.generate_speech(text)
        if not audio_paths:
            logger.warning(f"Failed to generate speech for: {text[:50]}...")
            return False

        logger.info(f"Audio generated: {len(audio_paths)} chunk(s)")

        # Play each chunk through Cozmo sequentially
        success = True
        for i, audio_path in enumerate(audio_paths):
            try:
                # Add delay between chunks to let pycozmo buffer clear
                if i > 0:
                    await asyncio.sleep(0.3)  # 300ms gap between chunks

                logger.debug(f"Playing chunk {i+1}/{len(audio_paths)}: {audio_path}")
                await self.robot.play_audio(audio_path, wait=True)
            except Exception as e:
                logger.error(f"Failed to play audio chunk {i+1} on Cozmo: {e}")
                try:
                    from brain.personality import error_log
                    error_log.log_error("voice", f"Failed to play chunk {i+1}: {text[:30]}", e, {"path": audio_path})
                except:
                    pass
                success = False
                # Continue trying to play remaining chunks

        if success:
            logger.info(f"Spoke: {text[:50]}...")
        return success

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
        "This is a longer sentence that should be split into multiple chunks to test the chunking functionality properly.",
    ]

    for text in test_texts:
        print(f"Generating: {text}")
        paths = voice.generate_speech(text)
        if paths:
            print(f"  -> Generated {len(paths)} chunk(s):")
            for p in paths:
                print(f"     {p}")
        else:
            print("  -> Failed!")

    voice.cleanup()


if __name__ == "__main__":
    asyncio.run(test_voice())
