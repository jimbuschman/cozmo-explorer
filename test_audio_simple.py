"""
Simple test of Cozmo audio playback - generates a beep and plays it
"""
import asyncio
import numpy as np
import soundfile as sf
from pathlib import Path
import pycozmo


def create_test_beep():
    """Create a simple beep WAV file"""
    output_dir = Path("data/audio_temp")
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "test_beep.wav"

    # Generate a simple beep (440Hz for 0.5 seconds)
    sample_rate = 22050  # Cozmo requires 22kHz
    duration = 0.5
    frequency = 440  # A4 note

    t = np.linspace(0, duration, int(sample_rate * duration), False)
    beep = np.sin(2 * np.pi * frequency * t)

    # Fade in/out to avoid clicks
    fade_len = int(sample_rate * 0.05)
    beep[:fade_len] *= np.linspace(0, 1, fade_len)
    beep[-fade_len:] *= np.linspace(1, 0, fade_len)

    # Convert to 16-bit integer
    beep_int = (beep * 32767 * 0.5).astype(np.int16)

    # Save as mono WAV
    sf.write(str(output_path), beep_int, sample_rate, subtype='PCM_16')
    print(f"Created test beep: {output_path}")

    return str(output_path)


def test_audio():
    """Test playing audio through Cozmo"""
    # Create test beep
    beep_path = create_test_beep()

    print("\nConnecting to Cozmo...")
    cli = pycozmo.Client()
    cli.start()
    cli.connect()
    cli.wait_for_robot()
    print("Connected!")

    try:
        print(f"\nSetting volume...")
        cli.set_volume(50000)

        print(f"Playing beep: {beep_path}")
        cli.play_audio(beep_path)

        print("Waiting for audio to complete...")
        cli.wait_for(pycozmo.event.EvtAudioCompleted)

        print("\nAudio test complete! Did you hear the beep?")

    except Exception as e:
        print(f"ERROR: {e}")

    finally:
        cli.disconnect()
        cli.stop()


if __name__ == "__main__":
    test_audio()
