"""
Test Cozmo voice generation and playback
"""
import asyncio
import sys

# Test voice generation locally first (without Cozmo)
def test_generation():
    """Test just the audio generation part"""
    print("Testing voice generation (no Cozmo needed)...")

    from audio.voice import CozmoVoice, HAS_AUDIO_DEPS

    if not HAS_AUDIO_DEPS:
        print("ERROR: Missing dependencies. Install with:")
        print("  pip install pyttsx3 librosa soundfile")
        return False

    voice = CozmoVoice()

    test_texts = [
        "I see a wooden table.",
        "There is a chair in front of me.",
        "Exploring the room now.",
    ]

    for text in test_texts:
        print(f"\nGenerating: '{text}'")
        paths = voice.generate_speech(text)
        if paths:
            print(f"  Success! Generated {len(paths)} chunk(s):")
            for p in paths:
                print(f"    {p}")
        else:
            print("  Failed to generate!")
            return False

    print("\nVoice generation test passed!")
    return True


async def test_with_cozmo():
    """Test voice playback through Cozmo"""
    print("\nTesting voice playback through Cozmo...")

    import pycozmo
    from audio.voice import CozmoVoice

    # Simple robot wrapper for the voice module
    class SimpleRobot:
        def __init__(self, client):
            self.client = client

        async def play_audio(self, wav_path: str, wait: bool = True):
            self.client.play_audio(wav_path)
            if wait:
                self.client.wait_for(pycozmo.event.EvtAudioCompleted)

    print("Connecting to Cozmo...")
    cli = pycozmo.Client()
    cli.start()
    cli.connect()
    cli.wait_for_robot()
    print("Connected!")

    try:
        robot = SimpleRobot(cli)
        voice = CozmoVoice(robot)

        test_phrases = [
            "Hello! I am Cozmo.",
            "I can see you.",
        ]

        for phrase in test_phrases:
            print(f"\nSaying: '{phrase}'")
            success = await voice.speak(phrase)
            if success:
                print("  Played successfully!")
                await asyncio.sleep(3.0)  # Wait for audio to finish
            else:
                print("  Failed to play!")

        print("\nCozmo voice test complete!")

    finally:
        cli.disconnect()
        cli.stop()


if __name__ == "__main__":
    # First test generation
    if not test_generation():
        sys.exit(1)

    # Ask if user wants to test with Cozmo
    print("\n" + "="*50)
    response = input("Test playback through Cozmo? (y/n): ").strip().lower()

    if response == 'y':
        asyncio.run(test_with_cozmo())
    else:
        print("Skipping Cozmo playback test.")
