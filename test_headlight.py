"""
Test Cozmo's headlight LED

NOTE: The headlight is an INFRARED (IR) LED - invisible to human eyes!
It improves camera performance in dark environments since Cozmo's camera is IR-sensitive.
This test captures images with and without the IR light to show the difference.
"""
import asyncio
import pycozmo
from pathlib import Path
from datetime import datetime


async def test_headlight():
    """Test the IR headlight by comparing camera images"""
    print("Connecting to Cozmo...")
    print("\nNOTE: The headlight is an IR (infrared) LED - invisible to humans!")
    print("You'll see the difference in the captured images.\n")

    cli = pycozmo.Client()
    cli.start()
    cli.connect()
    cli.wait_for_robot()

    print("Connected!")

    # Store captured image
    last_image = None

    def on_image(cli, image):
        nonlocal last_image
        last_image = image

    cli.add_handler(pycozmo.event.EvtNewRawCameraImage, on_image)
    cli.enable_camera(enable=True, color=True)

    # Create output directory
    output_dir = Path("data/ir_test")
    output_dir.mkdir(parents=True, exist_ok=True)

    try:
        # Wait for camera to start
        await asyncio.sleep(1.0)

        # Capture WITHOUT IR light
        print("Capturing image WITHOUT IR light...")
        cli.set_head_light(False)
        await asyncio.sleep(0.5)

        if last_image:
            path_off = output_dir / "ir_OFF.jpg"
            last_image.save(path_off, "JPEG")
            print(f"  Saved: {path_off}")

        # Capture WITH IR light
        print("Capturing image WITH IR light...")
        cli.set_head_light(True)
        await asyncio.sleep(0.5)

        if last_image:
            path_on = output_dir / "ir_ON.jpg"
            last_image.save(path_on, "JPEG")
            print(f"  Saved: {path_on}")

        # Turn off
        cli.set_head_light(False)

        print("\nTest complete!")
        print(f"Compare the images in {output_dir}/ to see the IR effect.")
        print("The 'ir_ON.jpg' should appear brighter, especially in darker areas.")

    finally:
        cli.disconnect()
        cli.stop()


if __name__ == "__main__":
    asyncio.run(test_headlight())
