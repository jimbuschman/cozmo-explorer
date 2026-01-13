"""
Test Cozmo's headlight LED
"""
import asyncio
import pycozmo


async def test_headlight():
    """Test turning the headlight on and off"""
    print("Connecting to Cozmo...")

    cli = pycozmo.Client()
    cli.start()
    cli.connect()
    cli.wait_for_robot()

    print("Connected!")

    try:
        print("Turning headlight ON...")
        cli.set_head_light(True)
        await asyncio.sleep(3.0)

        print("Turning headlight OFF...")
        cli.set_head_light(False)
        await asyncio.sleep(1.0)

        print("Blinking headlight 3 times...")
        for i in range(3):
            cli.set_head_light(True)
            await asyncio.sleep(0.5)
            cli.set_head_light(False)
            await asyncio.sleep(0.5)

        print("Test complete!")

    finally:
        cli.disconnect()
        cli.stop()


if __name__ == "__main__":
    asyncio.run(test_headlight())
