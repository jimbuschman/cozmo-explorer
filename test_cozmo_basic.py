"""Quick test of basic Cozmo connection and movement"""
import pycozmo
import time

print("Creating client...")
cli = pycozmo.Client()
cli.start()

print("Connecting to Cozmo...")
cli.connect()
cli.wait_for_robot()
print("Connected!")

print("Testing drive forward...")
cli.drive_wheels(lwheel_speed=50.0, rwheel_speed=50.0, duration=1.0)
time.sleep(1.5)

print("Testing turn...")
cli.drive_wheels(lwheel_speed=-30.0, rwheel_speed=30.0, duration=0.5)
time.sleep(1.0)

print("Done! Disconnecting...")
cli.disconnect()
cli.stop()
print("Test complete!")
