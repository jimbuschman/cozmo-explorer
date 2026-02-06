import pycozmo
import time

print("Connecting to Cozmo...")

with pycozmo.connect() as cli:
    print("Connected!")

    # Show all available methods (so we can find the right one)
    methods = [m for m in dir(cli) if not m.startswith('_')]
    print("\nAll client methods:")
    for m in methods:
        print(f"  {m}")

    print("\nCharger/drive related:")
    for m in methods:
        if any(k in m.lower() for k in ['charger', 'drive', 'dock', 'wheel', 'motor']):
            print(f"  {m}")
