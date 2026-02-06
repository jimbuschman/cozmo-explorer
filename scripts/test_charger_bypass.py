import pycozmo
import time

print("Step 1: Connecting to Cozmo...")

with pycozmo.connect() as cli:
    print("Step 2: Connected!")

    # Check charger status
    on_charger = getattr(cli, 'is_on_charger', 'unknown')
    print(f"Step 3: On charger = {on_charger}")

    # Try to drive off charger
    try:
        cli.drive_off_charger_contacts()
        print("Step 4: drive_off_charger_contacts() worked")
    except AttributeError:
        print("Step 4: drive_off_charger_contacts() not found, trying drive_off_charger()...")
        try:
            cli.drive_off_charger()
            print("Step 4: drive_off_charger() worked")
        except Exception as e:
            print(f"Step 4: That failed too: {e}")
    except Exception as e:
        print(f"Step 4: Error: {e}")

    time.sleep(2)

    print("Step 5: Driving forward...")
    cli.drive_wheels(50, 50)
    time.sleep(2)
    cli.drive_wheels(0, 0)
    print("Step 6: Done!")
