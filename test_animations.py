"""Test Cozmo animations and sounds"""
import pycozmo

def main():
    with pycozmo.connect() as cli:
        print("Loading animations...")
        cli.load_anims()

        names = cli.get_anim_names()
        print(f"\nFound {len(names)} animations:\n")

        # Print sorted list
        for name in sorted(names):
            print(f"  {name}")

        # Try playing a few explorer-related ones
        print("\n\nPlaying some test animations...")

        test_anims = [
            "anim_explorer_idle_01",
            "anim_sparking_success_01",
            "anim_pounce_success_01",
            "anim_reacttoface_unidentified_01",
        ]

        import time
        for anim in test_anims:
            if anim in names:
                print(f"Playing: {anim}")
                cli.play_anim(anim)
                time.sleep(3)
            else:
                print(f"Not found: {anim}")

if __name__ == "__main__":
    main()
