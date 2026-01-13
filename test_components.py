"""
Test Components

Quick tests for individual components without needing the robot.
Run these to verify your setup is working.
"""
import asyncio
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


async def test_llm():
    """Test LLM connection and basic query"""
    from llm.client import LLMClient

    print("\n" + "="*50)
    print("Testing LLM Connection")
    print("="*50)

    async with LLMClient() as llm:
        # Health check
        healthy = await llm.health_check()
        print(f"LLM healthy: {healthy}")

        if not healthy:
            print("Make sure Ollama is running: ollama serve")
            print(f"And the model is pulled: ollama pull {llm.model}")
            return False

        # Test query
        response = await llm.query_for_goal({
            "state": "IDLE",
            "position": {"x": 0, "y": 0, "angle_degrees": 0},
            "sensors": {"battery": 4.0, "on_charger": False, "cliff_detected": False},
            "context": {
                "exploration_time": 0,
                "stuck_count": 0,
                "current_goal": None,
                "interesting_observations": []
            }
        })

        print(f"\nLLM Response:\n{response}")
        return True


async def test_memory():
    """Test experience database"""
    from memory.experience_db import ExperienceDB, Experience
    from datetime import datetime
    import uuid

    print("\n" + "="*50)
    print("Testing Experience Database")
    print("="*50)

    async with ExperienceDB() as db:
        # Add some test experiences
        experiences = [
            Experience(
                id=str(uuid.uuid4()),
                description="Found a red chair in the corner",
                location_x=100,
                location_y=200,
                timestamp=datetime.now(),
                experience_type="object",
                importance=0.7
            ),
            Experience(
                id=str(uuid.uuid4()),
                description="Encountered a wall obstacle",
                location_x=300,
                location_y=100,
                timestamp=datetime.now(),
                experience_type="obstacle",
                importance=0.5
            ),
            Experience(
                id=str(uuid.uuid4()),
                description="Open floor area, easy to traverse",
                location_x=0,
                location_y=0,
                timestamp=datetime.now(),
                experience_type="area",
                importance=0.3
            ),
        ]

        for exp in experiences:
            await db.add_experience(exp)
            print(f"Added: {exp.description[:40]}...")

        # Test similarity search
        print("\nSearching for 'furniture'...")
        similar = await db.find_similar("furniture like a chair or table")
        for exp in similar:
            print(f"  Found: {exp.description}")

        # Test location search
        print("\nSearching near (150, 150)...")
        nearby = await db.find_near_location(150, 150, radius=200)
        for exp in nearby:
            print(f"  Found: {exp.description} at ({exp.location_x}, {exp.location_y})")

        count = await db.count()
        print(f"\nTotal experiences: {count}")

    return True


async def test_spatial_map():
    """Test spatial map"""
    from memory.spatial_map import SpatialMap

    print("\n" + "="*50)
    print("Testing Spatial Map")
    print("="*50)

    # Create a map
    spatial_map = SpatialMap(
        size_mm=(2000, 2000),
        resolution_mm=100,
        origin_mm=(1000, 1000)
    )

    # Simulate some exploration
    path = [
        (0, 0), (100, 0), (200, 0), (300, 0),
        (300, 100), (300, 200), (300, 300),
        (200, 300), (100, 300), (0, 300),
        (0, 200), (0, 100)
    ]

    for x, y in path:
        spatial_map.mark_visited(x, y)

    # Mark some obstacles
    spatial_map.mark_occupied(400, 100)
    spatial_map.mark_occupied(400, 200)

    # Add point of interest
    spatial_map.add_point(200, 200, "Interesting spot", "landmark")

    # Print summary
    summary = spatial_map.get_summary()
    print(f"Map size: {summary['size']}")
    print(f"Resolution: {summary['resolution']}")
    print(f"Exploration progress: {summary['exploration_progress']}")
    print(f"Visited: {summary['visited_percentage']}")

    # ASCII visualization
    print("\nMap visualization:")
    print(spatial_map.to_ascii(robot_x=0, robot_y=100))

    # Find nearest unknown
    unknown = spatial_map.find_nearest_unknown(0, 0)
    print(f"\nNearest unexplored: {unknown}")

    return True


async def test_state_store():
    """Test state persistence"""
    from memory.state_store import StateStore
    import config

    print("\n" + "="*50)
    print("Testing State Store")
    print("="*50)

    # Use a test database
    test_db = config.DATA_DIR / "test_state.db"

    with StateStore(str(test_db)) as store:
        # Test key-value
        store.set("test_key", {"hello": "world", "number": 42})
        value = store.get("test_key")
        print(f"Stored and retrieved: {value}")

        # Test session
        session_id = store.start_session()
        print(f"Started session: {session_id}")

        # Log some events
        store.log_event("test", "This is a test event", {"data": 123})
        store.log_event("test", "Another event")

        # End session
        store.end_session(duration=10.5, distance=500, areas=3, summary="Test run")

        # Get history
        history = store.get_session_history()
        print(f"Session history: {len(history)} sessions")

        events = store.get_events(event_type="test")
        print(f"Test events: {len(events)}")

    # Cleanup
    test_db.unlink(missing_ok=True)
    return True


async def test_robot_simulation():
    """Test robot interface in simulation mode"""
    from cozmo_interface.robot import CozmoRobot

    print("\n" + "="*50)
    print("Testing Robot (Simulation Mode)")
    print("="*50)

    async with CozmoRobot() as robot:
        print(f"Connected: {robot.is_connected}")
        print(f"State: {robot.state}")

        # Test basic commands (no-op in simulation)
        await robot.drive(50, duration=1.0)
        await robot.turn(90)
        await robot.stop()

        print("Basic commands executed (simulated)")

    return True


async def run_all_tests():
    """Run all component tests"""
    print("\n" + "#"*60)
    print("#  COZMO EXPLORER - COMPONENT TESTS")
    print("#"*60)

    results = {}

    # Run tests
    try:
        results['state_store'] = await test_state_store()
    except Exception as e:
        print(f"State store test failed: {e}")
        results['state_store'] = False

    try:
        results['spatial_map'] = await test_spatial_map()
    except Exception as e:
        print(f"Spatial map test failed: {e}")
        results['spatial_map'] = False

    try:
        results['memory'] = await test_memory()
    except Exception as e:
        print(f"Memory test failed: {e}")
        results['memory'] = False

    try:
        results['llm'] = await test_llm()
    except Exception as e:
        print(f"LLM test failed: {e}")
        results['llm'] = False

    try:
        results['robot_sim'] = await test_robot_simulation()
    except Exception as e:
        print(f"Robot simulation test failed: {e}")
        results['robot_sim'] = False

    # Summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)

    for test, passed in results.items():
        status = "PASS" if passed else "FAIL"
        print(f"  {test:20s} {status}")

    all_passed = all(results.values())
    print("\n" + ("All tests passed!" if all_passed else "Some tests failed."))

    return all_passed


if __name__ == "__main__":
    asyncio.run(run_all_tests())
