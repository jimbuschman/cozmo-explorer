"""
Simulator entry point.

Run: python -m simulator.run_sim

Controls:
    A           Start/stop autonomous frontier navigation
    Space       Pause/resume physics
    R           Reset to spawn
    1-4         Switch world preset
    Arrow keys  Manual drive
    Esc         Quit
"""
import asyncio
import logging
import sys
import os

# Add project root to path so imports work
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from simulator.sim_robot import SimRobot
from simulator.world import PRESETS, box_room
from simulator.renderer import Renderer
from memory.spatial_map import SpatialMap

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(name)s %(levelname)s: %(message)s',
    datefmt='%H:%M:%S',
)
logger = logging.getLogger(__name__)


async def run_frontier_nav(sim_robot, spatial_map):
    """Run FrontierNavigator autonomously - drives toward unexplored areas."""
    from brain.frontier_navigator import FrontierNavigator

    nav = FrontierNavigator(
        robot=sim_robot,
        spatial_map=spatial_map,
        duration=600.0,  # 10 minutes
        speed=config.WANDER_SPEED,
    )
    logger.info("Frontier navigation started")
    result = await nav.run()
    logger.info(f"Navigation finished: {result.name} - {nav.escapes} escapes, {nav.frontier_targets} targets")
    return result


async def main():
    world = box_room()
    sim_robot = SimRobot(world)
    spatial_map = SpatialMap()
    renderer = Renderer(spatial_map=spatial_map)

    await sim_robot.start()

    behavior_task = None  # Wander or zigzag
    maneuver_status = ""
    manual_driving = False
    auto_mode = False

    try:
        while True:
            actions = renderer.handle_events()

            if actions.get('quit'):
                break

            # Cancel running behavior on reset or world change
            def cancel_behavior():
                nonlocal behavior_task, auto_mode, maneuver_status
                if behavior_task and not behavior_task.done():
                    behavior_task.cancel()
                    behavior_task = None
                auto_mode = False
                maneuver_status = ""

            if actions.get('reset'):
                cancel_behavior()
                sim_robot.reset()
                spatial_map = SpatialMap()
                renderer.spatial_map = spatial_map

            if 'world_preset' in actions:
                preset_num = actions['world_preset']
                if preset_num in PRESETS:
                    cancel_behavior()
                    world = PRESETS[preset_num]()
                    sim_robot.reset(world)
                    spatial_map = SpatialMap()
                    renderer.spatial_map = spatial_map
                    maneuver_status = f"Loaded: {world.name}"

            # Toggle autonomous frontier navigation
            if actions.get('auto_wander'):
                if auto_mode:
                    cancel_behavior()
                    maneuver_status = "Auto stopped"
                    logger.info("Auto navigation stopped by user")
                else:
                    cancel_behavior()
                    auto_mode = True
                    maneuver_status = "AUTO NAV"
                    behavior_task = asyncio.create_task(run_frontier_nav(sim_robot, spatial_map))

            # Manual drive controls (only when not in auto mode)
            if not auto_mode:
                if actions.get('drive_forward'):
                    manual_driving = True
                    await sim_robot.set_wheels(config.WANDER_SPEED, config.WANDER_SPEED)
                elif actions.get('drive_backward'):
                    manual_driving = True
                    await sim_robot.set_wheels(-config.WANDER_SPEED, -config.WANDER_SPEED)
                elif actions.get('turn_left'):
                    manual_driving = True
                    await sim_robot.set_wheels(
                        config.WANDER_SPEED * 0.3, config.WANDER_SPEED)
                elif actions.get('turn_right'):
                    manual_driving = True
                    await sim_robot.set_wheels(
                        config.WANDER_SPEED, config.WANDER_SPEED * 0.3)
                elif actions.get('stop_drive') and manual_driving:
                    manual_driving = False
                    if behavior_task is None or behavior_task.done():
                        await sim_robot.stop()

            # Check behavior task status
            if behavior_task and behavior_task.done():
                try:
                    result = behavior_task.result()
                    maneuver_status = f"Nav: {result.name}" if hasattr(result, 'name') else str(result)
                except asyncio.CancelledError:
                    maneuver_status = "Cancelled"
                except Exception as e:
                    maneuver_status = f"Error: {e}"
                    logger.exception("Behavior task error")
                behavior_task = None
                auto_mode = False

            # Show auto mode in status
            if auto_mode and behavior_task and not behavior_task.done():
                front = sim_robot.sensors.get_front_obstacle_distance()
                visited_pct = spatial_map.get_visited_percentage() * 100
                maneuver_status = (
                    f"AUTO NAV | front={front}mm "
                    f"L={sim_robot.sensors.ext_ultra_l_mm} "
                    f"R={sim_robot.sensors.ext_ultra_r_mm} "
                    f"map={visited_pct:.1f}%"
                )

            # Pause physics
            if renderer.paused:
                sim_robot.state.left_speed = 0
                sim_robot.state.right_speed = 0

            renderer.draw(sim_robot, maneuver_status)
            renderer.tick(60)
            await asyncio.sleep(1 / 60)

    finally:
        if behavior_task and not behavior_task.done():
            behavior_task.cancel()
            try:
                await behavior_task
            except (asyncio.CancelledError, Exception):
                pass
        await sim_robot.shutdown()
        renderer.quit()


if __name__ == "__main__":
    asyncio.run(main())
