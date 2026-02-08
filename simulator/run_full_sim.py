"""
Full-stack simulation test harness.

Runs the mapper system (MapperStateMachine + FrontierNavigator + SpatialMap)
using SimRobot instead of real Cozmo. Accelerated time so hours of robot
mapping happen in minutes.

Single room:
    python -m simulator.run_full_sim --time-scale 5 --duration 300 --world furnished_room

Multi-room:
    python -m simulator.run_full_sim --time-scale 5 --duration 300 --rooms furnished_room,corridor,dead_end

Options:
    --time-scale N    Speed up asyncio.sleep by factor N (default: 10)
    --duration N      Sim-time seconds to run PER ROOM (default: 3600)
    --world NAME      Single world preset (default: multi_room)
    --rooms LIST      Comma-separated list of worlds to run in sequence
    --render          Show pygame window
    --headless        No rendering (default)
"""
import argparse
import asyncio
import logging
import os
import sys
import signal
import time
from datetime import datetime
from pathlib import Path

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from simulator.sim_robot import SimRobot
from simulator.world import PRESETS, multi_room, box_room, corridor, corner, dead_end, furnished_room
from memory.spatial_map import SpatialMap
from memory.experience_logger import ExperienceLogger
from memory.state_store import StateStore
from brain.mapper_state_machine import MapperStateMachine, MapperState

# Configure logging
log_format = '%(asctime)s | %(levelname)-8s | %(name)-20s | %(message)s'
logging.basicConfig(
    level=logging.INFO,
    format=log_format,
    datefmt='%H:%M:%S',
    handlers=[logging.StreamHandler()]
)
logger = logging.getLogger(__name__)


# ============================================================================
# World name -> constructor mapping
# ============================================================================
WORLD_MAP = {
    'box': box_room,
    'corridor': corridor,
    'corner': corner,
    'dead_end': dead_end,
    'multi_room': multi_room,
    'furnished_room': furnished_room,
}


# ============================================================================
# Time acceleration
# ============================================================================
_original_sleep = asyncio.sleep


def make_accelerated_sleep(time_scale: float):
    """Create a patched asyncio.sleep that divides delay by time_scale."""
    async def accelerated_sleep(delay, result=None):
        await _original_sleep(delay / time_scale, result)
    return accelerated_sleep


# ============================================================================
# Report generation
# ============================================================================
def generate_room_report(
    *,
    room_id: str,
    world_name: str,
    duration_sim: float,
    wall_clock_seconds: float,
    spatial_map: SpatialMap,
    mapper: MapperStateMachine,
    robot,
) -> str:
    """Generate a report section for a single room."""
    lines = []
    lines.append(f"== ROOM: {room_id} ({world_name}) ==")
    lines.append(f"  Sim duration:    {duration_sim:.0f}s ({duration_sim/60:.1f} min)")
    lines.append(f"  Wall clock:      {wall_clock_seconds:.1f}s ({wall_clock_seconds/60:.1f} min)")
    lines.append("")

    # Map exploration
    visited_pct = spatial_map.get_visited_percentage() * 100
    explored_pct = spatial_map.get_exploration_progress() * 100
    summary = spatial_map.get_summary()
    lines.append(f"  Map visited:     {visited_pct:.1f}%")
    lines.append(f"  Map known:       {explored_pct:.1f}%")
    lines.append(f"  Cell states:     {summary}")
    lines.append("")

    # Mapper stats
    status = mapper.get_status()
    lines.append(f"  Escapes:         {status['escapes']}")
    lines.append(f"  Frontier targets:{status['frontier_targets']}")
    lines.append(f"  Final state:     {status['state']}")
    lines.append("")

    # ASCII map
    lines.append(f"  MAP (ASCII):")
    ascii_map = spatial_map.to_ascii(robot_x=robot.pose.x, robot_y=robot.pose.y)
    for map_line in ascii_map.split('\n'):
        lines.append(f"  {map_line}")
    lines.append("")

    return "\n".join(lines)


def generate_final_report(
    *,
    room_reports: list,
    time_scale: float,
    total_sim_seconds: float,
    total_wall_seconds: float,
) -> str:
    """Generate the full report with all rooms and overall summary."""
    lines = []
    lines.append("=" * 70)
    lines.append("  MAPPER SIMULATION REPORT")
    lines.append("=" * 70)
    lines.append("")

    # Run parameters
    lines.append("== RUN PARAMETERS ==")
    lines.append(f"  Time scale:      {time_scale}x")
    lines.append(f"  Total sim time:  {total_sim_seconds:.0f}s ({total_sim_seconds/60:.1f} min)")
    lines.append(f"  Total wall time: {total_wall_seconds:.1f}s ({total_wall_seconds/60:.1f} min)")
    lines.append(f"  Rooms:           {len(room_reports)}")
    lines.append(f"  Generated:       {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append("")

    # Per-room reports
    for room_report in room_reports:
        lines.append(room_report)

    lines.append("=" * 70)
    lines.append("  END OF REPORT")
    lines.append("=" * 70)

    return "\n".join(lines)


# ============================================================================
# Single room runner
# ============================================================================
async def run_room(
    *,
    room_index: int,
    room_id: str,
    world_name: str,
    args,
    experience_logger: ExperienceLogger,
    state_store: StateStore,
    stop_event: asyncio.Event,
) -> dict:
    """Run a single room simulation."""
    logger.info("=" * 60)
    logger.info(f"ROOM {room_index + 1}: {room_id} (world: {world_name})")
    logger.info("=" * 60)

    room_start = time.monotonic()

    # Build world
    world_fn = WORLD_MAP.get(world_name)
    if not world_fn:
        logger.error(f"Unknown world: {world_name}. Options: {list(WORLD_MAP.keys())}")
        return {'room_report': f"== ROOM: {room_id} - SKIPPED (unknown world) ==\n",
                'elapsed_sim': 0, 'wall_seconds': 0}
    world = world_fn()

    # Create SimRobot
    sim_robot = SimRobot(world)
    await sim_robot.start()

    # Load existing map or start fresh
    map_path = config.DATA_DIR / f"sim_map_{room_id}.npz"
    if map_path.exists():
        spatial_map = SpatialMap.load(str(map_path))
        logger.info(f"Loaded existing map for {room_id}: {spatial_map.get_visited_percentage()*100:.1f}% visited")
    else:
        spatial_map = SpatialMap()

    # Set room_id on experience logger
    experience_logger.set_room_id(room_id)

    # Start session
    session_id = state_store.start_session()
    experience_logger.set_session_id(session_id)
    logger.info(f"Session {session_id} started for room {room_id}")

    # Create mapper state machine
    mapper = MapperStateMachine(
        robot=sim_robot,
        spatial_map=spatial_map,
        experience_logger=experience_logger,
    )

    # Renderer (optional)
    renderer = None
    if args.render:
        try:
            from simulator.renderer import Renderer
            renderer = Renderer(spatial_map=spatial_map)
            logger.info("Renderer enabled")
        except Exception as e:
            logger.warning(f"Could not start renderer: {e}")

    # Run
    sim_duration = args.duration
    real_duration = sim_duration / args.time_scale
    logger.info(f"Running room {room_id} for {sim_duration:.0f} sim-seconds ({real_duration:.0f}s wall clock)")

    sm_task = asyncio.create_task(mapper.start())

    elapsed_sim = 0.0
    try:
        last_status = time.monotonic()
        status_interval = 30.0

        while elapsed_sim < sim_duration and not stop_event.is_set():
            if renderer:
                actions = renderer.handle_events()
                if actions.get('quit'):
                    stop_event.set()
                    break
                front = sim_robot.sensors.get_front_obstacle_distance()
                visited_pct = spatial_map.get_visited_percentage() * 100
                status_text = (
                    f"ROOM {room_index+1} {room_id} {elapsed_sim:.0f}/{sim_duration:.0f}s | "
                    f"{mapper.state.name} | front={front}mm | map={visited_pct:.1f}%"
                )
                renderer.draw(sim_robot, status_text)
                renderer.tick(60)

            await _original_sleep(1 / 60)

            wall_now = time.monotonic()
            elapsed_sim = (wall_now - room_start) * args.time_scale

            # Check if mapper finished (DONE state)
            if mapper.state == MapperState.DONE:
                logger.info(f"Mapper reached DONE state at {elapsed_sim:.0f}s sim")
                break

            if wall_now - last_status >= status_interval:
                last_status = wall_now
                status = mapper.get_status()
                logger.info(
                    f"[Room {room_id} {elapsed_sim:.0f}s sim] "
                    f"State={status['state']} "
                    f"Map={status['visited_pct']:.1f}%/{status['explored_pct']:.1f}% "
                    f"Escapes={status['escapes']} "
                    f"Targets={status['frontier_targets']}"
                )

    finally:
        # Stop mapper
        await mapper.stop()
        try:
            await asyncio.wait_for(sm_task, timeout=5.0)
        except (asyncio.TimeoutError, asyncio.CancelledError):
            sm_task.cancel()

        await sim_robot.shutdown()

        if renderer:
            renderer.quit()

    wall_seconds = time.monotonic() - room_start

    # Save map
    map_path = config.DATA_DIR / f"sim_map_{room_id}.npz"
    spatial_map.save(str(map_path))

    # End session
    state_store.end_session(
        duration=wall_seconds,
        distance=0,
        areas=int(spatial_map.get_visited_percentage() * 100),
        summary=f"Room {room_id}: {world_name}, {args.time_scale}x, {elapsed_sim:.0f}s sim"
    )

    # Generate per-room report
    room_report = generate_room_report(
        room_id=room_id,
        world_name=world.name,
        duration_sim=elapsed_sim,
        wall_clock_seconds=wall_seconds,
        spatial_map=spatial_map,
        mapper=mapper,
        robot=sim_robot,
    )

    logger.info(f"Room {room_id} complete: {elapsed_sim:.0f}s sim, {wall_seconds:.1f}s wall, "
                f"{spatial_map.get_visited_percentage()*100:.1f}% visited")

    return {
        'room_report': room_report,
        'elapsed_sim': elapsed_sim,
        'wall_seconds': wall_seconds,
    }


# ============================================================================
# Main simulation
# ============================================================================
async def run_simulation(args):
    """Run the full-stack simulation (single or multi-room)."""
    start_wall = time.monotonic()

    # --- Patch time ---
    if args.time_scale != 1.0:
        asyncio.sleep = make_accelerated_sleep(args.time_scale)
        logger.info(f"Time acceleration: {args.time_scale}x")

    # --- Determine room list ---
    if args.rooms:
        room_list = [r.strip() for r in args.rooms.split(',')]
        for room_name in room_list:
            if room_name not in WORLD_MAP:
                logger.error(f"Unknown room: {room_name}. Options: {list(WORLD_MAP.keys())}")
                asyncio.sleep = _original_sleep
                return
    else:
        room_list = [args.world]

    logger.info(f"Room sequence: {room_list}")
    logger.info(f"Duration per room: {args.duration:.0f}s sim ({args.duration/args.time_scale:.0f}s wall)")

    # --- Create sim database ---
    sim_db_path = config.DATA_DIR / "sim_state.db"
    logger.info(f"Sim database: {sim_db_path}")

    state_store = StateStore(db_path=str(sim_db_path))
    state_store.connect()

    experience_logger = ExperienceLogger(db_path=str(sim_db_path))
    experience_logger.connect()

    # --- Signal handler ---
    stop_event = asyncio.Event()

    def signal_handler(sig, frame):
        logger.info("Interrupt received, stopping...")
        stop_event.set()

    signal.signal(signal.SIGINT, signal_handler)

    # --- Run rooms ---
    room_reports = []
    total_sim = 0.0
    total_wall = 0.0

    try:
        for i, world_name in enumerate(room_list):
            if stop_event.is_set():
                break

            room_id = f"room_{i+1}_{world_name}"

            result = await run_room(
                room_index=i,
                room_id=room_id,
                world_name=world_name,
                args=args,
                experience_logger=experience_logger,
                state_store=state_store,
                stop_event=stop_event,
            )

            room_reports.append(result['room_report'])
            total_sim += result['elapsed_sim']
            total_wall += result['wall_seconds']

    finally:
        # Restore original sleep
        asyncio.sleep = _original_sleep

        wall_elapsed = time.monotonic() - start_wall

        # --- Generate final report ---
        logger.info("Generating final report...")

        report = generate_final_report(
            room_reports=room_reports,
            time_scale=args.time_scale,
            total_sim_seconds=total_sim,
            total_wall_seconds=wall_elapsed,
        )

        # Save report
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_path = config.DATA_DIR / f"sim_report_{timestamp}.txt"
        report_path.write_text(report, encoding='utf-8')
        logger.info(f"Report saved: {report_path}")

        # Print report
        print("\n" + report)

        # Cleanup
        experience_logger.close()
        state_store.close()

        logger.info("Simulation complete.")


def main():
    parser = argparse.ArgumentParser(description="Mapper simulation test harness")
    parser.add_argument("--time-scale", type=float, default=10.0,
                        help="Speed up asyncio.sleep by this factor (default: 10)")
    parser.add_argument("--duration", type=float, default=3600.0,
                        help="Sim-time seconds to run PER ROOM (default: 3600)")
    parser.add_argument("--world", type=str, default="multi_room",
                        choices=list(WORLD_MAP.keys()),
                        help="Single world preset (default: multi_room)")
    parser.add_argument("--rooms", type=str, default=None,
                        help="Comma-separated list of worlds (e.g. furnished_room,corridor,dead_end)")
    parser.add_argument("--render", action="store_true", default=False,
                        help="Show pygame renderer")
    parser.add_argument("--headless", action="store_true", default=True,
                        help="No rendering (default)")
    args = parser.parse_args()

    asyncio.run(run_simulation(args))


if __name__ == "__main__":
    main()
