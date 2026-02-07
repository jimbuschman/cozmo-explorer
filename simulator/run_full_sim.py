"""
Full-stack simulation test harness.

Runs the FULL system stack (StateMachine + LearningCoordinator + ExperienceLogger +
RulesStore + LLM + SpatialMap) using SimRobot instead of real Cozmo. Accelerated
time so hours of robot experience happen in minutes.

Run: python -m simulator.run_full_sim --time-scale 10 --duration 3600 --no-llm

Options:
    --time-scale N    Speed up asyncio.sleep by factor N (default: 10)
    --duration N      Sim-time seconds to run (default: 3600)
    --world NAME      World preset: box, corridor, corner, dead_end, multi_room (default: multi_room)
    --render          Show pygame window
    --headless        No rendering (default)
    --llm-host URL    Ollama host URL
    --llm-model NAME  Ollama model name
    --no-llm          Disable LLM entirely (statistical proposals only)
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
from simulator.world import PRESETS, multi_room, box_room, corridor, corner, dead_end
from memory.spatial_map import SpatialMap
from memory.experience_logger import ExperienceLogger
from memory.pattern_analyzer import PatternAnalyzer
from memory.learned_rules import LearnedRulesStore
from memory.state_store import StateStore
from brain.learning_coordinator import LearningCoordinator
from brain.state_machine import StateMachine, Goal

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
def generate_report(
    *,
    world_name: str,
    time_scale: float,
    duration_sim: float,
    wall_clock_seconds: float,
    spatial_map: SpatialMap,
    experience_logger: ExperienceLogger,
    rules_store: LearnedRulesStore,
    robot,
) -> str:
    """Generate a text report of the simulation run."""
    lines = []
    lines.append("=" * 70)
    lines.append("  FULL-STACK SIMULATION REPORT")
    lines.append("=" * 70)
    lines.append("")

    # Run parameters
    lines.append("== RUN PARAMETERS ==")
    lines.append(f"  World:           {world_name}")
    lines.append(f"  Time scale:      {time_scale}x")
    lines.append(f"  Sim duration:    {duration_sim:.0f}s ({duration_sim/60:.1f} min)")
    lines.append(f"  Wall clock:      {wall_clock_seconds:.1f}s ({wall_clock_seconds/60:.1f} min)")
    lines.append(f"  Generated:       {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append("")

    # Map exploration
    lines.append("== MAP EXPLORATION ==")
    visited_pct = spatial_map.get_visited_percentage() * 100
    summary = spatial_map.get_summary()
    lines.append(f"  Visited:         {visited_pct:.1f}%")
    lines.append(f"  Cell states:     {summary}")
    lines.append("")

    # Experience data
    lines.append("== EXPERIENCE DATA ==")
    stall_stats = experience_logger.get_recovery_statistics("escape_stall")
    cliff_stats = experience_logger.get_recovery_statistics("escape_cliff")
    lines.append(f"  Stall escapes:   {stall_stats.get('total', 0)} total, "
                 f"{stall_stats.get('success_rate', 0)*100:.0f}% success")
    lines.append(f"  Cliff escapes:   {cliff_stats.get('total', 0)} total, "
                 f"{cliff_stats.get('success_rate', 0)*100:.0f}% success")

    # Angle breakdown
    by_angle = stall_stats.get('by_angle', {})
    if by_angle:
        lines.append("  Stall escape by angle:")
        for angle, stats in sorted(by_angle.items(), key=lambda x: str(x[0])):
            rate = stats['success'] / stats['total'] * 100 if stats['total'] > 0 else 0
            lines.append(f"    {angle:>6}: {stats['total']:>3} attempts, {rate:.0f}% success")
    lines.append("")

    # Rules lifecycle
    lines.append("== RULES LIFECYCLE ==")
    all_rules = rules_store.get_all_rules()
    if all_rules:
        # Summary by status
        by_status = {}
        for rule in all_rules:
            by_status.setdefault(rule.status, []).append(rule)
        for status, rules in sorted(by_status.items()):
            lines.append(f"  {status}: {len(rules)}")

        lines.append("")
        lines.append("  All rules:")
        for rule in all_rules:
            rate = f"{rule.success_rate*100:.0f}%" if rule.times_applied > 0 else "n/a"
            lines.append(f"    [{rule.status:>10}] {rule.name}")
            lines.append(f"               applied={rule.times_applied}, "
                         f"success={rule.times_successful}, rate={rate}")
            if rule.description:
                lines.append(f"               {rule.description[:80]}")
    else:
        lines.append("  No rules proposed.")
    lines.append("")

    # ASCII map
    lines.append("== MAP (ASCII) ==")
    ascii_map = spatial_map.to_ascii(robot_x=robot.pose.x, robot_y=robot.pose.y)
    lines.append(ascii_map)
    lines.append("")

    lines.append("=" * 70)
    lines.append("  END OF REPORT")
    lines.append("=" * 70)

    return "\n".join(lines)


# ============================================================================
# Main simulation
# ============================================================================
async def run_simulation(args):
    """Run the full-stack simulation."""
    start_wall = time.monotonic()

    # --- Patch time ---
    if args.time_scale != 1.0:
        asyncio.sleep = make_accelerated_sleep(args.time_scale)
        logger.info(f"Time acceleration: {args.time_scale}x")

    # --- Build world ---
    world_fn = WORLD_MAP.get(args.world)
    if not world_fn:
        logger.error(f"Unknown world: {args.world}. Options: {list(WORLD_MAP.keys())}")
        return
    world = world_fn()
    logger.info(f"World: {world.name}")

    # --- Create SimRobot ---
    sim_robot = SimRobot(world)
    await sim_robot.start()
    logger.info("SimRobot started")

    # --- Create spatial map ---
    spatial_map = SpatialMap()

    # --- Create separate sim database ---
    sim_db_path = config.DATA_DIR / "sim_state.db"
    logger.info(f"Sim database: {sim_db_path}")

    # State store
    state_store = StateStore(db_path=str(sim_db_path))
    state_store.connect()

    # Experience logger
    experience_logger = ExperienceLogger(db_path=str(sim_db_path))
    experience_logger.connect()

    # Rules store
    rules_store = LearnedRulesStore(db_path=str(sim_db_path))
    rules_store.connect()

    # Pattern analyzer
    pattern_analyzer = PatternAnalyzer(experience_logger)

    # --- LLM client (optional) ---
    llm_client = None
    if not args.no_llm:
        try:
            from llm.client import LLMClient
            llm_host = args.llm_host or config.OLLAMA_HOST
            llm_model = args.llm_model or config.OLLAMA_MODEL
            llm_client = LLMClient(host=llm_host, model=llm_model)
            await llm_client.connect()
            if await llm_client.health_check():
                logger.info(f"LLM ready: {llm_model} at {llm_host}")
            else:
                logger.warning("LLM not available - falling back to statistical proposals")
                await llm_client.close()
                llm_client = None
        except Exception as e:
            logger.warning(f"LLM init failed: {e} - statistical proposals only")
            llm_client = None
    else:
        logger.info("LLM disabled (--no-llm)")

    # --- Learning coordinator ---
    learning_coordinator = LearningCoordinator(
        llm_client=llm_client,
        experience_logger=experience_logger,
        pattern_analyzer=pattern_analyzer,
        rules_store=rules_store,
    )

    # Adjust learning thresholds for sim speed
    learning_coordinator.MIN_SAMPLES_FOR_ANALYSIS = 10
    learning_coordinator.ANALYSIS_COOLDOWN_MINUTES = max(1, learning_coordinator.ANALYSIS_COOLDOWN_MINUTES / args.time_scale)

    # --- State machine ---
    state_machine = StateMachine(
        robot=sim_robot,
        llm_client=llm_client,
        memory=None,  # No ChromaDB in sim
        spatial_map=spatial_map,
        vision_observer=None,  # No camera in sim
        learning_coordinator=learning_coordinator,
        experience_logger=experience_logger,
        rules_store=rules_store,
    )

    # Adjust learning interval for time scale
    state_machine.LEARNING_INTERVAL_SECONDS = max(
        30, state_machine.LEARNING_INTERVAL_SECONDS / args.time_scale
    )
    logger.info(f"Learning interval: {state_machine.LEARNING_INTERVAL_SECONDS:.0f}s "
                f"(real: {state_machine.LEARNING_INTERVAL_SECONDS / args.time_scale:.0f}s)")

    # --- Session ---
    session_id = state_store.start_session()
    experience_logger.set_session_id(session_id)
    logger.info(f"Session {session_id} started")

    # Set initial goal
    state_machine.set_goal(Goal(
        description="Explore and learn survival rules",
        goal_type="explore",
    ))

    # --- Renderer (optional) ---
    renderer = None
    if args.render:
        try:
            from simulator.renderer import Renderer
            renderer = Renderer(spatial_map=spatial_map)
            logger.info("Renderer enabled")
        except Exception as e:
            logger.warning(f"Could not start renderer: {e}")

    # --- Run ---
    sim_duration = args.duration  # in sim-seconds
    real_duration = sim_duration / args.time_scale
    logger.info(f"Running for {sim_duration:.0f} sim-seconds ({real_duration:.0f}s wall clock)")
    logger.info("=" * 60)

    stop_event = asyncio.Event()

    def signal_handler(sig, frame):
        logger.info("Interrupt received, stopping...")
        stop_event.set()

    signal.signal(signal.SIGINT, signal_handler)

    # Start state machine in background
    sm_task = asyncio.create_task(state_machine.start())

    try:
        elapsed_sim = 0.0
        last_status = time.monotonic()
        status_interval = 30.0  # wall-clock seconds between status prints

        while elapsed_sim < sim_duration and not stop_event.is_set():
            # Renderer frame
            if renderer:
                actions = renderer.handle_events()
                if actions.get('quit'):
                    break
                front = sim_robot.sensors.get_front_obstacle_distance()
                status_text = (
                    f"SIM {elapsed_sim:.0f}/{sim_duration:.0f}s | "
                    f"{state_machine.state.name} | front={front}mm"
                )
                renderer.draw(sim_robot, status_text)
                renderer.tick(60)

            # Use original sleep for frame timing (not accelerated)
            await _original_sleep(1 / 60)

            # Track sim time based on wall clock * time_scale
            wall_now = time.monotonic()
            elapsed_sim = (wall_now - start_wall) * args.time_scale

            # Periodic status
            if wall_now - last_status >= status_interval:
                last_status = wall_now
                stall_stats = experience_logger.get_recovery_statistics("escape_stall")
                coord_status = learning_coordinator.get_status_summary()
                visited = spatial_map.get_visited_percentage() * 100
                logger.info(
                    f"[{elapsed_sim:.0f}s sim] "
                    f"State={state_machine.state.name} "
                    f"Map={visited:.1f}% "
                    f"Escapes={stall_stats.get('total', 0)} "
                    f"({stall_stats.get('success_rate', 0)*100:.0f}% ok) "
                    f"Rules={coord_status['total_rules']} "
                    f"({coord_status.get('by_status', {})})"
                )

    finally:
        # Stop state machine
        await state_machine.stop()
        try:
            await asyncio.wait_for(sm_task, timeout=5.0)
        except (asyncio.TimeoutError, asyncio.CancelledError):
            sm_task.cancel()

        # Stop robot
        await sim_robot.shutdown()

        # Restore original sleep
        asyncio.sleep = _original_sleep

        wall_elapsed = time.monotonic() - start_wall

        # --- Generate report ---
        logger.info("Generating report...")
        report = generate_report(
            world_name=world.name,
            time_scale=args.time_scale,
            duration_sim=elapsed_sim,
            wall_clock_seconds=wall_elapsed,
            spatial_map=spatial_map,
            experience_logger=experience_logger,
            rules_store=rules_store,
            robot=sim_robot,
        )

        # Save report
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_path = config.DATA_DIR / f"sim_report_{timestamp}.txt"
        report_path.write_text(report, encoding='utf-8')
        logger.info(f"Report saved: {report_path}")

        # Print report
        print("\n" + report)

        # End session
        state_store.end_session(
            duration=wall_elapsed,
            distance=0,
            areas=int(spatial_map.get_visited_percentage() * 100),
            summary=f"Sim run: {world.name}, {args.time_scale}x, {elapsed_sim:.0f}s sim"
        )

        # Cleanup
        if llm_client:
            await llm_client.close()
        experience_logger.close()
        rules_store.close()
        state_store.close()

        if renderer:
            renderer.quit()

        logger.info("Simulation complete.")


def main():
    parser = argparse.ArgumentParser(description="Full-stack simulation test harness")
    parser.add_argument("--time-scale", type=float, default=10.0,
                        help="Speed up asyncio.sleep by this factor (default: 10)")
    parser.add_argument("--duration", type=float, default=3600.0,
                        help="Sim-time seconds to run (default: 3600)")
    parser.add_argument("--world", type=str, default="multi_room",
                        choices=list(WORLD_MAP.keys()),
                        help="World preset (default: multi_room)")
    parser.add_argument("--render", action="store_true", default=False,
                        help="Show pygame renderer")
    parser.add_argument("--headless", action="store_true", default=True,
                        help="No rendering (default)")
    parser.add_argument("--llm-host", type=str, default=None,
                        help="Ollama host URL")
    parser.add_argument("--llm-model", type=str, default=None,
                        help="Ollama model name")
    parser.add_argument("--no-llm", action="store_true", default=False,
                        help="Disable LLM entirely")
    args = parser.parse_args()

    asyncio.run(run_simulation(args))


if __name__ == "__main__":
    main()
