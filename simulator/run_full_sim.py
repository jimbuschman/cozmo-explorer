"""
Full-stack simulation test harness.

Runs the FULL system stack (StateMachine + LearningCoordinator + ExperienceLogger +
RulesStore + LLM + SpatialMap) using SimRobot instead of real Cozmo. Accelerated
time so hours of robot experience happen in minutes.

Single room:
    python -m simulator.run_full_sim --time-scale 5 --duration 300 --world furnished_room

Multi-room (learns across rooms):
    python -m simulator.run_full_sim --time-scale 5 --duration 300 --rooms furnished_room,corridor,dead_end

Options:
    --time-scale N    Speed up asyncio.sleep by factor N (default: 10)
    --duration N      Sim-time seconds to run PER ROOM (default: 3600)
    --world NAME      Single world preset (default: multi_room)
    --rooms LIST      Comma-separated list of worlds to run in sequence
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
from simulator.world import PRESETS, multi_room, box_room, corridor, corner, dead_end, furnished_room
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
    experience_logger: ExperienceLogger,
    rules_store: LearnedRulesStore,
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
    summary = spatial_map.get_summary()
    lines.append(f"  Map visited:     {visited_pct:.1f}%")
    lines.append(f"  Cell states:     {summary}")
    lines.append("")

    # Experience data (filtered by room_id)
    stall_stats = experience_logger.get_recovery_statistics("escape_stall", room_id=room_id)
    cliff_stats = experience_logger.get_recovery_statistics("escape_cliff", room_id=room_id)
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
    experience_logger: ExperienceLogger,
    rules_store: LearnedRulesStore,
) -> str:
    """Generate the full report with all rooms and overall summary."""
    lines = []
    lines.append("=" * 70)
    lines.append("  FULL-STACK SIMULATION REPORT")
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

    # Overall experience data
    lines.append("== OVERALL EXPERIENCE DATA ==")
    stall_stats = experience_logger.get_recovery_statistics("escape_stall")
    cliff_stats = experience_logger.get_recovery_statistics("escape_cliff")
    lines.append(f"  Stall escapes:   {stall_stats.get('total', 0)} total, "
                 f"{stall_stats.get('success_rate', 0)*100:.0f}% success")
    lines.append(f"  Cliff escapes:   {cliff_stats.get('total', 0)} total, "
                 f"{cliff_stats.get('success_rate', 0)*100:.0f}% success")
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
    rules_store: LearnedRulesStore,
    learning_coordinator: LearningCoordinator,
    state_store: StateStore,
    observer_llm,
    stop_event: asyncio.Event,
) -> dict:
    """
    Run a single room simulation.

    Returns dict with room_report, elapsed_sim, wall_seconds.
    """
    logger.info("=" * 60)
    logger.info(f"ROOM {room_index + 1}: {room_id} (world: {world_name})")
    logger.info("=" * 60)

    room_start = time.monotonic()

    # Build world
    world_fn = WORLD_MAP.get(world_name)
    if not world_fn:
        logger.error(f"Unknown world: {world_name}. Options: {list(WORLD_MAP.keys())}")
        return {'room_report': f"== ROOM: {room_id} - SKIPPED (unknown world) ==\n", 'elapsed_sim': 0, 'wall_seconds': 0}
    world = world_fn()

    # Create SimRobot for this room
    sim_robot = SimRobot(world)
    await sim_robot.start()

    # Fresh spatial map for each room
    spatial_map = SpatialMap()

    # Set room_id on experience logger
    experience_logger.set_room_id(room_id)

    # Create state machine for this room
    state_machine = StateMachine(
        robot=sim_robot,
        llm_client=observer_llm,
        memory=None,
        spatial_map=spatial_map,
        vision_observer=None,
        learning_coordinator=learning_coordinator,
        experience_logger=experience_logger,
        rules_store=rules_store,
    )

    # Adjust learning interval for time scale
    state_machine.LEARNING_INTERVAL_SECONDS = max(
        30, 300 / args.time_scale
    )

    # Start session for this room
    session_id = state_store.start_session()
    experience_logger.set_session_id(session_id)
    logger.info(f"Session {session_id} started for room {room_id}")

    # Set initial goal
    state_machine.set_goal(Goal(
        description=f"Explore {room_id} and learn survival rules",
        goal_type="explore",
    ))

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

    sm_task = asyncio.create_task(state_machine.start())

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
                status_text = (
                    f"ROOM {room_index+1} {room_id} {elapsed_sim:.0f}/{sim_duration:.0f}s | "
                    f"{state_machine.state.name} | front={front}mm"
                )
                renderer.draw(sim_robot, status_text)
                renderer.tick(60)

            await _original_sleep(1 / 60)

            wall_now = time.monotonic()
            elapsed_sim = (wall_now - room_start) * args.time_scale

            if wall_now - last_status >= status_interval:
                last_status = wall_now
                stall_stats = experience_logger.get_recovery_statistics("escape_stall", room_id=room_id)
                coord_status = learning_coordinator.get_status_summary()
                visited = spatial_map.get_visited_percentage() * 100
                logger.info(
                    f"[Room {room_id} {elapsed_sim:.0f}s sim] "
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

        await sim_robot.shutdown()

        if renderer:
            renderer.quit()

    wall_seconds = time.monotonic() - room_start

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
        experience_logger=experience_logger,
        rules_store=rules_store,
        robot=sim_robot,
    )

    logger.info(f"Room {room_id} complete: {elapsed_sim:.0f}s sim, {wall_seconds:.1f}s wall")

    return {
        'room_report': room_report,
        'elapsed_sim': elapsed_sim,
        'wall_seconds': wall_seconds,
    }


# ============================================================================
# Inter-room learning
# ============================================================================
async def run_inter_room_learning(learning_coordinator, rules_store, room_id):
    """Run a learning analysis between rooms to process what was learned."""
    logger.info(f"Running inter-room learning analysis after {room_id}...")
    try:
        # Promote any proposed rules to testing
        proposed_rules = rules_store.get_rules_by_status("proposed")
        for rule in proposed_rules:
            rules_store.update_rule_status(rule.id, "testing")
            logger.info(f"  Promoted to testing: {rule.name}")

        # Run analysis cycle
        proposals = await learning_coordinator.run_analysis_cycle()
        if proposals:
            logger.info(f"  Inter-room analysis proposed {len(proposals)} new rules")
            for rule in proposals:
                logger.info(f"    - {rule.name}: {rule.description}")
                # Immediately promote to testing
                if rule.status == "proposed":
                    rules_store.update_rule_status(rule.id, "testing")

        # Validate tested rules
        testing_rules = learning_coordinator.get_testing_rules()
        for rule_info in testing_rules:
            if rule_info['tests'] >= learning_coordinator.MIN_SAMPLES_FOR_VALIDATION:
                result = await learning_coordinator.validate_proposal(rule_info['rule_id'])
                if result:
                    status = "VALIDATED" if result.passed else "REJECTED"
                    logger.info(f"  Rule {rule_info['name']} {status}: {result.details}")

        # Activate validated rules
        activated = learning_coordinator.activate_validated_rules()
        if activated:
            logger.info(f"  Activated {len(activated)} rules for next room")

    except Exception as e:
        logger.error(f"Inter-room learning failed: {e}")


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
        # Validate all rooms exist
        for room_name in room_list:
            if room_name not in WORLD_MAP:
                logger.error(f"Unknown room: {room_name}. Options: {list(WORLD_MAP.keys())}")
                asyncio.sleep = _original_sleep
                return
    else:
        room_list = [args.world]

    logger.info(f"Room sequence: {room_list}")
    logger.info(f"Duration per room: {args.duration:.0f}s sim ({args.duration/args.time_scale:.0f}s wall)")

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

    # --- LLM clients (optional) ---
    observer_llm = None
    analysis_llm = None
    if not args.no_llm:
        from llm.client import LLMClient
        llm_host = args.llm_host or config.OLLAMA_HOST

        # Observer LLM (local default model for journaling)
        try:
            observer_llm = LLMClient(host=llm_host, model=config.OLLAMA_MODEL)
            await observer_llm.connect()
            if await observer_llm.health_check():
                logger.info(f"Observer LLM ready: {config.OLLAMA_MODEL}")
            else:
                logger.warning("Observer LLM not available")
                await observer_llm.close()
                observer_llm = None
        except Exception as e:
            logger.warning(f"Observer LLM init failed: {e}")
            observer_llm = None

        # Analysis LLM (bigger model for learning proposals)
        analysis_model = args.llm_model or config.OLLAMA_MODEL
        if analysis_model == config.OLLAMA_MODEL and observer_llm:
            analysis_llm = observer_llm
            logger.info(f"Analysis LLM: reusing observer ({analysis_model})")
        else:
            try:
                analysis_llm = LLMClient(host=llm_host, model=analysis_model)
                await analysis_llm.connect()
                if await analysis_llm.health_check():
                    logger.info(f"Analysis LLM ready: {analysis_model}")
                else:
                    logger.warning(f"Analysis LLM ({analysis_model}) not available - falling back to statistical")
                    await analysis_llm.close()
                    analysis_llm = None
            except Exception as e:
                logger.warning(f"Analysis LLM init failed: {e} - statistical proposals only")
                analysis_llm = None
    else:
        logger.info("LLM disabled (--no-llm)")

    # --- Learning coordinator (shared across rooms) ---
    learning_coordinator = LearningCoordinator(
        llm_client=analysis_llm,
        experience_logger=experience_logger,
        pattern_analyzer=pattern_analyzer,
        rules_store=rules_store,
    )

    # Adjust learning thresholds for sim speed
    learning_coordinator.MIN_SAMPLES_FOR_ANALYSIS = 10
    learning_coordinator.ANALYSIS_COOLDOWN_MINUTES = max(1, learning_coordinator.ANALYSIS_COOLDOWN_MINUTES / args.time_scale)

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

            # Room ID: "room_1_furnished_room", "room_2_corridor", etc.
            room_id = f"room_{i+1}_{world_name}"

            result = await run_room(
                room_index=i,
                room_id=room_id,
                world_name=world_name,
                args=args,
                experience_logger=experience_logger,
                rules_store=rules_store,
                learning_coordinator=learning_coordinator,
                state_store=state_store,
                observer_llm=observer_llm,
                stop_event=stop_event,
            )

            room_reports.append(result['room_report'])
            total_sim += result['elapsed_sim']
            total_wall += result['wall_seconds']

            # Run learning between rooms (not after the last one)
            if i < len(room_list) - 1 and not stop_event.is_set():
                await run_inter_room_learning(learning_coordinator, rules_store, room_id)

    finally:
        # Restore original sleep
        asyncio.sleep = _original_sleep

        wall_elapsed = time.monotonic() - start_wall

        # --- Generate final report ---
        logger.info("Generating final report...")

        # Run one last learning analysis
        if not stop_event.is_set():
            last_room_id = f"room_{len(room_reports)}_{room_list[len(room_reports)-1]}" if room_reports else "unknown"
            await run_inter_room_learning(learning_coordinator, rules_store, last_room_id)

        report = generate_final_report(
            room_reports=room_reports,
            time_scale=args.time_scale,
            total_sim_seconds=total_sim,
            total_wall_seconds=wall_elapsed,
            experience_logger=experience_logger,
            rules_store=rules_store,
        )

        # Save report
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_path = config.DATA_DIR / f"sim_report_{timestamp}.txt"
        report_path.write_text(report, encoding='utf-8')
        logger.info(f"Report saved: {report_path}")

        # Print report
        print("\n" + report)

        # Cleanup
        if observer_llm:
            await observer_llm.close()
        if analysis_llm and analysis_llm is not observer_llm:
            await analysis_llm.close()
        experience_logger.close()
        rules_store.close()
        state_store.close()

        logger.info("Simulation complete.")


def main():
    parser = argparse.ArgumentParser(description="Full-stack simulation test harness")
    parser.add_argument("--time-scale", type=float, default=10.0,
                        help="Speed up asyncio.sleep by this factor (default: 10)")
    parser.add_argument("--duration", type=float, default=3600.0,
                        help="Sim-time seconds to run PER ROOM (default: 3600)")
    parser.add_argument("--world", type=str, default="multi_room",
                        choices=list(WORLD_MAP.keys()),
                        help="Single world preset (default: multi_room)")
    parser.add_argument("--rooms", type=str, default=None,
                        help="Comma-separated list of worlds to run in sequence (e.g. furnished_room,corridor,dead_end)")
    parser.add_argument("--render", action="store_true", default=False,
                        help="Show pygame renderer")
    parser.add_argument("--headless", action="store_true", default=True,
                        help="No rendering (default)")
    parser.add_argument("--llm-host", type=str, default=None,
                        help="Ollama host URL")
    parser.add_argument("--llm-model", type=str, default=None,
                        help="Ollama model for learning analysis (observer uses default gemma3)")
    parser.add_argument("--no-llm", action="store_true", default=False,
                        help="Disable LLM entirely")
    args = parser.parse_args()

    asyncio.run(run_simulation(args))


if __name__ == "__main__":
    main()
