"""
Cozmo Explorer - Main Entry Point

An autonomous exploration system for the Cozmo robot.
"""
import asyncio
import logging
import math
import os
import signal
import sys
from datetime import datetime
from typing import Optional

import config
from perception.external_sensors import ExternalSensorReader
from cozmo_interface.robot import CozmoRobot
from brain.state_machine import StateMachine, RobotState, Goal
from llm.client import LLMClient
from memory.experience_db import ExperienceDB
from memory.spatial_map import SpatialMap
from memory.state_store import StateStore
from perception.vision_observer import VisionObserver
from brain.personality import CozmoPersonality, get_random_line

# Learning system components
from memory.experience_logger import ExperienceLogger
from memory.pattern_analyzer import PatternAnalyzer
from memory.learned_rules import LearnedRulesStore
from brain.learning_coordinator import LearningCoordinator
from control.manual_controller import ManualController

# Configure logging - both screen and file
log_format = '%(asctime)s | %(levelname)-8s | %(name)-20s | %(message)s'
log_datefmt = '%H:%M:%S'

# Set up root logger
logging.basicConfig(
    level=logging.DEBUG,  # Capture DEBUG for file
    format=log_format,
    datefmt=log_datefmt,
    handlers=[
        # Console handler (INFO and above)
        logging.StreamHandler(),
        # File handler (DEBUG and above)
        logging.FileHandler(config.DATA_DIR / 'cozmo.log', mode='w', encoding='utf-8')
    ]
)

# Set console to INFO only (less noisy)
logging.getLogger().handlers[0].setLevel(logging.INFO)

logger = logging.getLogger(__name__)


class CozmoExplorer:
    """
    Main application class that coordinates all components.
    """

    def __init__(self):
        self.robot = CozmoRobot()
        self.llm = LLMClient()
        self.experiences = ExperienceDB()
        self.spatial_map = SpatialMap()
        self.state_store = StateStore()
        self.state_machine: StateMachine = None
        self.vision_observer: VisionObserver = None
        self.personality = CozmoPersonality()  # Alien explorer mode!
        self.external_sensors: Optional[ExternalSensorReader] = None

        # Learning system components
        self.experience_logger: Optional[ExperienceLogger] = None
        self.rules_store: Optional[LearnedRulesStore] = None
        self.pattern_analyzer: Optional[PatternAnalyzer] = None
        self.learning_coordinator: Optional[LearningCoordinator] = None

        # Manual control
        self.manual_controller: Optional[ManualController] = None
        self._manual_mode = False  # True when in manual mode

        self._running = False
        self._start_time: datetime = None

    async def initialize(self) -> bool:
        """Initialize all components"""
        logger.info("=" * 60)
        logger.info("  COZMO EXPLORER")
        if config.PHASE1_PURE_SURVIVAL:
            logger.info("  Phase 1: Grounded Survival Learning")
        else:
            logger.info("  Phase 2+: LLM-Directed Exploration")
        logger.info("=" * 60)

        # Connect to state store
        logger.info("Connecting to state store...")
        self.state_store.connect()

        # Initialize learning system components
        logger.info("Initializing learning system...")
        self.experience_logger = ExperienceLogger()
        self.experience_logger.connect()

        self.rules_store = LearnedRulesStore()
        self.rules_store.connect()

        self.pattern_analyzer = PatternAnalyzer(self.experience_logger)

        # Learning coordinator will be created after LLM is ready

        # Load previous map if exists
        map_file = config.DATA_DIR / "spatial_map.npz"
        if map_file.exists():
            try:
                self.spatial_map = SpatialMap.load(str(map_file))
                logger.info("Loaded previous spatial map")
            except Exception as e:
                logger.warning(f"Could not load map: {e}")

        # Connect to experience database
        logger.info("Connecting to experience database...")
        await self.experiences.connect()
        exp_count = await self.experiences.count()
        logger.info(f"Loaded {exp_count} previous experiences")

        # Check LLM availability (optional in Phase 1)
        if config.ENABLE_LLM:
            logger.info("Checking LLM availability...")
            try:
                await self.llm.connect()
                if await self.llm.health_check():
                    logger.info(f"LLM ready: {self.llm.model}")
                    # Set up alien explorer personality
                    self.llm.set_personality(self.personality)
                    if config.PHASE1_PURE_SURVIVAL:
                        logger.info("Phase 1 mode: LLM will observe and journal only")
                    else:
                        logger.info("Phase 2+ mode: LLM will provide guidance")
                else:
                    logger.warning("LLM not available - running in autonomous-only mode")
                    self.llm = None
            except Exception as e:
                logger.warning(f"LLM initialization failed: {e} - running without LLM")
                self.llm = None
        else:
            logger.info("LLM disabled by config - running in pure autonomous mode")
            self.llm = None

        # Create learning coordinator (works with or without LLM)
        self.learning_coordinator = LearningCoordinator(
            llm_client=self.llm,
            experience_logger=self.experience_logger,
            pattern_analyzer=self.pattern_analyzer,
            rules_store=self.rules_store
        )
        logger.info("Learning coordinator initialized")

        # Connect to robot
        logger.info("Connecting to Cozmo...")
        logger.info("(Make sure PC is connected to Cozmo's WiFi)")
        connected = await self.robot.connect()

        if not connected:
            logger.error("Failed to connect to Cozmo!")
            return False

        logger.info("Robot connected!")

        # Connect external sensors (ESP32 pod)
        ext_mode = os.environ.get("EXT_SENSOR_MODE", config.EXT_SENSOR_MODE)
        ext_port = os.environ.get("EXT_SENSOR_PORT", config.EXT_SENSOR_PORT)
        ext_udp_port = int(os.environ.get("EXT_SENSOR_UDP_PORT", config.EXT_SENSOR_UDP_PORT))

        if ext_mode == "udp":
            logger.info(f"Connecting external sensors via WiFi UDP (port {ext_udp_port})...")
            self.external_sensors = ExternalSensorReader(
                mode="udp",
                udp_port=ext_udp_port
            )
        else:
            logger.info(f"Connecting external sensors via serial ({ext_port})...")
            self.external_sensors = ExternalSensorReader(
                mode="serial",
                port=ext_port,
                baud=config.EXT_SENSOR_BAUD
            )

        if self.external_sensors.start():
            logger.info("External sensors connected!")
            self.robot._sensors.ext_connected = True

            def on_ext_reading(reading):
                s = self.robot._sensors
                # Distance sensors
                s.ext_tof_mm = reading.tof_mm
                s.ext_ultra_l_mm = reading.ultra_l_mm
                s.ext_ultra_c_mm = reading.ultra_c_mm
                s.ext_ultra_r_mm = reading.ultra_r_mm
                # Orientation - recompute from raw accel with mounting correction
                # (bypasses ESP32's complementary filter which drifts with inverted axes)
                ax = reading.ax_g * config.IMU_ACCEL_X_SIGN
                ay = reading.ay_g * config.IMU_ACCEL_Y_SIGN
                az = reading.az_g * config.IMU_ACCEL_Z_SIGN
                s.ext_pitch = math.atan2(ay, math.sqrt(ax * ax + az * az)) * 180.0 / math.pi
                s.ext_roll = math.atan2(-ax, az) * 180.0 / math.pi
                s.ext_yaw = reading.yaw  # Yaw still from ESP32 gyro integration
                # Raw IMU data (uncorrected)
                s.ext_ax_g = reading.ax_g
                s.ext_ay_g = reading.ay_g
                s.ext_az_g = reading.az_g
                s.ext_gx_dps = reading.gx_dps
                s.ext_gy_dps = reading.gy_dps
                s.ext_gz_dps = reading.gz_dps
                s.ext_ts_ms = reading.ts_ms

            self.external_sensors.set_callback(on_ext_reading)
        else:
            logger.warning("External sensors not available - reactive collision detection only")

        # Initialize vision observer first (state machine needs it)
        self.vision_observer = VisionObserver(
            robot=self.robot,
            llm_client=self.llm,
            experience_db=self.experiences,
            capture_interval=20.0,  # Capture every 20 seconds
            enabled=True,
            save_images=True  # Save images to data/images/
        )

        # Initialize state machine with all memory systems and learning
        self.state_machine = StateMachine(
            robot=self.robot,
            llm_client=self.llm,
            memory=self.experiences,
            spatial_map=self.spatial_map,
            vision_observer=self.vision_observer,
            learning_coordinator=self.learning_coordinator,
            experience_logger=self.experience_logger,
            rules_store=self.rules_store
        )

        # Give state machine access to voice for speaking decisions
        if self.vision_observer.voice:
            self.state_machine.voice = self.vision_observer.voice

        # Set up callbacks
        self.state_machine.set_state_change_callback(self._on_state_change)
        self.state_machine.set_goal_complete_callback(self._on_goal_complete)

        # Connect vision observations to memory context
        self._setup_vision_callback()

        # Initialize manual controller if enabled
        if config.MANUAL_CONTROL_ENABLED:
            logger.info("Manual control enabled - creating controller")
            self.manual_controller = ManualController(
                robot=self.robot,
                experience_logger=self.experience_logger,
                rules_store=self.rules_store,
                on_mode_change=self._on_manual_mode_change
            )
            self._manual_mode = True
        else:
            self._manual_mode = False

        logger.info("Initialization complete!")

        # Log mission start to journal
        trailer_status = "with trailer" if config.TRAILER_MODE else "without trailer"
        manual_status = "manual control" if config.MANUAL_CONTROL_ENABLED else "autonomous"
        self.personality.log_milestone(
            "Mission Started",
            f"Beginning exploration ({manual_status}, {trailer_status}). Previous experiences: {exp_count}. Systems nominal."
        )

        return True

    async def run(self):
        """Main run loop"""
        self._running = True
        self._start_time = datetime.now()

        # Start a new session
        session_id = self.state_store.start_session()
        logger.info(f"Started exploration session {session_id}")

        # Set session ID for experience logging
        if self.experience_logger:
            self.experience_logger.set_session_id(session_id)

        # Set initial goal
        self.state_machine.set_goal(Goal(
            description="Explore and map the environment",
            goal_type="explore"
        ))

        # Position tracking for map updates
        last_x, last_y = 0, 0

        try:
            # Set head to look forward (not down)
            await self.robot.set_head_angle(0.35)  # Look up/forward
            logger.info("Head positioned for exploration")

            # Start manual controller GUI if enabled
            if self.manual_controller:
                logger.info("Starting manual control GUI...")
                loop = asyncio.get_event_loop()
                self.manual_controller.start(loop)

                # Run GUI in a separate thread
                import threading
                gui_thread = threading.Thread(target=self.manual_controller.run_gui, daemon=True)
                gui_thread.start()
                logger.info("Manual control GUI running in background")

            # Start state machine in background (if not in manual mode)
            state_task = None
            if not self._manual_mode:
                state_task = asyncio.create_task(self.state_machine.start())

            # Start vision observer
            await self.vision_observer.start()
            logger.info("Vision observer started - capturing images every 20s")

            # Main monitoring loop
            last_status_time = datetime.now()
            while self._running:
                await asyncio.sleep(0.5)

                # Check if manual controller closed
                if self.manual_controller and not self.manual_controller.is_running:
                    logger.info("Manual controller closed - exiting")
                    break

                # Refresh pose from pycozmo
                self.robot._update_pose_from_client()

                # Update spatial map with current position (even in manual mode)
                x, y = self.robot.pose.x, self.robot.pose.y
                if abs(x - last_x) > 10 or abs(y - last_y) > 10:
                    self.spatial_map.mark_visited(x, y)
                    last_x, last_y = x, y

                # Start/stop state machine based on mode changes
                if not self._manual_mode and state_task is None:
                    logger.info("Starting autonomous behaviors...")
                    state_task = asyncio.create_task(self.state_machine.start())
                elif self._manual_mode and state_task is not None:
                    # Will be handled by pause/resume in state machine
                    pass

                # Periodic status (every 30 seconds)
                now = datetime.now()
                if (now - last_status_time).total_seconds() >= 30:
                    self._print_status()
                    last_status_time = now

        except asyncio.CancelledError:
            logger.info("Run loop cancelled")
        finally:
            await self._shutdown()

    async def _shutdown(self):
        """Clean shutdown"""
        logger.info("Shutting down...")

        # Stop manual controller
        if self.manual_controller:
            self.manual_controller.stop()

        # Stop vision observer
        if self.vision_observer:
            await self.vision_observer.stop()

        # Stop state machine
        if self.state_machine:
            await self.state_machine.stop()

        # Save state
        duration = (datetime.now() - self._start_time).total_seconds() if self._start_time else 0

        self.state_store.save_robot_state(
            position={'x': self.robot.pose.x, 'y': self.robot.pose.y, 'angle': self.robot.pose.angle},
            battery=self.robot.sensors.battery_voltage,
            exploration_time=self.state_machine.context.exploration_time if self.state_machine else 0
        )

        self.state_store.end_session(
            duration=duration,
            distance=self.state_machine.context.exploration_time * 50 if self.state_machine else 0,  # rough estimate
            areas=int(self.spatial_map.get_visited_percentage() * 100),
            summary=f"Explored for {duration:.0f}s"
        )

        # Save map
        try:
            self.spatial_map.save(str(config.DATA_DIR / "spatial_map.npz"))
        except Exception as e:
            logger.error(f"Failed to save map: {e}")

        # Log mission end to journal
        self.personality.log_milestone(
            "Mission Ended",
            f"Exploration session complete. Duration: {duration:.0f}s. "
            f"Area visited: {self.spatial_map.get_visited_percentage()*100:.1f}%"
        )

        # Print final memory usage
        if self.llm:
            self.llm.memory.print_usage()

        # Print learning system status
        if self.learning_coordinator:
            status = self.learning_coordinator.get_status_summary()
            logger.info(f"Learning system: {status['total_rules']} rules, {status['data_samples']} samples")
            if status['by_status']:
                logger.info(f"  Rules by status: {status['by_status']}")

        # Stop external sensors
        if self.external_sensors:
            self.external_sensors.stop()

        # Disconnect and close learning components
        await self.robot.disconnect()
        if self.llm:
            await self.llm.close()
        await self.experiences.close()
        self.state_store.close()

        if self.experience_logger:
            self.experience_logger.close()
        if self.rules_store:
            self.rules_store.close()

        logger.info("Shutdown complete")

    def _on_state_change(self, old_state: RobotState, new_state: RobotState):
        """Handle state changes"""
        self.state_store.log_event(
            "state_change",
            f"{old_state.name} -> {new_state.name}",
            {"from": old_state.name, "to": new_state.name}
        )

    def _on_goal_complete(self, goal: Goal, success: bool):
        """Handle goal completion"""
        logger.info(f"Goal {'completed' if success else 'failed'}: {goal.description}")
        self.state_store.log_event(
            "goal_complete",
            goal.description,
            {"success": success, "goal_type": goal.goal_type}
        )

        # Update memory context with outcome
        if self.state_machine and self.state_machine.memory_context:
            outcome = "completed" if success else "failed"
            self.state_machine.memory_context.update_last_decision_outcome(outcome, success)

    def _on_manual_mode_change(self, is_manual: bool):
        """Handle switching between manual and auto modes"""
        self._manual_mode = is_manual
        if is_manual:
            logger.info("Switched to MANUAL mode - autonomous behaviors paused")
            if self.state_machine:
                self.state_machine.pause()
        else:
            logger.info("Switched to AUTO mode - autonomous behaviors resumed")
            if self.state_machine:
                self.state_machine.resume()

    def _setup_vision_callback(self):
        """Set up callback to feed vision observations to memory context"""
        if not self.vision_observer:
            return

        # Store original method
        original_capture = self.vision_observer._capture_and_analyze

        async def capture_with_context_update():
            """Wrapper that updates memory context after capture"""
            observation = await original_capture()
            if observation and self.state_machine:
                # Update memory context with current observation
                self.state_machine.memory_context.set_current_observation(
                    observation.description
                )

                # Add to conversation memory (if LLM available)
                if self.llm:
                    self.llm.add_observation_to_memory(observation.description)

                # Log to personality journal
                location = (self.robot.pose.x, self.robot.pose.y)
                self.personality.log_observation(observation.description, location)

            return observation

        # Replace with wrapped version
        self.vision_observer._capture_and_analyze = capture_with_context_update

    def _print_status(self):
        """Print current status - Phase 1 focused"""
        robot = self.robot
        sm = self.state_machine

        logger.info("-" * 40)

        # Show mode
        mode = "Phase 1 (Survival)" if config.PHASE1_PURE_SURVIVAL else "Phase 2+ (Directed)"
        llm_status = "observer" if config.LLM_OBSERVER_MODE else "director"
        if not self.llm:
            llm_status = "disabled"
        control_mode = "MANUAL" if self._manual_mode else "AUTO"
        trailer_status = "TRAILER" if config.TRAILER_MODE else ""
        logger.info(f"Mode: {mode} | LLM: {llm_status} | Control: {control_mode} {trailer_status}")

        logger.info(f"State: {sm.state.name}")
        logger.info(f"Position: ({robot.pose.x:.0f}, {robot.pose.y:.0f})")
        logger.info(f"Battery: {robot.sensors.battery_voltage:.2f}V")
        logger.info(f"Exploration: {sm.context.exploration_time:.0f}s")
        logger.info(f"Map: {self.spatial_map.get_visited_percentage()*100:.1f}% visited")

        # Show external sensor status
        if self.robot.sensors.ext_connected:
            s = self.robot.sensors
            dists = s.get_obstacle_distances()
            logger.info(f"Obstacles: F={dists['front']}mm L={dists['left']}mm R={dists['right']}mm")
            logger.info(f"IMU: pitch={s.ext_pitch:.1f}° roll={s.ext_roll:.1f}° yaw={s.ext_yaw:.1f}°")

        # Show learning system status (Phase 1 focus)
        if self.learning_coordinator:
            status = self.learning_coordinator.get_status_summary()
            active_rules = status['by_status'].get('active', 0)
            testing_rules = status['testing']
            proposed_rules = status['by_status'].get('proposed', 0)
            logger.info(f"Learning: {status['data_samples']} samples | Rules: {active_rules} active, {testing_rules} testing, {proposed_rules} proposed")

            # Show recovery success rate if available
            if self.experience_logger:
                stats = self.experience_logger.get_recovery_statistics("escape_stall")
                if stats.get('total', 0) > 0:
                    logger.info(f"Recovery: {stats['total']} attempts, {stats.get('success_rate', 0)*100:.0f}% success rate")

        # Show recent observations (less prominent in Phase 1)
        if not config.PHASE1_PURE_SURVIVAL and self.vision_observer and self.vision_observer.recent_observations:
            recent = self.vision_observer.get_recent_descriptions(3)
            if recent:
                logger.info("Recent observations:")
                for obs in recent:
                    logger.info(f"  - {obs[:60]}...")

        logger.info("-" * 40)

    def stop(self):
        """Request stop"""
        self._running = False


async def main():
    """Main entry point"""
    explorer = CozmoExplorer()

    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        logger.info("Interrupt received, stopping...")
        explorer.stop()

    signal.signal(signal.SIGINT, signal_handler)

    # Initialize
    if not await explorer.initialize():
        logger.error("Initialization failed!")
        return 1

    # Run
    try:
        await explorer.run()
    except Exception as e:
        logger.error(f"Error: {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)
