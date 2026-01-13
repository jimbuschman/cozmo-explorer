"""
Cozmo Explorer - Main Entry Point

An autonomous exploration system for the Cozmo robot.
"""
import asyncio
import logging
import signal
import sys
from datetime import datetime

import config
from cozmo_interface.robot import CozmoRobot
from brain.state_machine import StateMachine, RobotState, Goal
from llm.client import LLMClient
from memory.experience_db import ExperienceDB
from memory.spatial_map import SpatialMap
from memory.state_store import StateStore
from perception.vision_observer import VisionObserver
from brain.personality import CozmoPersonality, get_random_line

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s | %(levelname)-8s | %(name)-20s | %(message)s',
    datefmt='%H:%M:%S'
)
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

        self._running = False
        self._start_time: datetime = None

    async def initialize(self) -> bool:
        """Initialize all components"""
        logger.info("=" * 60)
        logger.info("  COZMO EXPLORER")
        logger.info("=" * 60)

        # Connect to state store
        logger.info("Connecting to state store...")
        self.state_store.connect()

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

        # Check LLM availability
        logger.info("Checking LLM availability...")
        await self.llm.connect()
        if await self.llm.health_check():
            logger.info(f"LLM ready: {self.llm.model}")
            # Set up alien explorer personality
            self.llm.set_personality(self.personality)
        else:
            logger.warning("LLM not available - running in autonomous-only mode")

        # Connect to robot
        logger.info("Connecting to Cozmo...")
        logger.info("(Make sure PC is connected to Cozmo's WiFi)")
        connected = await self.robot.connect()

        if not connected:
            logger.error("Failed to connect to Cozmo!")
            return False

        logger.info("Robot connected!")

        # Initialize vision observer first (state machine needs it)
        self.vision_observer = VisionObserver(
            robot=self.robot,
            llm_client=self.llm,
            experience_db=self.experiences,
            capture_interval=20.0,  # Capture every 20 seconds
            enabled=True,
            save_images=True  # Save images to data/images/
        )

        # Initialize state machine with all memory systems
        self.state_machine = StateMachine(
            robot=self.robot,
            llm_client=self.llm,
            memory=self.experiences,
            spatial_map=self.spatial_map,
            vision_observer=self.vision_observer
        )

        # Give state machine access to voice for speaking decisions
        if self.vision_observer.voice:
            self.state_machine.voice = self.vision_observer.voice

        # Set up callbacks
        self.state_machine.set_state_change_callback(self._on_state_change)
        self.state_machine.set_goal_complete_callback(self._on_goal_complete)

        # Connect vision observations to memory context
        self._setup_vision_callback()

        logger.info("Initialization complete!")

        # Log mission start to journal
        self.personality.log_milestone(
            "Mission Started",
            f"Beginning exploration. Previous experiences: {exp_count}. Systems nominal."
        )

        return True

    async def run(self):
        """Main run loop"""
        self._running = True
        self._start_time = datetime.now()

        # Start a new session
        session_id = self.state_store.start_session()
        logger.info(f"Started exploration session {session_id}")

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

            # Start state machine in background
            state_task = asyncio.create_task(self.state_machine.start())

            # Start vision observer
            await self.vision_observer.start()
            logger.info("Vision observer started - capturing images every 20s")

            # Main monitoring loop
            while self._running:
                await asyncio.sleep(0.5)

                # Update spatial map with current position
                x, y = self.robot.pose.x, self.robot.pose.y
                if abs(x - last_x) > 10 or abs(y - last_y) > 10:
                    self.spatial_map.mark_visited(x, y)
                    last_x, last_y = x, y

                # Periodic status
                if self.state_machine.context.exploration_time % 30 < 0.5:
                    self._print_status()

        except asyncio.CancelledError:
            logger.info("Run loop cancelled")
        finally:
            await self._shutdown()

    async def _shutdown(self):
        """Clean shutdown"""
        logger.info("Shutting down...")

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
        self.llm.memory.print_usage()

        # Disconnect
        await self.robot.disconnect()
        await self.llm.close()
        await self.experiences.close()
        self.state_store.close()

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

                # Add to conversation memory
                self.llm.add_observation_to_memory(observation.description)

                # Log to personality journal
                location = (self.robot.pose.x, self.robot.pose.y)
                self.personality.log_observation(observation.description, location)

            return observation

        # Replace with wrapped version
        self.vision_observer._capture_and_analyze = capture_with_context_update

    def _print_status(self):
        """Print current status"""
        robot = self.robot
        sm = self.state_machine

        logger.info("-" * 40)
        logger.info(f"State: {sm.state.name}")
        logger.info(f"Position: ({robot.pose.x:.0f}, {robot.pose.y:.0f})")
        logger.info(f"Battery: {robot.sensors.battery_voltage:.2f}V")
        logger.info(f"Exploration: {sm.context.exploration_time:.0f}s")
        logger.info(f"Map: {self.spatial_map.get_visited_percentage()*100:.1f}% visited")

        # Show recent observations
        if self.vision_observer and self.vision_observer.recent_observations:
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
