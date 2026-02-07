"""
State Machine

The core "brain" that manages robot states and decides what to do.
Coordinates between behaviors, memory, and LLM guidance.
"""
import asyncio
import logging
from enum import Enum, auto
from typing import Optional, Callable, Any, TYPE_CHECKING
from dataclasses import dataclass, field
from datetime import datetime

import config
from cozmo_interface.robot import CozmoRobot
from brain.behaviors import (
    Behavior, BehaviorFactory, BehaviorStatus, BehaviorResult
)
from brain.memory_context import MemoryContext
from llm.client import LLMDecision

if TYPE_CHECKING:
    from brain.learning_coordinator import LearningCoordinator
    from memory.experience_logger import ExperienceLogger
    from memory.learned_rules import LearnedRulesStore

logger = logging.getLogger(__name__)


class RobotState(Enum):
    """High-level robot states"""
    IDLE = auto()           # Waiting, not doing anything
    EXPLORING = auto()      # Actively exploring environment
    INVESTIGATING = auto()  # Looking closely at something interesting
    GOING_TO = auto()       # Navigating to a specific location
    STUCK = auto()          # Can't make progress
    CHARGING = auto()       # On charger, battery low
    WAITING_FOR_LLM = auto() # Waiting for LLM guidance
    ERROR = auto()          # Something went wrong


@dataclass
class Goal:
    """A goal set by the LLM or internally"""
    description: str
    goal_type: str  # "explore", "go_to", "investigate", "idle"
    parameters: dict = field(default_factory=dict)
    created_at: datetime = field(default_factory=datetime.now)
    priority: int = 1  # 1 = highest


@dataclass
class StateContext:
    """Context passed between states"""
    current_goal: Optional[Goal] = None
    last_behavior_result: Optional[BehaviorResult] = None
    stuck_count: int = 0
    exploration_time: float = 0.0
    last_llm_query: Optional[datetime] = None
    interesting_observations: list = field(default_factory=list)


class StateMachine:
    """
    Manages robot state transitions and behavior execution.

    The state machine runs in a loop:
    1. Check sensors/status
    2. Decide if state should change
    3. Execute appropriate behavior for current state
    4. Update memory/context
    5. Occasionally query LLM for guidance
    6. Periodically run learning analysis
    """

    # Learning configuration
    LEARNING_INTERVAL_SECONDS = 300  # Run learning every 5 minutes

    def __init__(
        self,
        robot: CozmoRobot,
        llm_client=None,
        memory=None,
        spatial_map=None,
        vision_observer=None,
        learning_coordinator: "LearningCoordinator" = None,
        experience_logger: "ExperienceLogger" = None,
        rules_store: "LearnedRulesStore" = None
    ):
        self.robot = robot
        self.llm_client = llm_client
        self.memory = memory  # ExperienceDB
        self.spatial_map = spatial_map
        self.vision_observer = vision_observer

        # Learning system components
        self.learning_coordinator = learning_coordinator
        self.experience_logger = experience_logger
        self.rules_store = rules_store

        # Create behavior factory with learning integration
        self.behaviors = BehaviorFactory(
            robot,
            experience_logger=experience_logger,
            rules_store=rules_store
        )
        self.state = RobotState.IDLE
        self.context = StateContext()

        self._running = False
        self._paused = False
        self._current_behavior: Optional[Behavior] = None
        self._last_learning_time: Optional[datetime] = None

        # Memory context builder for rich LLM queries
        self.memory_context = MemoryContext(
            experience_db=memory,
            spatial_map=spatial_map,
            vision_observer=vision_observer
        )

        # Voice for speaking (will be set from vision_observer if available)
        self.voice = None

        # Callbacks for external monitoring
        self._on_state_change: Optional[Callable] = None
        self._on_goal_complete: Optional[Callable] = None

        # Configuration
        self.llm_query_interval = config.LLM_QUERY_INTERVAL
        self.max_stuck_attempts = 3
        self.low_battery_threshold = config.LOW_BATTERY_VOLTAGE

    @property
    def is_running(self) -> bool:
        return self._running

    def set_state_change_callback(self, callback: Callable):
        """Set callback for state changes: callback(old_state, new_state)"""
        self._on_state_change = callback

    def set_goal_complete_callback(self, callback: Callable):
        """Set callback for goal completion: callback(goal, success)"""
        self._on_goal_complete = callback

    async def start(self):
        """Start the state machine loop"""
        if self._running:
            logger.warning("State machine already running")
            return

        self._running = True
        logger.info("Starting state machine")

        try:
            await self._run_loop()
        except Exception as e:
            logger.error(f"State machine error: {e}")
            self.state = RobotState.ERROR
        finally:
            self._running = False

    async def stop(self):
        """Stop the state machine"""
        logger.info("Stopping state machine")
        self._running = False

        if self._current_behavior:
            self._current_behavior.cancel()

        await self.robot.stop()

    def pause(self):
        """Pause the state machine (for manual control)"""
        if not self._paused:
            logger.info("State machine paused")
            self._paused = True
            if self._current_behavior:
                self._current_behavior.cancel()

    def resume(self):
        """Resume the state machine after pause"""
        if self._paused:
            logger.info("State machine resumed")
            self._paused = False

    @property
    def is_paused(self) -> bool:
        """Check if state machine is paused"""
        return self._paused

    def set_goal(self, goal: Goal):
        """Set a new goal for the robot"""
        logger.info(f"New goal: {goal.description}")
        self.context.current_goal = goal
        self.context.stuck_count = 0

    async def _run_loop(self):
        """Main state machine loop"""
        while self._running:
            # Check if paused (manual control mode)
            if self._paused:
                await asyncio.sleep(0.5)
                continue

            # Check for safety conditions first
            if await self._check_safety():
                continue

            # Phase 1: LLM observes but doesn't direct
            # Phase 2+: LLM provides guidance
            if config.LLM_OBSERVER_MODE:
                # Phase 1: LLM only observes and journals
                if await self._should_llm_observe():
                    # Run observation in background (non-blocking)
                    asyncio.create_task(self._llm_observe_only())
            else:
                # Phase 2+: LLM sets goals
                if await self._should_query_llm():
                    await self._query_llm_for_guidance()

            # Check if we should run learning cycle
            if await self._should_run_learning():
                await self._run_learning_cycle()

            # Execute behavior based on current state
            await self._execute_state()

            # Small delay to prevent tight loop
            await asyncio.sleep(0.1)

    async def _should_run_learning(self) -> bool:
        """Check if it's time to run learning analysis"""
        if not self.learning_coordinator:
            return False

        # Don't run during safety-critical states
        if self.state in (RobotState.STUCK, RobotState.CHARGING, RobotState.ERROR):
            return False

        # Check interval
        if self._last_learning_time:
            elapsed = (datetime.now() - self._last_learning_time).total_seconds()
            if elapsed < self.LEARNING_INTERVAL_SECONDS:
                return False

        # Let the coordinator decide based on data availability
        return self.learning_coordinator.should_run_analysis()

    async def _run_learning_cycle(self):
        """Run the learning analysis cycle"""
        if not self.learning_coordinator:
            return

        logger.info("Running learning analysis cycle")

        try:
            # Move proposed rules to testing so they enter the testing pipeline
            if self.rules_store:
                proposed_rules = self.rules_store.get_rules_by_status("proposed")
                for rule in proposed_rules:
                    self.rules_store.update_rule_status(rule.id, "testing")
                    # Initialize testing stats in the coordinator
                    if self.learning_coordinator and rule.id not in self.learning_coordinator._testing_rules:
                        self.learning_coordinator._testing_rules[rule.id] = {
                            'started': datetime.now(),
                            'tests': 0,
                            'successes': 0,
                            'baseline_rate': 0.5
                        }
                    logger.info(f"Promoted rule to testing: {rule.name} (id={rule.id})")

            # Run analysis and get proposals
            proposals = await self.learning_coordinator.run_analysis_cycle()

            if proposals:
                logger.info(f"Learning cycle proposed {len(proposals)} new rules")
                for rule in proposals:
                    logger.info(f"  - {rule.name}: {rule.description}")

            # Check and validate any rules that have been tested enough
            testing_rules = self.learning_coordinator.get_testing_rules()
            for rule_info in testing_rules:
                if rule_info['tests'] >= self.learning_coordinator.MIN_SAMPLES_FOR_VALIDATION:
                    result = await self.learning_coordinator.validate_proposal(rule_info['rule_id'])
                    if result:
                        if result.passed:
                            logger.info(f"Rule {rule_info['name']} validated: {result.details}")
                        else:
                            logger.info(f"Rule {rule_info['name']} rejected: {result.details}")

            # Activate any newly validated rules
            activated = self.learning_coordinator.activate_validated_rules()
            if activated:
                logger.info(f"Activated {len(activated)} learned rules")

            self._last_learning_time = datetime.now()

        except Exception as e:
            logger.error(f"Learning cycle failed: {e}")

    async def _check_safety(self) -> bool:
        """
        Check safety conditions that override normal operation.
        Returns True if a safety condition was handled.
        """
        # Check if picked up
        if self.robot.sensors.is_picked_up:
            await self._change_state(RobotState.IDLE)
            await self.robot.stop()
            logger.info("Robot picked up - going idle")
            await asyncio.sleep(1.0)
            return True

        # Check battery
        if self.robot.sensors.battery_voltage > 0 and self.robot.sensors.battery_voltage < self.low_battery_threshold:
            if self.robot.sensors.is_on_charger:
                await self._change_state(RobotState.CHARGING)
                return True
            elif self.state != RobotState.IDLE:
                # Low battery but not on charger - go idle and warn once
                logger.warning(f"Low battery ({self.robot.sensors.battery_voltage:.2f}V)! Place on charger.")
                await self._change_state(RobotState.IDLE)
                await self.robot.stop()
                return True
            # Already idle with low battery - don't block, just wait
            await asyncio.sleep(2.0)
            return True

        # Check if on charger and charging complete
        if self.state == RobotState.CHARGING:
            if self.robot.sensors.battery_voltage > 4.0:
                logger.info("Charging complete!")
                await self._change_state(RobotState.IDLE)
            return True

        return False

    async def _should_query_llm(self) -> bool:
        """Determine if we should ask the LLM for guidance (Phase 2+)"""
        # Phase 1: LLM doesn't set goals, only observes
        if not config.LLM_GOAL_SETTING_ENABLED:
            return False

        if self.llm_client is None:
            return False

        # Query if we have no goal
        if self.context.current_goal is None:
            return True

        # Query if we're stuck
        if self.state == RobotState.STUCK:
            return True

        # Query periodically
        if self.context.last_llm_query is None:
            return True

        elapsed = (datetime.now() - self.context.last_llm_query).total_seconds()
        return elapsed > self.llm_query_interval

    async def _should_llm_observe(self) -> bool:
        """Check if LLM should make an observation (Phase 1 only)"""
        if not config.LLM_OBSERVER_MODE:
            return False

        if self.llm_client is None:
            return False

        # Less frequent than goal-setting
        if self.context.last_llm_query is None:
            return True

        elapsed = (datetime.now() - self.context.last_llm_query).total_seconds()
        return elapsed > config.LLM_OBSERVER_INTERVAL

    async def _llm_observe_only(self):
        """
        Let LLM observe and journal without setting goals (Phase 1).

        The LLM acts as a passive observer, noting interesting patterns
        and journaling observations, but doesn't direct the robot's actions.
        """
        if self.llm_client is None:
            return

        try:
            # Build simple context for observation
            observation_prompt = f"""You are observing a robot learning to survive and navigate.
Do NOT give commands or set goals. Simply observe and note anything interesting.

Current state: {self.state.name}
Exploration time: {self.context.exploration_time:.1f}s
Stuck count: {self.context.stuck_count}
Position: ({self.robot.pose.x:.0f}, {self.robot.pose.y:.0f})
Battery: {self.robot.sensors.battery_voltage:.2f}V

External sensors connected: {self.robot.sensors.ext_connected}
"""
            if self.robot.sensors.ext_connected:
                dists = self.robot.sensors.get_obstacle_distances()
                observation_prompt += f"""Front distance: {dists['front']}mm
Left distance: {dists['left']}mm
Right distance: {dists['right']}mm
"""

            observation_prompt += """
Write a brief journal observation (1-2 sentences). Focus on:
- What the robot seems to be learning
- Any patterns you notice
- Interesting sensor readings or behaviors

Respond with just the observation text, no commands or suggestions."""

            # Get observation from LLM
            observation = await self.llm_client._chat(observation_prompt)

            if observation:
                observation = observation.strip()
                logger.debug(f"LLM observation: {observation[:100]}...")

                # Log to journal if memory context available
                if self.memory_context:
                    self.memory_context.add_observation(
                        f"[Observer] {observation}"
                    )

            self.context.last_llm_query = datetime.now()

        except Exception as e:
            logger.debug(f"LLM observation failed: {e}")

    async def _query_llm_for_guidance(self):
        """Ask the LLM what to do next using rich memory context"""
        if self.llm_client is None:
            # No LLM, use default behavior
            self.set_goal(Goal(
                description="Explore the environment",
                goal_type="explore"
            ))
            return

        old_state = self.state
        await self._change_state(RobotState.WAITING_FOR_LLM)

        try:
            # Build rich context from all memory systems
            context_snapshot = await self.memory_context.build_context(
                robot=self.robot,
                current_state=self.state.name,
                exploration_time=self.context.exploration_time,
                stuck_count=self.context.stuck_count
            )

            # Format context for LLM
            context_prompt = self.memory_context.format_for_llm(context_snapshot)

            # Get structured decision from LLM
            decision: LLMDecision = await self.llm_client.get_decision(context_prompt)

            if decision.is_valid():
                # Record the decision in memory
                self.memory_context.record_decision(
                    action=decision.action,
                    target=decision.target,
                    reasoning=decision.reasoning
                )

                # Convert decision to goal
                goal = Goal(
                    description=decision.reasoning or f"{decision.action}: {decision.target or 'general'}",
                    goal_type=decision.action,
                    parameters={"target": decision.target} if decision.target else {}
                )
                self.set_goal(goal)

                # Speak if there's something to say (and voice is enabled)
                if decision.speak and self.voice and config.ENABLE_VOICE:
                    try:
                        await self.voice.speak(decision.speak)
                    except Exception as e:
                        logger.debug(f"Could not speak: {e}")

                logger.info(f"LLM decided: {decision.action} -> {decision.target or 'general'}")
                logger.info(f"Reasoning: {decision.reasoning}")
            else:
                logger.warning(f"Invalid decision from LLM: {decision.action}")

            self.context.last_llm_query = datetime.now()

        except Exception as e:
            logger.error(f"LLM query failed: {e}")

        # Return to previous state or idle
        if self.context.current_goal:
            await self._change_state(self._goal_to_state(self.context.current_goal))
        else:
            await self._change_state(old_state if old_state != RobotState.WAITING_FOR_LLM else RobotState.IDLE)

    def _build_state_summary(self) -> dict:
        """Build a summary of current state for the LLM"""
        return {
            "state": self.state.name,
            "position": {
                "x": self.robot.pose.x,
                "y": self.robot.pose.y,
                "angle_degrees": self.robot.pose.angle * 57.3
            },
            "sensors": {
                "battery": self.robot.sensors.battery_voltage,
                "on_charger": self.robot.sensors.is_on_charger,
                "cliff_detected": self.robot.sensors.cliff_detected
            },
            "context": {
                "exploration_time": self.context.exploration_time,
                "stuck_count": self.context.stuck_count,
                "current_goal": self.context.current_goal.description if self.context.current_goal else None,
                "interesting_observations": self.context.interesting_observations[-5:]
            }
        }

    def _parse_llm_response(self, response: str) -> Optional[Goal]:
        """Parse LLM response into a Goal"""
        # Simple parsing - in practice, use structured output
        response_lower = response.lower()

        if "explore" in response_lower:
            return Goal(
                description=response,
                goal_type="explore"
            )
        elif "investigate" in response_lower or "look" in response_lower:
            return Goal(
                description=response,
                goal_type="investigate"
            )
        elif "go to" in response_lower or "navigate" in response_lower:
            return Goal(
                description=response,
                goal_type="go_to"
            )
        elif "wait" in response_lower or "idle" in response_lower:
            return Goal(
                description=response,
                goal_type="idle"
            )

        # Default to explore
        return Goal(
            description=response,
            goal_type="explore"
        )

    def _goal_to_state(self, goal: Goal) -> RobotState:
        """Convert a goal type to a robot state"""
        mapping = {
            "explore": RobotState.EXPLORING,
            "investigate": RobotState.INVESTIGATING,
            "go_to": RobotState.GOING_TO,
            "idle": RobotState.IDLE
        }
        return mapping.get(goal.goal_type, RobotState.IDLE)

    async def _change_state(self, new_state: RobotState):
        """Change to a new state"""
        if new_state == self.state:
            return

        old_state = self.state
        self.state = new_state
        logger.info(f"State: {old_state.name} -> {new_state.name}")

        if self._on_state_change:
            self._on_state_change(old_state, new_state)

    async def _execute_state(self):
        """Execute behavior for current state"""
        if self.state == RobotState.IDLE:
            await self._do_idle()

        elif self.state == RobotState.EXPLORING:
            await self._do_explore()

        elif self.state == RobotState.INVESTIGATING:
            await self._do_investigate()

        elif self.state == RobotState.GOING_TO:
            await self._do_go_to()

        elif self.state == RobotState.STUCK:
            await self._do_stuck()

        elif self.state == RobotState.CHARGING:
            await self._do_charging()

        elif self.state == RobotState.WAITING_FOR_LLM:
            await asyncio.sleep(0.5)

        elif self.state == RobotState.ERROR:
            await self.robot.stop()
            await asyncio.sleep(1.0)

    async def _do_idle(self):
        """Idle state - wait and occasionally look around"""
        await asyncio.sleep(1.0)

        # If no goal, set default explore goal
        # In Phase 1, always explore - don't wait for LLM
        if self.context.current_goal is None:
            goal_desc = "Explore and learn survival rules" if config.PHASE1_PURE_SURVIVAL else "Explore the environment"
            self.set_goal(Goal(
                description=goal_desc,
                goal_type="explore"
            ))

        # Transition to the state matching the current goal
        if self.context.current_goal is not None:
            await self._change_state(self._goal_to_state(self.context.current_goal))

    async def _do_explore(self):
        """Exploration state - wander and discover"""
        # Run wander behavior for a bit
        behavior = self.behaviors.wander(duration=10.0)
        self._current_behavior = behavior

        result = await behavior.run()
        self._current_behavior = None
        self.context.last_behavior_result = result

        if result.status == BehaviorStatus.INTERRUPTED:
            if "Cliff" in result.message:
                self.context.stuck_count += 1
                if self.context.stuck_count >= self.max_stuck_attempts:
                    await self._change_state(RobotState.STUCK)
            elif "picked up" in result.message.lower():
                await self._change_state(RobotState.IDLE)
        else:
            self.context.stuck_count = 0
            self.context.exploration_time += result.data.get('duration', 0) if result.data else 0

    async def _do_investigate(self):
        """Investigation state - look closely at something"""
        behavior = self.behaviors.look_around(capture_images=True)
        self._current_behavior = behavior

        result = await behavior.run()
        self._current_behavior = None
        self.context.last_behavior_result = result

        # Store captured images in memory if available
        if result.data and 'images' in result.data and self.memory:
            for img_data in result.data['images']:
                # Would store in memory here
                pass

        # Goal complete, go back to exploring
        if self.context.current_goal and self.context.current_goal.goal_type == "investigate":
            if self._on_goal_complete:
                self._on_goal_complete(self.context.current_goal, True)
            self.context.current_goal = None

        await self._change_state(RobotState.IDLE)

    async def _do_go_to(self):
        """Navigation state - go to a specific point"""
        goal = self.context.current_goal

        if not goal or 'target' not in goal.parameters:
            logger.warning("Go-to state with no target")
            await self._change_state(RobotState.IDLE)
            return

        from cozmo_interface.robot import RobotPose
        target = RobotPose(**goal.parameters['target'])

        behavior = self.behaviors.go_to(target)
        self._current_behavior = behavior

        result = await behavior.run()
        self._current_behavior = None
        self.context.last_behavior_result = result

        if result.status == BehaviorStatus.COMPLETED:
            if self._on_goal_complete:
                self._on_goal_complete(goal, True)
            self.context.current_goal = None
            await self._change_state(RobotState.IDLE)
        elif result.status == BehaviorStatus.FAILED:
            self.context.stuck_count += 1
            if self.context.stuck_count >= self.max_stuck_attempts:
                await self._change_state(RobotState.STUCK)

    async def _do_stuck(self):
        """Stuck state - try to recover"""
        logger.warning("Robot is stuck, attempting recovery")

        # Try backing up and turning
        behavior = self.behaviors.avoid_obstacle()
        result = await behavior.run()

        # Reset stuck count and try again
        self.context.stuck_count = 0
        await self._change_state(RobotState.EXPLORING)

    async def _do_charging(self):
        """Charging state - wait for battery"""
        await asyncio.sleep(5.0)
        logger.info(f"Charging... Battery: {self.robot.sensors.battery_voltage}V")
