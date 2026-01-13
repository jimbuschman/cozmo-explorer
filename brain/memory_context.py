"""
Memory Context Builder

Gathers context from all memory systems to provide rich context for LLM decisions.
Connects: VisionObserver, ChromaDB experiences, SpatialMap, and decision history.
"""
import logging
from typing import Optional, List, Dict, Any
from dataclasses import dataclass, field
from datetime import datetime
from collections import deque

logger = logging.getLogger(__name__)


@dataclass
class Decision:
    """Record of a decision made by the LLM"""
    timestamp: datetime
    action: str
    target: Optional[str]
    reasoning: str
    outcome: Optional[str] = None  # filled in after execution
    success: Optional[bool] = None


@dataclass
class ContextSnapshot:
    """A snapshot of all relevant context for decision-making"""
    # Current state
    position: Dict[str, float]
    battery_voltage: float
    is_on_charger: bool
    current_state: str
    exploration_time: float
    stuck_count: int

    # What I see now
    current_observation: Optional[str] = None

    # Recent history
    recent_observations: List[str] = field(default_factory=list)
    recent_decisions: List[Decision] = field(default_factory=list)

    # Memory recall
    relevant_memories: List[Dict] = field(default_factory=list)

    # Spatial awareness
    exploration_percentage: float = 0.0
    visited_percentage: float = 0.0
    nearby_points_of_interest: List[str] = field(default_factory=list)
    unexplored_directions: List[str] = field(default_factory=list)


class MemoryContext:
    """
    Builds rich context for LLM decision-making by aggregating:
    - Current robot state
    - Recent visual observations
    - Relevant past experiences from ChromaDB
    - Spatial map awareness
    - Recent decision history
    """

    def __init__(
        self,
        experience_db=None,
        spatial_map=None,
        vision_observer=None,
        max_decision_history: int = 20
    ):
        self.experience_db = experience_db
        self.spatial_map = spatial_map
        self.vision_observer = vision_observer

        # Track recent decisions
        self.decision_history: deque = deque(maxlen=max_decision_history)

        # Current observation (set by vision observer callback)
        self.current_observation: Optional[str] = None

    def record_decision(self, action: str, target: str = None, reasoning: str = ""):
        """Record a decision that was made"""
        decision = Decision(
            timestamp=datetime.now(),
            action=action,
            target=target,
            reasoning=reasoning
        )
        self.decision_history.append(decision)
        logger.debug(f"Recorded decision: {action}")
        return decision

    def update_last_decision_outcome(self, outcome: str, success: bool):
        """Update the outcome of the most recent decision"""
        if self.decision_history:
            last = self.decision_history[-1]
            last.outcome = outcome
            last.success = success

    def set_current_observation(self, observation: str):
        """Set what the robot currently sees (called by vision observer)"""
        self.current_observation = observation

    async def build_context(
        self,
        robot,
        current_state: str,
        exploration_time: float,
        stuck_count: int
    ) -> ContextSnapshot:
        """
        Build a complete context snapshot for LLM decision-making.

        Args:
            robot: CozmoRobot instance
            current_state: Current state machine state name
            exploration_time: Total exploration time in seconds
            stuck_count: Number of times robot has been stuck

        Returns:
            ContextSnapshot with all relevant context
        """
        # Basic state
        context = ContextSnapshot(
            position={
                "x": robot.pose.x,
                "y": robot.pose.y,
                "angle_degrees": robot.pose.angle * 57.3
            },
            battery_voltage=robot.sensors.battery_voltage,
            is_on_charger=robot.sensors.is_on_charger,
            current_state=current_state,
            exploration_time=exploration_time,
            stuck_count=stuck_count
        )

        # Current observation
        context.current_observation = self.current_observation

        # Recent observations from vision observer
        if self.vision_observer:
            context.recent_observations = self.vision_observer.get_recent_descriptions(5)

        # Recent decisions
        context.recent_decisions = list(self.decision_history)[-5:]

        # Query relevant memories from ChromaDB
        if self.experience_db and self.current_observation:
            context.relevant_memories = await self._get_relevant_memories(
                self.current_observation,
                robot.pose.x,
                robot.pose.y
            )

        # Spatial awareness
        if self.spatial_map:
            context.exploration_percentage = self.spatial_map.get_exploration_progress() * 100
            context.visited_percentage = self.spatial_map.get_visited_percentage() * 100
            context.nearby_points_of_interest = self._get_nearby_poi_descriptions(
                robot.pose.x, robot.pose.y
            )
            context.unexplored_directions = self._get_unexplored_directions(
                robot.pose.x, robot.pose.y
            )

        return context

    async def _get_relevant_memories(
        self,
        current_observation: str,
        x: float,
        y: float,
        n_semantic: int = 3,
        n_spatial: int = 3
    ) -> List[Dict]:
        """
        Get relevant memories based on:
        1. Semantic similarity to current observation
        2. Spatial proximity to current location
        """
        memories = []

        try:
            # Semantic search - "have I seen something like this?"
            similar = await self.experience_db.find_similar(
                current_observation,
                n_results=n_semantic
            )
            for exp in similar:
                memories.append({
                    "type": "semantic_match",
                    "description": exp.description,
                    "location": f"({exp.location_x:.0f}, {exp.location_y:.0f})",
                    "when": self._time_ago(exp.timestamp),
                    "experience_type": exp.experience_type
                })

            # Spatial search - "what have I seen nearby?"
            nearby = await self.experience_db.find_near_location(
                x, y, radius=300.0, n_results=n_spatial
            )
            for exp in nearby:
                # Avoid duplicates
                if not any(m["description"] == exp.description for m in memories):
                    memories.append({
                        "type": "nearby",
                        "description": exp.description,
                        "location": f"({exp.location_x:.0f}, {exp.location_y:.0f})",
                        "when": self._time_ago(exp.timestamp),
                        "experience_type": exp.experience_type
                    })

        except Exception as e:
            logger.error(f"Failed to get relevant memories: {e}")

        return memories

    def _get_nearby_poi_descriptions(self, x: float, y: float) -> List[str]:
        """Get descriptions of nearby points of interest"""
        if not self.spatial_map:
            return []

        points = self.spatial_map.get_points_near(x, y, radius=500.0)
        return [f"{p.label} ({p.point_type}) at ({p.x:.0f}, {p.y:.0f})" for p in points]

    def _get_unexplored_directions(self, x: float, y: float) -> List[str]:
        """Determine which directions have unexplored areas"""
        if not self.spatial_map:
            return []

        directions = []
        check_distance = 200.0  # mm

        # Check each cardinal direction
        checks = [
            ("North", x, y + check_distance),
            ("South", x, y - check_distance),
            ("East", x + check_distance, y),
            ("West", x - check_distance, y),
        ]

        for name, cx, cy in checks:
            if self.spatial_map.is_unknown(cx, cy):
                directions.append(name)

        return directions

    def _time_ago(self, timestamp: datetime) -> str:
        """Convert timestamp to human-readable 'X ago' format"""
        if not timestamp:
            return "unknown"

        delta = datetime.now() - timestamp
        seconds = delta.total_seconds()

        if seconds < 60:
            return f"{int(seconds)}s ago"
        elif seconds < 3600:
            return f"{int(seconds/60)}m ago"
        else:
            return f"{int(seconds/3600)}h ago"

    def format_for_llm(self, context: ContextSnapshot) -> str:
        """
        Format the context snapshot into a prompt string for the LLM.
        """
        lines = []

        # Current state
        lines.append("=== CURRENT STATE ===")
        lines.append(f"Position: ({context.position['x']:.0f}, {context.position['y']:.0f}) facing {context.position['angle_degrees']:.0f}°")

        battery_status = "charging" if context.is_on_charger else self._battery_level(context.battery_voltage)
        lines.append(f"Battery: {context.battery_voltage:.2f}V ({battery_status})")
        lines.append(f"State: {context.current_state}")
        lines.append(f"Time exploring: {context.exploration_time:.0f}s")
        if context.stuck_count > 0:
            lines.append(f"Times stuck: {context.stuck_count}")

        # What I see now
        lines.append("")
        lines.append("=== WHAT I SEE NOW ===")
        if context.current_observation:
            lines.append(context.current_observation)
        else:
            lines.append("(No current observation)")

        # Recent observations
        if context.recent_observations:
            lines.append("")
            lines.append("=== RECENT OBSERVATIONS ===")
            for i, obs in enumerate(context.recent_observations):
                lines.append(f"- {obs[:100]}{'...' if len(obs) > 100 else ''}")

        # Relevant memories
        if context.relevant_memories:
            lines.append("")
            lines.append("=== RELEVANT MEMORIES ===")
            for mem in context.relevant_memories:
                match_type = "Similar to now" if mem["type"] == "semantic_match" else "Nearby"
                lines.append(f"- [{match_type}] {mem['when']} at {mem['location']}: {mem['description'][:80]}...")

        # Spatial awareness
        lines.append("")
        lines.append("=== SPATIAL AWARENESS ===")
        lines.append(f"Explored: {context.exploration_percentage:.1f}% of area")
        lines.append(f"Actually visited: {context.visited_percentage:.1f}%")

        if context.unexplored_directions:
            lines.append(f"Unexplored directions: {', '.join(context.unexplored_directions)}")
        else:
            lines.append("All nearby directions explored")

        if context.nearby_points_of_interest:
            lines.append("Nearby points of interest:")
            for poi in context.nearby_points_of_interest:
                lines.append(f"  - {poi}")

        # Recent decisions
        if context.recent_decisions:
            lines.append("")
            lines.append("=== RECENT DECISIONS ===")
            for dec in context.recent_decisions[-3:]:
                outcome_str = ""
                if dec.outcome:
                    outcome_str = f" → {dec.outcome}"
                    if dec.success is not None:
                        outcome_str += f" ({'success' if dec.success else 'failed'})"
                lines.append(f"- {self._time_ago(dec.timestamp)}: {dec.action}")
                if dec.target:
                    lines.append(f"    Target: {dec.target}")
                if dec.reasoning:
                    lines.append(f"    Reason: {dec.reasoning[:60]}...")
                if outcome_str:
                    lines.append(f"    Outcome: {outcome_str}")

        lines.append("")
        lines.append("=== WHAT SHOULD I DO NEXT? ===")
        lines.append("Respond with a JSON object containing: action, target (optional), reasoning, speak (optional)")

        return "\n".join(lines)

    def _battery_level(self, voltage: float) -> str:
        """Convert voltage to human-readable level"""
        if voltage <= 0.1:
            return "unknown"  # Not yet received from robot
        elif voltage >= 4.5:
            return "full"
        elif voltage >= 4.0:
            return "good"
        elif voltage >= 3.7:
            return "medium"
        elif voltage >= 3.5:
            return "low"
        else:
            return "critical"
