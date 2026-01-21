"""
Learned Rules Store

Stores and applies validated behavioral rules learned from experience.
Rules are proposed by the LLM, validated through testing, and then activated.
"""
import sqlite3
import logging
import json
from typing import Optional, List, Dict, Any
from datetime import datetime
from pathlib import Path
from dataclasses import dataclass, asdict
from enum import Enum

import config

logger = logging.getLogger(__name__)


class RuleStatus(Enum):
    """Status of a learned rule"""
    PROPOSED = "proposed"      # Just created, awaiting validation
    TESTING = "testing"        # Currently being tested
    VALIDATED = "validated"    # Passed validation, ready to activate
    REJECTED = "rejected"      # Failed validation or safety check
    ACTIVE = "active"          # Currently in use
    DISABLED = "disabled"      # Manually disabled


@dataclass
class RuleCondition:
    """A condition that must be met for a rule to apply"""
    sensor: str           # e.g., "ext_ultra_r_mm", "ext_ultra_l_mm"
    operator: str         # "<", ">", "<=", ">=", "==", "between"
    value: Any            # Single value or [min, max] for "between"

    def evaluate(self, sensor_context: Dict[str, Any]) -> bool:
        """Check if this condition is met given sensor values"""
        actual = sensor_context.get(self.sensor)
        if actual is None:
            return False

        if self.operator == "<":
            return actual < self.value
        elif self.operator == ">":
            return actual > self.value
        elif self.operator == "<=":
            return actual <= self.value
        elif self.operator == ">=":
            return actual >= self.value
        elif self.operator == "==":
            return actual == self.value
        elif self.operator == "between":
            if isinstance(self.value, list) and len(self.value) == 2:
                return self.value[0] <= actual <= self.value[1]
        return False


@dataclass
class LearnedRule:
    """A behavioral rule learned from experience"""
    id: Optional[int] = None
    name: str = ""
    description: str = ""
    conditions: List[Dict] = None  # List of condition dicts
    action_modifier: Dict = None   # How to modify the action
    status: str = "proposed"
    proposed_at: Optional[datetime] = None
    validated_at: Optional[datetime] = None
    evidence_summary: str = ""
    test_results: Optional[Dict] = None
    safety_score: float = 1.0  # 0-1, higher is safer

    # Scope for future location-based rules
    scope: str = "global"  # "global" or "location"
    location_id: Optional[str] = None  # For location-scoped rules (future use)

    # Performance tracking for conflict resolution
    times_applied: int = 0
    times_successful: int = 0

    def __post_init__(self):
        if self.conditions is None:
            self.conditions = []
        if self.action_modifier is None:
            self.action_modifier = {}

    @property
    def specificity(self) -> int:
        """Number of conditions - more specific rules have more conditions"""
        return len(self.conditions)

    @property
    def success_rate(self) -> float:
        """Historical success rate of this rule"""
        if self.times_applied == 0:
            return 0.5  # Neutral default for untested rules
        return self.times_successful / self.times_applied

    def get_conditions(self) -> List[RuleCondition]:
        """Parse conditions into RuleCondition objects"""
        return [
            RuleCondition(
                sensor=c.get('sensor', ''),
                operator=c.get('op', c.get('operator', '==')),
                value=c.get('value')
            )
            for c in self.conditions
        ]

    def matches_context(self, sensor_context: Dict[str, Any]) -> bool:
        """Check if all conditions are met for this rule to apply"""
        conditions = self.get_conditions()
        if not conditions:
            return False
        return all(c.evaluate(sensor_context) for c in conditions)


class LearnedRulesStore:
    """
    SQLite-backed storage for learned behavioral rules.

    Provides:
    - Rule CRUD operations
    - Rule application to actions
    - Status tracking through validation lifecycle
    """

    # Safety constraints
    MIN_TURN_ANGLE = -180
    MAX_TURN_ANGLE = 180
    MIN_SPEED = 10
    MAX_SPEED = 100
    MIN_BACKUP_DURATION = 0.3
    MAX_BACKUP_DURATION = 2.0

    # Protected sensors (rules cannot disable these)
    PROTECTED_SENSORS = {'cliff_detected', 'is_picked_up'}

    def __init__(self, db_path: str = None):
        self.db_path = Path(db_path or config.SQLITE_PATH)
        self._conn: Optional[sqlite3.Connection] = None

    def connect(self):
        """Connect to database and create tables"""
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(str(self.db_path))
        self._conn.row_factory = sqlite3.Row
        self._create_tables()
        logger.info(f"LearnedRulesStore connected: {self.db_path}")

    def _create_tables(self):
        """Create rules table if it doesn't exist"""
        cursor = self._conn.cursor()

        cursor.execute("""
            CREATE TABLE IF NOT EXISTS learned_rules (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                description TEXT,
                conditions TEXT,
                action_modifier TEXT,
                status TEXT DEFAULT 'proposed',
                proposed_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                validated_at TIMESTAMP,
                evidence_summary TEXT,
                test_results TEXT,
                safety_score REAL DEFAULT 1.0,
                scope TEXT DEFAULT 'global',
                location_id TEXT,
                times_applied INTEGER DEFAULT 0,
                times_successful INTEGER DEFAULT 0
            )
        """)

        # Index for active rules lookup
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_learned_rules_status
            ON learned_rules(status)
        """)

        self._conn.commit()

    def close(self):
        """Close database connection"""
        if self._conn:
            self._conn.close()
            self._conn = None

    # ==================== Rule CRUD ====================

    def add_rule(self, rule: LearnedRule) -> int:
        """Add a new rule and return its ID"""
        if not self._conn:
            return -1

        cursor = self._conn.cursor()
        cursor.execute("""
            INSERT INTO learned_rules
            (name, description, conditions, action_modifier, status,
             evidence_summary, test_results, safety_score,
             scope, location_id, times_applied, times_successful)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            rule.name,
            rule.description,
            json.dumps(rule.conditions),
            json.dumps(rule.action_modifier),
            rule.status,
            rule.evidence_summary,
            json.dumps(rule.test_results) if rule.test_results else None,
            rule.safety_score,
            rule.scope,
            rule.location_id,
            rule.times_applied,
            rule.times_successful
        ))
        self._conn.commit()

        rule_id = cursor.lastrowid
        logger.info(f"Added rule {rule_id}: {rule.name} (scope={rule.scope})")
        return rule_id

    def get_rule(self, rule_id: int) -> Optional[LearnedRule]:
        """Get a rule by ID"""
        if not self._conn:
            return None

        cursor = self._conn.cursor()
        cursor.execute("SELECT * FROM learned_rules WHERE id = ?", (rule_id,))
        row = cursor.fetchone()

        if row:
            return self._row_to_rule(row)
        return None

    def get_rules_by_status(self, status: str) -> List[LearnedRule]:
        """Get all rules with a given status"""
        if not self._conn:
            return []

        cursor = self._conn.cursor()
        cursor.execute(
            "SELECT * FROM learned_rules WHERE status = ? ORDER BY proposed_at DESC",
            (status,)
        )

        return [self._row_to_rule(row) for row in cursor.fetchall()]

    def get_active_rules(self) -> List[LearnedRule]:
        """Get all active rules"""
        return self.get_rules_by_status("active")

    def get_all_rules(self) -> List[LearnedRule]:
        """Get all rules regardless of status"""
        if not self._conn:
            return []

        cursor = self._conn.cursor()
        cursor.execute("SELECT * FROM learned_rules ORDER BY proposed_at DESC")
        return [self._row_to_rule(row) for row in cursor.fetchall()]

    def update_rule_status(
        self,
        rule_id: int,
        status: str,
        test_results: Dict = None
    ):
        """Update a rule's status"""
        if not self._conn:
            return

        cursor = self._conn.cursor()

        if status == "validated" or status == "active":
            cursor.execute("""
                UPDATE learned_rules
                SET status = ?, validated_at = CURRENT_TIMESTAMP, test_results = ?
                WHERE id = ?
            """, (status, json.dumps(test_results) if test_results else None, rule_id))
        else:
            cursor.execute("""
                UPDATE learned_rules
                SET status = ?, test_results = ?
                WHERE id = ?
            """, (status, json.dumps(test_results) if test_results else None, rule_id))

        self._conn.commit()
        logger.info(f"Rule {rule_id} status -> {status}")

    def delete_rule(self, rule_id: int):
        """Delete a rule"""
        if not self._conn:
            return

        cursor = self._conn.cursor()
        cursor.execute("DELETE FROM learned_rules WHERE id = ?", (rule_id,))
        self._conn.commit()
        logger.info(f"Deleted rule {rule_id}")

    def _row_to_rule(self, row) -> LearnedRule:
        """Convert a database row to a LearnedRule"""
        return LearnedRule(
            id=row['id'],
            name=row['name'],
            description=row['description'],
            conditions=json.loads(row['conditions']) if row['conditions'] else [],
            action_modifier=json.loads(row['action_modifier']) if row['action_modifier'] else {},
            status=row['status'],
            proposed_at=datetime.fromisoformat(row['proposed_at']) if row['proposed_at'] else None,
            validated_at=datetime.fromisoformat(row['validated_at']) if row['validated_at'] else None,
            evidence_summary=row['evidence_summary'],
            test_results=json.loads(row['test_results']) if row['test_results'] else None,
            safety_score=row['safety_score'],
            scope=row['scope'] if 'scope' in row.keys() else 'global',
            location_id=row['location_id'] if 'location_id' in row.keys() else None,
            times_applied=row['times_applied'] if 'times_applied' in row.keys() else 0,
            times_successful=row['times_successful'] if 'times_successful' in row.keys() else 0
        )

    # ==================== Rule Application ====================

    def apply_rules_to_action(
        self,
        base_action: Dict[str, Any],
        sensor_context: Dict[str, Any],
        location_id: Optional[str] = None
    ) -> tuple[Dict[str, Any], Optional[int]]:
        """
        Apply active rules to modify a base action.

        Uses conflict resolution: most specific matching rule wins.
        Specificity = number of conditions. Ties broken by success_rate.

        Args:
            base_action: The original action parameters
                e.g., {"angles": [-120, -90, 90, 120], "backup_duration": 0.7}
            sensor_context: Current sensor values
                e.g., {"ext_ultra_l_mm": 100, "ext_ultra_r_mm": 300, ...}
            location_id: Optional current location for location-scoped rules (future)

        Returns:
            Tuple of (modified action parameters, rule_id that was applied or None)
        """
        active_rules = self.get_active_rules()
        if not active_rules:
            return base_action, None

        # Filter to matching rules
        matching_rules = []
        for rule in active_rules:
            # Check scope - global rules always apply, location rules only in their location
            if rule.scope == "location" and rule.location_id != location_id:
                continue
            if rule.matches_context(sensor_context):
                matching_rules.append(rule)

        if not matching_rules:
            return base_action, None

        # Conflict resolution: most specific wins, then highest success rate
        # Location-scoped rules get priority over global rules (when implemented)
        best_rule = max(
            matching_rules,
            key=lambda r: (
                r.scope == "location",  # Location rules beat global (future)
                r.specificity,           # More conditions = more specific
                r.success_rate           # Higher success rate breaks ties
            )
        )

        logger.debug(f"Selected rule '{best_rule.name}' (specificity={best_rule.specificity}, "
                     f"success_rate={best_rule.success_rate:.1%}) from {len(matching_rules)} matching rules")

        modified = self._apply_modifier(base_action.copy(), best_rule.action_modifier)
        return modified, best_rule.id

    def record_rule_application(self, rule_id: int, success: bool):
        """
        Record that a rule was applied and whether it succeeded.

        Call this after an action completes to track rule performance.
        """
        if not self._conn:
            return

        cursor = self._conn.cursor()
        if success:
            cursor.execute("""
                UPDATE learned_rules
                SET times_applied = times_applied + 1,
                    times_successful = times_successful + 1
                WHERE id = ?
            """, (rule_id,))
        else:
            cursor.execute("""
                UPDATE learned_rules
                SET times_applied = times_applied + 1
                WHERE id = ?
            """, (rule_id,))
        self._conn.commit()

    def _apply_modifier(
        self,
        action: Dict[str, Any],
        modifier: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Apply a single modifier to an action"""
        result = action.copy()

        # Turn angle preference
        if 'turn_angle_preference' in modifier:
            pref = modifier['turn_angle_preference']
            if isinstance(pref, list) and pref:
                # Replace or filter angles
                result['angles'] = pref

        # Turn angle filter (remove certain angles)
        if 'remove_angles' in modifier:
            remove = set(modifier['remove_angles'])
            if 'angles' in result:
                result['angles'] = [a for a in result['angles'] if a not in remove]
                # Ensure at least one angle remains
                if not result['angles']:
                    result['angles'] = action.get('angles', [-90, 90])

        # Backup duration modifier
        if 'backup_duration' in modifier:
            result['backup_duration'] = max(
                self.MIN_BACKUP_DURATION,
                min(self.MAX_BACKUP_DURATION, modifier['backup_duration'])
            )

        # Speed modifier
        if 'speed_multiplier' in modifier:
            if 'speed' in result:
                result['speed'] = max(
                    self.MIN_SPEED,
                    min(self.MAX_SPEED, result['speed'] * modifier['speed_multiplier'])
                )

        return result

    # ==================== Safety Checks ====================

    def is_rule_safe(self, rule: LearnedRule) -> tuple[bool, str]:
        """
        Check if a rule passes safety requirements.

        Returns (is_safe, reason)
        """
        # Check for protected sensor disabling
        for cond in rule.conditions:
            sensor = cond.get('sensor', '')
            if sensor in self.PROTECTED_SENSORS:
                # Rules cannot disable safety sensors
                op = cond.get('op', cond.get('operator', ''))
                if op in ('==', '!=') and cond.get('value') in (False, 0, 'false'):
                    return False, f"Cannot disable safety sensor: {sensor}"

        # Check action modifier bounds
        modifier = rule.action_modifier or {}

        # Turn angles must be in valid range
        if 'turn_angle_preference' in modifier:
            angles = modifier['turn_angle_preference']
            if isinstance(angles, list):
                for angle in angles:
                    if not (self.MIN_TURN_ANGLE <= angle <= self.MAX_TURN_ANGLE):
                        return False, f"Turn angle {angle} out of bounds [{self.MIN_TURN_ANGLE}, {self.MAX_TURN_ANGLE}]"

        # Backup duration must be in range
        if 'backup_duration' in modifier:
            duration = modifier['backup_duration']
            if not (self.MIN_BACKUP_DURATION <= duration <= self.MAX_BACKUP_DURATION):
                return False, f"Backup duration {duration} out of bounds [{self.MIN_BACKUP_DURATION}, {self.MAX_BACKUP_DURATION}]"

        # Speed multiplier check
        if 'speed_multiplier' in modifier:
            mult = modifier['speed_multiplier']
            if mult < 0.3 or mult > 2.0:
                return False, f"Speed multiplier {mult} out of bounds [0.3, 2.0]"

        return True, "Passed all safety checks"

    def check_conflicting_rules(self, new_rule: LearnedRule) -> List[LearnedRule]:
        """
        Check if a new rule conflicts with existing active rules.

        Returns list of conflicting rules.
        """
        active_rules = self.get_active_rules()
        conflicts = []

        for existing in active_rules:
            # Check if conditions overlap significantly
            if self._conditions_overlap(new_rule.conditions, existing.conditions):
                # Check if modifiers conflict
                if self._modifiers_conflict(new_rule.action_modifier, existing.action_modifier):
                    conflicts.append(existing)

        return conflicts

    def _conditions_overlap(self, conds1: List[Dict], conds2: List[Dict]) -> bool:
        """Check if two condition sets could both be true simultaneously"""
        # Simplified check: same sensors being tested
        sensors1 = {c.get('sensor') for c in conds1}
        sensors2 = {c.get('sensor') for c in conds2}
        return bool(sensors1 & sensors2)

    def _modifiers_conflict(self, mod1: Dict, mod2: Dict) -> bool:
        """Check if two modifiers would conflict"""
        # Conflict if both try to set the same parameter differently
        for key in mod1:
            if key in mod2 and mod1[key] != mod2[key]:
                return True
        return False

    # ==================== Context Manager ====================

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


# Utility function for creating rules from LLM proposals
def create_rule_from_proposal(proposal: Dict[str, Any]) -> LearnedRule:
    """
    Create a LearnedRule from an LLM proposal dict.

    Expected proposal format:
    {
        "name": "rule_name",
        "description": "What this rule does",
        "conditions": [{"sensor": "...", "op": "<", "value": 150}],
        "action_modifier": {"turn_angle_preference": [-90, -120]},
        "evidence": "Why this rule should work"
    }
    """
    return LearnedRule(
        name=proposal.get('name', 'unnamed_rule'),
        description=proposal.get('description', ''),
        conditions=proposal.get('conditions', []),
        action_modifier=proposal.get('action_modifier', {}),
        status='proposed',
        evidence_summary=proposal.get('evidence', '')
    )
