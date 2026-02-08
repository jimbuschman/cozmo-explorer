"""
Learning Coordinator

Orchestrates LLM analysis of experiences and validation of proposed rules.
This is the central component that ties together experience logging,
pattern analysis, and rule management.
"""
import asyncio
import logging
import json
import re
from typing import Optional, List, Dict, Any
from datetime import datetime, timedelta
from dataclasses import dataclass

from llm.prompts import LEARNING_ANALYSIS_PROMPT
from memory.experience_logger import ExperienceLogger
from memory.pattern_analyzer import PatternAnalyzer, AnalysisReport
from memory.learned_rules import LearnedRulesStore, LearnedRule, create_rule_from_proposal

logger = logging.getLogger(__name__)


@dataclass
class ValidationResult:
    """Result of validating a proposed rule"""
    rule_id: int
    passed: bool
    success_rate: float
    sample_size: int
    baseline_rate: float
    improvement: float
    details: str


class LearningCoordinator:
    """
    Coordinates the learning cycle:
    1. Gather experience data
    2. Run pattern analysis
    3. Query LLM for rule proposals
    4. Safety check proposals
    5. Validate rules through testing
    6. Activate validated rules
    """

    # Configuration
    MIN_SAMPLES_FOR_ANALYSIS = 20      # Minimum actions before running analysis
    MIN_SAMPLES_FOR_VALIDATION = 10    # Minimum tests before judging a rule
    VALIDATION_TIMEOUT_HOURS = 24      # Auto-reject if not validated in time
    ANALYSIS_COOLDOWN_MINUTES = 30     # Minimum time between analyses

    def __init__(
        self,
        llm_client,
        experience_logger: ExperienceLogger,
        pattern_analyzer: PatternAnalyzer,
        rules_store: LearnedRulesStore
    ):
        self.llm = llm_client
        self.experience_logger = experience_logger
        self.pattern_analyzer = pattern_analyzer
        self.rules_store = rules_store

        self._last_analysis_time: Optional[datetime] = None
        self._testing_rules: Dict[int, Dict] = {}  # rule_id -> test stats

    async def run_analysis_cycle(self) -> List[LearnedRule]:
        """
        Run a complete analysis cycle.

        Returns list of newly proposed rules.
        """
        logger.info("Starting learning analysis cycle")

        # Check if we have enough data
        stats = self.experience_logger.get_recovery_statistics("escape_stall")
        if stats.get('total', 0) < self.MIN_SAMPLES_FOR_ANALYSIS:
            logger.info(f"Not enough data for analysis ({stats.get('total', 0)} < {self.MIN_SAMPLES_FOR_ANALYSIS})")
            return []

        # Generate analysis report
        report = self.pattern_analyzer.generate_analysis_report()
        report_text = self.pattern_analyzer.format_report_for_llm(report)

        # Query LLM for proposals
        proposals = await self._query_llm_for_proposals(report_text)

        if not proposals:
            logger.info("No proposals from LLM")
            return []

        # Process each proposal
        new_rules = []
        for proposal in proposals:
            rule = await self._process_proposal(proposal)
            if rule:
                new_rules.append(rule)

        self._last_analysis_time = datetime.now()
        logger.info(f"Analysis cycle complete: {len(new_rules)} new rules proposed")

        return new_rules

    async def _query_llm_for_proposals(self, analysis_report: str) -> List[Dict]:
        """Query the LLM for rule proposals based on analysis"""
        if not self.llm:
            logger.warning("No LLM client available")
            return self._statistical_proposals(analysis_report)

        try:
            prompt = LEARNING_ANALYSIS_PROMPT.format(analysis_report=analysis_report)
            response = await self.llm._chat(prompt)

            proposals = self._parse_llm_response(response)
            logger.info(f"LLM proposed {len(proposals)} rules")
            return proposals

        except Exception as e:
            logger.error(f"LLM query failed: {e}")
            return self._statistical_proposals(analysis_report)

    def _parse_llm_response(self, response: str) -> List[Dict]:
        """Parse LLM response to extract rule proposals"""
        # Try to find JSON in the response
        try:
            # Look for JSON block
            patterns = [
                r'```json\s*(.*?)\s*```',
                r'```\s*(.*?)\s*```',
                r'(\{[^{}]*"proposals"[^{}]*\[.*?\].*?\})',
            ]

            for pattern in patterns:
                matches = re.findall(pattern, response, re.DOTALL)
                for match in matches:
                    try:
                        data = json.loads(match.strip())
                        if isinstance(data, dict) and 'proposals' in data:
                            return data['proposals']
                    except json.JSONDecodeError:
                        continue

            # Try parsing whole response as JSON
            data = json.loads(response.strip())
            if isinstance(data, dict) and 'proposals' in data:
                return data['proposals']
            elif isinstance(data, list):
                return data

        except json.JSONDecodeError:
            pass

        logger.warning("Could not parse JSON from LLM response")
        return []

    def _statistical_proposals(self, report_text: str) -> List[Dict]:
        """
        Generate proposals from statistical analysis alone (no LLM fallback).

        This provides basic learning even without LLM access.
        """
        proposals = []

        # Get raw stats
        stall_stats = self.experience_logger.get_recovery_statistics("escape_stall")

        if not stall_stats.get('by_angle'):
            return proposals

        # Find best and worst angles
        by_angle = stall_stats['by_angle']
        angle_success = []

        for angle, stats in by_angle.items():
            if angle == 'unknown' or stats['total'] < 3:
                continue
            rate = stats['success'] / stats['total'] if stats['total'] > 0 else 0
            angle_success.append((angle, rate, stats['total']))

        if len(angle_success) < 2:
            return proposals

        # Sort by success rate
        angle_success.sort(key=lambda x: x[1], reverse=True)

        # If there's a clear winner, propose a rule
        best = angle_success[0]
        worst = angle_success[-1]

        if best[1] - worst[1] > 0.2 and best[2] >= 5:  # 20% improvement, 5+ samples
            # Get collision context from precursors
            precursors = self.experience_logger.get_collision_precursors()

            # Determine if there's a directional pattern
            left_closer_angles = []
            right_closer_angles = []

            pairs = self.experience_logger.get_action_outcome_pairs(action_type="escape_stall", limit=100)
            for pair in pairs:
                ctx = pair.get('context', {})
                left = ctx.get('ultra_left') or 999
                right = ctx.get('ultra_right') or 999
                params = pair.get('parameters') or {}
                angle = params.get('angle', 0)
                outcome = pair.get('outcome_type')

                if left < right - 30 and outcome == 'success':
                    right_closer_angles.append(angle)  # Turn right when left is closer
                elif right < left - 30 and outcome == 'success':
                    left_closer_angles.append(angle)

            # Propose directional rules if patterns exist
            if len(left_closer_angles) >= 3:
                best_left = max(set(left_closer_angles), key=left_closer_angles.count)
                proposals.append({
                    "name": "turn_away_left_obstacle",
                    "description": "When obstacle closer on left, prefer turning right",
                    "conditions": [
                        {"sensor": "ext_ultra_l_mm", "op": "<", "value": 200},
                        {"sensor": "ext_ultra_r_mm", "op": ">", "value": 250}
                    ],
                    "action_modifier": {
                        "turn_angle_preference": [a for a in [-120, -90, -60] if a < 0]
                    },
                    "evidence": f"Statistical: right turns work better when left obstacle detected (n={len(left_closer_angles)})"
                })

            if len(right_closer_angles) >= 3:
                proposals.append({
                    "name": "turn_away_right_obstacle",
                    "description": "When obstacle closer on right, prefer turning left",
                    "conditions": [
                        {"sensor": "ext_ultra_r_mm", "op": "<", "value": 200},
                        {"sensor": "ext_ultra_l_mm", "op": ">", "value": 250}
                    ],
                    "action_modifier": {
                        "turn_angle_preference": [a for a in [120, 90, 60] if a > 0]
                    },
                    "evidence": f"Statistical: left turns work better when right obstacle detected (n={len(right_closer_angles)})"
                })

        return proposals

    async def _process_proposal(self, proposal: Dict) -> Optional[LearnedRule]:
        """Process a single proposal: validate safety and store"""
        try:
            rule = create_rule_from_proposal(proposal)

            # Check for duplicate by name
            existing = self.rules_store.get_all_rules()
            for existing_rule in existing:
                if existing_rule.name == rule.name:
                    logger.info(f"Skipping duplicate rule: '{rule.name}' (already exists as id={existing_rule.id})")
                    return None

            # Safety check
            is_safe, reason = self.rules_store.is_rule_safe(rule)
            if not is_safe:
                logger.warning(f"Rule '{rule.name}' failed safety check: {reason}")
                return None

            # Check for conflicts
            conflicts = self.rules_store.check_conflicting_rules(rule)
            if conflicts:
                conflict_names = [c.name for c in conflicts]
                logger.warning(f"Rule '{rule.name}' conflicts with: {conflict_names}")
                # Could still add with lower priority, or reject
                rule.description += f" (Note: may conflict with {conflict_names})"

            # Store the rule
            rule_id = self.rules_store.add_rule(rule)
            rule.id = rule_id

            logger.info(f"Stored proposed rule: {rule.name} (id={rule_id})")
            return rule

        except Exception as e:
            logger.error(f"Failed to process proposal: {e}")
            return None

    def _get_baseline_success_rate(self) -> float:
        """Get baseline success rate from recent actions without rules"""
        stats = self.experience_logger.get_recovery_statistics("escape_stall")
        return stats.get('success_rate', 0.5)

    async def validate_proposal(self, rule_id: int) -> Optional[ValidationResult]:
        """
        Validate a proposed rule based on test results from the DB.

        Uses times_applied/times_successful from the database (written by
        behaviors.py record_rule_application) so stats survive restarts.
        """
        rule = self.rules_store.get_rule(rule_id)
        if not rule:
            return None

        tests = rule.times_applied
        successes = rule.times_successful

        # Check if we have enough samples
        if tests < self.MIN_SAMPLES_FOR_VALIDATION:
            # Check for timeout based on proposed_at
            if rule.proposed_at:
                elapsed = datetime.now() - rule.proposed_at
                if elapsed > timedelta(hours=self.VALIDATION_TIMEOUT_HOURS):
                    logger.warning(f"Rule {rule_id} timed out during validation")
                    self.rules_store.update_rule_status(rule_id, "rejected", {
                        'reason': 'timeout',
                        'tests': tests
                    })
                    return ValidationResult(
                        rule_id=rule_id,
                        passed=False,
                        success_rate=0,
                        sample_size=tests,
                        baseline_rate=self._get_baseline_success_rate(),
                        improvement=0,
                        details="Timed out before enough tests"
                    )
            return None  # Still collecting data

        # Calculate results
        success_rate = successes / tests
        baseline = self._get_baseline_success_rate()
        improvement = success_rate - baseline

        # Validation criteria
        passed = (
            improvement > 0.1 or  # 10% improvement over baseline
            (improvement >= 0 and success_rate > 0.7)  # Or 70%+ success with no regression
        )

        result = ValidationResult(
            rule_id=rule_id,
            passed=passed,
            success_rate=success_rate,
            sample_size=tests,
            baseline_rate=baseline,
            improvement=improvement,
            details=f"Success rate: {success_rate:.1%}, baseline: {baseline:.1%}, improvement: {improvement:+.1%}"
        )

        # Update rule status
        if passed:
            self.rules_store.update_rule_status(rule_id, "validated", {
                'success_rate': success_rate,
                'improvement': improvement,
                'sample_size': tests
            })
            logger.info(f"Rule {rule_id} VALIDATED: {result.details}")
        else:
            self.rules_store.update_rule_status(rule_id, "rejected", {
                'success_rate': success_rate,
                'improvement': improvement,
                'sample_size': tests,
                'reason': 'insufficient_improvement'
            })
            logger.info(f"Rule {rule_id} REJECTED: {result.details}")

        return result

    def record_test_result(self, rule_id: int, success: bool):
        """Record a test result for a rule being validated.

        Note: This is now handled by rules_store.record_rule_application()
        called directly from behaviors.py. Kept for backward compatibility.
        """
        self.rules_store.record_rule_application(rule_id, success)

    def activate_validated_rules(self) -> List[int]:
        """Activate all validated rules"""
        validated = self.rules_store.get_rules_by_status("validated")
        activated = []

        for rule in validated:
            self.rules_store.update_rule_status(rule.id, "active")
            activated.append(rule.id)
            logger.info(f"Activated rule: {rule.name} (id={rule.id})")

        return activated

    def should_run_analysis(self) -> bool:
        """Check if we should run an analysis cycle"""
        # Check cooldown
        if self._last_analysis_time:
            elapsed = datetime.now() - self._last_analysis_time
            if elapsed < timedelta(minutes=self.ANALYSIS_COOLDOWN_MINUTES):
                return False

        # Check if we have enough new data
        stats = self.experience_logger.get_recovery_statistics("escape_stall")
        return stats.get('total', 0) >= self.MIN_SAMPLES_FOR_ANALYSIS

    def get_testing_rules(self) -> List[Dict]:
        """Get info about rules currently being tested (from DB)"""
        testing_rules = self.rules_store.get_rules_by_status("testing")
        result = []
        for rule in testing_rules:
            result.append({
                'rule_id': rule.id,
                'name': rule.name,
                'tests': rule.times_applied,
                'successes': rule.times_successful,
                'success_rate': rule.success_rate,
                'proposed_at': rule.proposed_at.isoformat() if rule.proposed_at else None
            })
        return result

    def get_status_summary(self) -> Dict[str, Any]:
        """Get a summary of the learning system status"""
        all_rules = self.rules_store.get_all_rules()

        by_status = {}
        for rule in all_rules:
            status = rule.status
            if status not in by_status:
                by_status[status] = 0
            by_status[status] += 1

        return {
            'total_rules': len(all_rules),
            'by_status': by_status,
            'testing': by_status.get('testing', 0),
            'last_analysis': self._last_analysis_time.isoformat() if self._last_analysis_time else None,
            'data_samples': self.experience_logger.get_recovery_statistics("escape_stall").get('total', 0)
        }
