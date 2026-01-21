"""
Pattern Analyzer

Extracts patterns from logged experiences for learning.
Analyzes sensor data, actions, and outcomes to identify correlations.
"""
import logging
from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass
from collections import defaultdict
from statistics import mean, stdev

from memory.experience_logger import ExperienceLogger

logger = logging.getLogger(__name__)


@dataclass
class RecoveryPattern:
    """Pattern observed in recovery actions"""
    action_type: str
    context_conditions: Dict[str, Any]  # Sensor conditions when this pattern applies
    recommended_angles: List[int]  # Turn angles that worked well
    success_rate: float
    sample_size: int
    evidence: str  # Human-readable summary


@dataclass
class CollisionPrecursor:
    """Sensor pattern that preceded collisions"""
    sensor_name: str
    threshold_value: int
    direction: str  # "below" or "above"
    collision_rate: float
    sample_size: int


@dataclass
class AnalysisReport:
    """Complete analysis report for LLM consumption"""
    session_count: int
    total_actions: int
    recovery_patterns: List[RecoveryPattern]
    collision_precursors: List[CollisionPrecursor]
    sensor_correlations: Dict[str, float]
    recommendations: List[str]


class PatternAnalyzer:
    """
    Analyzes logged experiences to extract patterns.

    Works with ExperienceLogger data to find:
    - Which turn angles work best for recovery
    - Sensor patterns that precede collisions
    - Environmental correlations
    """

    def __init__(self, experience_logger: ExperienceLogger):
        self.logger = experience_logger

    def analyze_recovery_outcomes(self, action_type: str = "escape_stall") -> List[RecoveryPattern]:
        """
        Analyze which turn angles work best for different sensor contexts.

        Returns patterns showing successful recovery strategies.
        """
        # Get action-outcome pairs
        pairs = self.logger.get_action_outcome_pairs(action_type=action_type, limit=500)

        if not pairs:
            return []

        # Group by context (binned sensor values)
        context_groups = defaultdict(list)
        for pair in pairs:
            context = pair.get('context', {})
            # Bin context into categories
            context_key = self._bin_context(context)
            context_groups[context_key].append(pair)

        patterns = []
        for context_key, group in context_groups.items():
            if len(group) < 3:  # Need minimum sample size
                continue

            # Analyze success rates by angle
            angle_outcomes = defaultdict(lambda: {'success': 0, 'total': 0})
            for pair in group:
                params = pair.get('parameters') or {}
                angle = params.get('angle', 0)
                outcome = pair.get('outcome_type', 'unknown')

                angle_outcomes[angle]['total'] += 1
                if outcome == 'success':
                    angle_outcomes[angle]['success'] += 1

            # Find best performing angles
            best_angles = []
            for angle, stats in angle_outcomes.items():
                if stats['total'] >= 2:  # Minimum samples
                    rate = stats['success'] / stats['total']
                    if rate >= 0.6:  # 60%+ success rate
                        best_angles.append((angle, rate, stats['total']))

            if not best_angles:
                continue

            # Sort by success rate
            best_angles.sort(key=lambda x: x[1], reverse=True)
            recommended = [a[0] for a in best_angles[:4]]  # Top 4 angles

            overall_success = sum(1 for p in group if p.get('outcome_type') == 'success')
            overall_rate = overall_success / len(group)

            # Generate evidence summary
            evidence = self._format_recovery_evidence(context_key, best_angles, len(group))

            patterns.append(RecoveryPattern(
                action_type=action_type,
                context_conditions=self._parse_context_key(context_key),
                recommended_angles=recommended,
                success_rate=overall_rate,
                sample_size=len(group),
                evidence=evidence
            ))

        return patterns

    def analyze_collision_precursors(self) -> List[CollisionPrecursor]:
        """
        Identify sensor patterns that preceded collisions.

        Useful for proactive avoidance.
        """
        precursors = self.logger.get_collision_precursors()

        if not precursors:
            return []

        # Analyze sensor distributions before collisions
        sensor_values = defaultdict(list)
        for p in precursors:
            ctx = p.get('context', {})
            for sensor_name in ['ultra_left', 'ultra_center', 'ultra_right', 'tof']:
                value = ctx.get(sensor_name)
                if value is not None and value > 0:
                    sensor_values[sensor_name].append(value)

        patterns = []
        for sensor_name, values in sensor_values.items():
            if len(values) < 5:
                continue

            avg = mean(values)
            # Low distance readings before collision indicate the sensor
            # detected something but we didn't react in time
            if avg < 200:  # mm
                patterns.append(CollisionPrecursor(
                    sensor_name=sensor_name,
                    threshold_value=int(avg * 1.2),  # Add buffer
                    direction="below",
                    collision_rate=0.8,  # This is approximate
                    sample_size=len(values)
                ))

        return patterns

    def analyze_sensor_correlations(self) -> Dict[str, float]:
        """
        Find correlations between sensor readings and outcomes.

        Returns dict mapping sensor conditions to outcome correlations.
        """
        pairs = self.logger.get_action_outcome_pairs(limit=500)

        if len(pairs) < 10:
            return {}

        correlations = {}

        # Analyze front obstacle distance vs success rate
        front_low = [p for p in pairs if (p.get('context', {}).get('ultra_center') or 999) < 150]
        front_high = [p for p in pairs if (p.get('context', {}).get('ultra_center') or 0) >= 150]

        if len(front_low) >= 5 and len(front_high) >= 5:
            low_success = sum(1 for p in front_low if p.get('outcome_type') == 'success') / len(front_low)
            high_success = sum(1 for p in front_high if p.get('outcome_type') == 'success') / len(front_high)
            correlations['front_clear_correlation'] = high_success - low_success

        # Analyze asymmetry (left vs right) vs turn direction
        left_closer = [p for p in pairs if (p.get('context', {}).get('ultra_left') or 999) < (p.get('context', {}).get('ultra_right') or 999)]
        for pair in left_closer:
            params = pair.get('parameters') or {}
            angle = params.get('angle', 0)
            # Positive angle = turn left, negative = turn right
            # If left is closer, turning right (negative) might be better
            pair['_turn_away'] = angle < 0 if angle != 0 else None

        turn_away = [p for p in left_closer if p.get('_turn_away') is True]
        turn_toward = [p for p in left_closer if p.get('_turn_away') is False]

        if len(turn_away) >= 3 and len(turn_toward) >= 3:
            away_success = sum(1 for p in turn_away if p.get('outcome_type') == 'success') / len(turn_away)
            toward_success = sum(1 for p in turn_toward if p.get('outcome_type') == 'success') / len(turn_toward)
            correlations['turn_away_from_obstacle'] = away_success - toward_success

        return correlations

    def generate_analysis_report(self) -> AnalysisReport:
        """
        Generate a complete analysis report for LLM consumption.

        This is the main entry point for the learning coordinator.
        """
        # Gather all analyses
        recovery_patterns = self.analyze_recovery_outcomes("escape_stall")
        recovery_patterns.extend(self.analyze_recovery_outcomes("escape_cliff"))

        collision_precursors = self.analyze_collision_precursors()
        correlations = self.analyze_sensor_correlations()

        # Get basic stats
        stall_stats = self.logger.get_recovery_statistics("escape_stall")
        cliff_stats = self.logger.get_recovery_statistics("escape_cliff")
        total_actions = stall_stats.get('total', 0) + cliff_stats.get('total', 0)

        # Generate recommendations based on patterns
        recommendations = self._generate_recommendations(
            recovery_patterns, collision_precursors, correlations
        )

        return AnalysisReport(
            session_count=1,  # Could track across sessions
            total_actions=total_actions,
            recovery_patterns=recovery_patterns,
            collision_precursors=collision_precursors,
            sensor_correlations=correlations,
            recommendations=recommendations
        )

    def format_report_for_llm(self, report: AnalysisReport) -> str:
        """
        Format the analysis report as a string for LLM context.
        """
        lines = []
        lines.append("=== EXPERIENCE ANALYSIS REPORT ===")
        lines.append(f"Total recovery actions analyzed: {report.total_actions}")
        lines.append("")

        # Recovery patterns
        if report.recovery_patterns:
            lines.append("== RECOVERY PATTERNS ==")
            for pattern in report.recovery_patterns:
                lines.append(f"Action: {pattern.action_type}")
                lines.append(f"  Context: {pattern.context_conditions}")
                lines.append(f"  Best angles: {pattern.recommended_angles}")
                lines.append(f"  Success rate: {pattern.success_rate:.1%} (n={pattern.sample_size})")
                lines.append(f"  Evidence: {pattern.evidence}")
                lines.append("")
        else:
            lines.append("== RECOVERY PATTERNS ==")
            lines.append("  Not enough data to identify patterns yet.")
            lines.append("")

        # Collision precursors
        if report.collision_precursors:
            lines.append("== COLLISION WARNING SIGNS ==")
            for precursor in report.collision_precursors:
                lines.append(f"  {precursor.sensor_name} {precursor.direction} {precursor.threshold_value}mm")
                lines.append(f"    -> High collision likelihood (n={precursor.sample_size})")
            lines.append("")
        else:
            lines.append("== COLLISION WARNING SIGNS ==")
            lines.append("  Not enough collision data to identify patterns.")
            lines.append("")

        # Sensor correlations
        if report.sensor_correlations:
            lines.append("== SENSOR CORRELATIONS ==")
            for name, corr in report.sensor_correlations.items():
                direction = "positive" if corr > 0 else "negative"
                lines.append(f"  {name}: {direction} correlation ({corr:+.2f})")
            lines.append("")

        # Recommendations
        if report.recommendations:
            lines.append("== DATA-DRIVEN RECOMMENDATIONS ==")
            for rec in report.recommendations:
                lines.append(f"  - {rec}")
            lines.append("")

        return "\n".join(lines)

    def _bin_context(self, context: Dict[str, Any]) -> str:
        """
        Bin sensor context into categories for grouping.

        Creates a key like "left:close,right:far,front:medium"
        """
        parts = []

        # Front obstacle
        front = context.get('ultra_center') or context.get('tof') or 999
        if front < 100:
            parts.append("front:very_close")
        elif front < 200:
            parts.append("front:close")
        elif front < 400:
            parts.append("front:medium")
        else:
            parts.append("front:far")

        # Left vs right asymmetry
        left = context.get('ultra_left') or 999
        right = context.get('ultra_right') or 999
        if left < right - 50:
            parts.append("asym:left_closer")
        elif right < left - 50:
            parts.append("asym:right_closer")
        else:
            parts.append("asym:balanced")

        return ",".join(sorted(parts))

    def _parse_context_key(self, key: str) -> Dict[str, str]:
        """Parse a context key back into conditions"""
        conditions = {}
        for part in key.split(","):
            if ":" in part:
                name, value = part.split(":", 1)
                conditions[name] = value
        return conditions

    def _format_recovery_evidence(
        self,
        context_key: str,
        angle_stats: List[Tuple[int, float, int]],
        total_samples: int
    ) -> str:
        """Format evidence for a recovery pattern"""
        conditions = self._parse_context_key(context_key)
        cond_str = ", ".join(f"{k}={v}" for k, v in conditions.items())

        angle_str = ", ".join(
            f"{angle}° ({rate:.0%}, n={n})"
            for angle, rate, n in angle_stats[:3]
        )

        return f"When {cond_str}: best angles are {angle_str} (total n={total_samples})"

    def _generate_recommendations(
        self,
        patterns: List[RecoveryPattern],
        precursors: List[CollisionPrecursor],
        correlations: Dict[str, float]
    ) -> List[str]:
        """Generate actionable recommendations from patterns"""
        recommendations = []

        # From recovery patterns
        for pattern in patterns:
            if pattern.success_rate > 0.7 and pattern.sample_size >= 10:
                ctx = pattern.context_conditions
                if ctx.get('asym') == 'left_closer':
                    recommendations.append(
                        f"When obstacle is closer on left, prefer right turns: {pattern.recommended_angles}"
                    )
                elif ctx.get('asym') == 'right_closer':
                    recommendations.append(
                        f"When obstacle is closer on right, prefer left turns: {pattern.recommended_angles}"
                    )

        # From collision precursors
        for precursor in precursors:
            if precursor.sample_size >= 5:
                recommendations.append(
                    f"Start evasive action when {precursor.sensor_name} < {precursor.threshold_value}mm"
                )

        # From correlations
        if correlations.get('turn_away_from_obstacle', 0) > 0.2:
            recommendations.append(
                "Turning away from the closer obstacle improves recovery success"
            )

        return recommendations
