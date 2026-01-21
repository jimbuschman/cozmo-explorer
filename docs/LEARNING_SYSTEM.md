# Cozmo Explorer - Learning & Memory System

## Overview

The Learning System enables Cozmo to improve its navigation behavior over time by:
1. Logging sensor data, actions, and outcomes
2. Analyzing patterns in the data
3. Proposing behavioral rules (via LLM or statistics)
4. Validating rules through testing
5. Activating validated rules to modify behavior

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              DATA COLLECTION                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  WanderBehavior                                                              │
│       │                                                                      │
│       ├── _escape_stall() / _escape_cliff()                                 │
│       │       │                                                              │
│       │       ├── Capture "before" image                                    │
│       │       ├── Log sensor snapshot (with image path)                     │
│       │       ├── Apply learned rules (if any)                              │
│       │       ├── Execute recovery maneuver                                 │
│       │       ├── Capture "after" image                                     │
│       │       ├── Log outcome                                               │
│       │       └── Record rule performance                                   │
│       │                                                                      │
│       └──────────────────────────────────────────────────────────────────── │
│                                    │                                         │
│                                    ▼                                         │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                     ExperienceLogger (SQLite)                        │    │
│  │                                                                      │    │
│  │  sensor_snapshots    action_events       outcome_events              │    │
│  │  ├─ pose x/y/angle   ├─ action_type      ├─ outcome_type            │    │
│  │  ├─ battery          ├─ parameters       ├─ details                 │    │
│  │  ├─ cliff_detected   ├─ trigger          └─ sensor_snapshot_id      │    │
│  │  ├─ accel x/y/z      ├─ context_id                                  │    │
│  │  ├─ gyro x/y/z       └─ timestamp                                   │    │
│  │  ├─ head_angle                                                       │    │
│  │  ├─ lift_height                                                      │    │
│  │  ├─ ext_* (distances)                                               │    │
│  │  ├─ ext_* (IMU raw)                                                 │    │
│  │  └─ image_path                                                       │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│                              ANALYSIS & LEARNING                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  StateMachine._run_learning_cycle()  (every 5 minutes)                      │
│       │                                                                      │
│       ▼                                                                      │
│  ┌─────────────────────┐     ┌─────────────────────┐                        │
│  │   PatternAnalyzer   │────▶│  AnalysisReport     │                        │
│  │                     │     │  - recovery_patterns│                        │
│  │  - analyze_recovery │     │  - collision_precur │                        │
│  │  - analyze_collisions│    │  - correlations     │                        │
│  │  - find_correlations│     │  - recommendations  │                        │
│  └─────────────────────┘     └──────────┬──────────┘                        │
│                                         │                                    │
│                                         ▼                                    │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                      LearningCoordinator                             │    │
│  │                                                                      │    │
│  │  1. Format report for LLM                                           │    │
│  │  2. Query LLM for rule proposals (or use statistical fallback)      │    │
│  │  3. Safety check each proposal                                      │    │
│  │  4. Store as "proposed" status                                      │    │
│  │  5. Track testing progress                                          │    │
│  │  6. Validate after enough samples                                   │    │
│  │  7. Activate validated rules                                        │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                         │                                    │
│                                         ▼                                    │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                     LearnedRulesStore (SQLite)                       │    │
│  │                                                                      │    │
│  │  learned_rules                                                       │    │
│  │  ├─ name, description                                               │    │
│  │  ├─ conditions: [{"sensor": "ext_ultra_l_mm", "op": "<", "value": 200}]│ │
│  │  ├─ action_modifier: {"turn_angle_preference": [-90, -120]}         │    │
│  │  ├─ status: proposed → testing → validated/rejected → active        │    │
│  │  ├─ scope: "global" | "location" (future)                           │    │
│  │  ├─ location_id: null (for future location-based rules)             │    │
│  │  ├─ times_applied, times_successful (for conflict resolution)       │    │
│  │  └─ evidence_summary, test_results, safety_score                    │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│                              RULE APPLICATION                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  WanderBehavior._escape_stall()                                             │
│       │                                                                      │
│       ├── base_action = {"angles": [-120, -90, 90, 120], ...}              │
│       │                                                                      │
│       ├── sensor_context = {ext_ultra_l_mm: 150, ext_ultra_r_mm: 400, ...} │
│       │                                                                      │
│       ▼                                                                      │
│  LearnedRulesStore.apply_rules_to_action(base_action, sensor_context)       │
│       │                                                                      │
│       ├── Find all active rules that match sensor_context                   │
│       │                                                                      │
│       ├── CONFLICT RESOLUTION (if multiple rules match):                    │
│       │   1. Location-scoped rules beat global rules (future)               │
│       │   2. More specific rules beat less specific (# of conditions)       │
│       │   3. Higher success rate breaks ties                                │
│       │                                                                      │
│       ├── Apply best rule's modifier                                        │
│       │                                                                      │
│       └── Return (modified_action, applied_rule_id)                         │
│                                                                              │
│       Result: {"angles": [-90, -120], ...}  (only right turns if left closer)│
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Data Captured

### Cozmo Internal Sensors
| Sensor | Field | Notes |
|--------|-------|-------|
| Odometry | pose_x, pose_y, pose_angle | From wheel encoders, drifts over time |
| Cliff | cliff_detected | IR sensors on front underside |
| Accelerometer | accel_x, accel_y, accel_z | Used for collision detection |
| Gyroscope | gyro_x, gyro_y, gyro_z | Rotation rates |
| Pickup | is_picked_up | Detects when lifted |
| Battery | battery_voltage | |
| Charger | is_on_charger | |
| Head | head_angle | Radians |
| Lift | lift_height | mm |

### External Sensors (ESP32 Pod)
| Sensor | Field | Notes |
|--------|-------|-------|
| Time-of-Flight | ext_tof_mm | Front distance |
| Ultrasonic L/C/R | ext_ultra_l/c/r_mm | Left, center, right distances |
| IMU Orientation | ext_pitch, ext_roll, ext_yaw | Derived angles |
| IMU Accel Raw | ext_ax_g, ext_ay_g, ext_az_g | In g's |
| IMU Gyro Raw | ext_gx_dps, ext_gy_dps, ext_gz_dps | Degrees per second |
| Timestamp | ext_ts_ms | ESP32 timestamp |

### Images
- Captured before and after recovery actions
- Stored in `data/learning_images/`
- Path stored in `sensor_snapshots.image_path` and `outcome_events.details`

## Rule Lifecycle

```
         ┌───────────┐
         │  PROPOSED │  ← LLM or statistical analysis creates rule
         └─────┬─────┘
               │ Safety check passes
               ▼
         ┌───────────┐
         │  TESTING  │  ← Rule is applied, outcomes tracked
         └─────┬─────┘
               │ After MIN_SAMPLES_FOR_VALIDATION (default: 10)
               ▼
    ┌──────────┴──────────┐
    │                     │
    ▼                     ▼
┌───────────┐       ┌───────────┐
│ VALIDATED │       │ REJECTED  │
└─────┬─────┘       └───────────┘
      │ Manual or auto-activate
      ▼
┌───────────┐
│  ACTIVE   │  ← Rule modifies behavior
└───────────┘
```

### Validation Criteria
- **Pass**: 10%+ improvement over baseline OR 70%+ success with no regression
- **Reject**: Insufficient improvement after enough samples
- **Timeout**: Auto-reject if not validated within 24 hours

## Safety Constraints

Rules cannot:
- Disable `cliff_detected` or `is_picked_up` safety checks
- Set turn angles outside -180° to 180°
- Set backup duration outside 0.3s to 2.0s
- Set speed multiplier outside 0.3x to 2.0x

## Conflict Resolution

When multiple rules match the current sensor context:

1. **Scope Priority**: Location-scoped rules beat global rules (for future use)
2. **Specificity**: More conditions = more specific = higher priority
3. **Success Rate**: Historical performance breaks ties

Example:
```
Rule A: 1 condition, 60% success rate
Rule B: 2 conditions, 70% success rate
Rule C: 2 conditions, 65% success rate

If all match → Rule B wins (most specific + highest success)
```

## Graceful Degradation

| Missing Component | Behavior |
|-------------------|----------|
| No LLM | Uses statistical analysis for rule proposals |
| No external sensors | Uses internal sensors only (accel, cliff) |
| No rules_store | Falls back to hardcoded angles |
| No experience_logger | No logging, original behavior |

## Files

### New Files
| File | Purpose |
|------|---------|
| `memory/experience_logger.py` | SQLite logging of sensors, actions, outcomes |
| `memory/pattern_analyzer.py` | Extract patterns from logged data |
| `memory/learned_rules.py` | Store and apply validated rules |
| `brain/learning_coordinator.py` | Orchestrate LLM analysis and validation |

### Modified Files
| File | Changes |
|------|---------|
| `llm/prompts.py` | Added LEARNING_ANALYSIS_PROMPT |
| `brain/behaviors.py` | Added logging and rule application to WanderBehavior |
| `brain/state_machine.py` | Added periodic learning cycle |
| `main.py` | Wire up all learning components |
| `cozmo_interface/robot.py` | Added external IMU raw fields |

## Future: Location-Based Rules (Phase 2)

The schema is ready for location-based rules:
- `scope` field: "global" or "location"
- `location_id` field: Links to map region

**Phase 1 data limitations:**
- No room recognition during data collection
- pose_x/pose_y are odometry only (drift, reset each session)
- Phase 1 data = global rules only

**Phase 2 requirements:**
- Room recognition running during data collection
- Each snapshot tagged with location_id at collection time
- Location-scoped rules get priority over global rules

## Configuration

| Setting | Default | Location |
|---------|---------|----------|
| LEARNING_INTERVAL_SECONDS | 300 (5 min) | state_machine.py |
| MIN_SAMPLES_FOR_ANALYSIS | 20 | learning_coordinator.py |
| MIN_SAMPLES_FOR_VALIDATION | 10 | learning_coordinator.py |
| VALIDATION_TIMEOUT_HOURS | 24 | learning_coordinator.py |
| ANALYSIS_COOLDOWN_MINUTES | 30 | learning_coordinator.py |

## Usage

### Viewing Learning Status

During runtime, status is printed periodically:
```
Learning: 45 samples, 2 active rules, 1 testing
```

At shutdown:
```
Learning system: 5 rules, 45 samples
  Rules by status: {'active': 2, 'rejected': 1, 'proposed': 2}
```

### Manual Rule Management

```python
# Get all rules
rules = rules_store.get_all_rules()

# Get active rules
active = rules_store.get_active_rules()

# Manually activate a validated rule
rules_store.update_rule_status(rule_id, "active")

# Disable a rule
rules_store.update_rule_status(rule_id, "disabled")

# Delete a rule
rules_store.delete_rule(rule_id)
```

### Querying Experience Data

```python
# Get recovery statistics
stats = experience_logger.get_recovery_statistics("escape_stall")
# {'total': 45, 'success_rate': 0.67, 'by_angle': {...}}

# Get action-outcome pairs for analysis
pairs = experience_logger.get_action_outcome_pairs(
    action_type="escape_stall",
    outcome_type="success",
    limit=100
)

# Get collision precursors
precursors = experience_logger.get_collision_precursors()
```

## Testing Checklist

1. **Phase 1 Test**: Run 2 sessions, verify sensor_snapshots and action_events tables populated
2. **Phase 2 Test**: Run `pattern_analyzer.generate_analysis_report()`, verify meaningful output
3. **Phase 3 Test**: Manually add a rule, verify it modifies turn angles in `_escape_stall()`
4. **Phase 4 Test**: Trigger learning cycle, verify LLM proposals are parsed and stored
5. **Phase 5 Test**: Validate a proposal, verify status transitions and rule activation
