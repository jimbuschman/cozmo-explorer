# Cozmo Explorer - System Architecture

## Design Philosophy

**"The system learns, not Cozmo."**

Previous iterations tried to make the robot learn in real-time (proposing rules from escape patterns, LLM analyzing behavior every 5 minutes). After 10 hours of sim time, 42 learned rules, and 2,884 escape attempts, it achieved only 1.6% map coverage. The rules optimized escape turn angles but never addressed the real problem: the robot had no idea where to go.

The current architecture treats Cozmo as a mapping platform. The robot drives toward unexplored areas and builds the map. The system (LLM + database) learns from completed maps. This achieved 56.5% coverage in 2 hours of sim time with zero rules.

---

## Component Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        MAPPER STATE MACHINE                          │
│  brain/mapper_state_machine.py                                      │
│                                                                      │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐     │
│  │ MAPPING  │───▶│CAPTURING │───▶│REVIEWING │───▶│   DONE   │     │
│  │          │◀───│          │    │          │    │          │     │
│  └────┬─────┘    └──────────┘    └──────────┘    └──────────┘     │
│       │                                                              │
│       ▼                                                              │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │               FRONTIER NAVIGATOR                              │  │
│  │  brain/frontier_navigator.py                                  │  │
│  │                                                               │  │
│  │  while not done:                                              │  │
│  │    1. Update map (ray-trace all sensors)                      │  │
│  │    2. Check for obstacles → escape if needed                  │  │
│  │    3. Check for stalls → escape if needed                     │  │
│  │    4. Pick frontier target (nearest, or cluster if relocating)│  │
│  │    5. Turn toward target                                      │  │
│  │    6. Drive forward                                           │  │
│  └──────────────────────────────────────────────────────────────┘  │
│       │                                                              │
│       ▼                                                              │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │                    SPATIAL MAP                                 │  │
│  │  memory/spatial_map.py                                        │  │
│  │                                                               │  │
│  │  - 100x100 occupancy grid (5000x5000mm at 50mm resolution)   │  │
│  │  - Cell states: UNKNOWN, FREE, OCCUPIED, VISITED              │  │
│  │  - Sensor ray-tracing (marks cells along ray as FREE,         │  │
│  │    endpoint as OCCUPIED)                                      │  │
│  │  - Frontier finding (UNKNOWN adjacent to FREE/VISITED)        │  │
│  │  - Cluster-based frontier selection for relocation             │  │
│  └──────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

---

## FrontierNavigator (brain/frontier_navigator.py)

The core navigation loop. Runs in 30-second segments called by the mapper state machine.

### Main Loop

```
for each tick (0.05s):
    update_map(sensors)         # ray-trace 4 sensors into occupancy grid

    if obstacle too close:
        record escape direction
        execute reverse-arc escape
        skip to next tick

    if stall detected (IMU):
        record escape direction
        execute reverse-arc escape
        skip to next tick

    check stagnation (every 60s sim-time):
        if coverage gain < 0.5%:
            enter relocation mode
        if relocating and coverage recovering:
            exit relocation mode

    if no target or time to recheck:
        if relocating:
            target = find_best_frontier_cluster()   # big region, far away
        else:
            target = find_nearest_frontier()         # closest unexplored edge
            (exclude angle of recent escape if applicable)

    if heading is off:
        turn toward target (update map during turn)

    drive forward
```

### Escape Mechanism

Escapes are **deterministic** (no rules, no learning):

1. **Reverse-arc**: Robot backs up while simultaneously turning. Implemented as a step loop (0.05s ticks) with negative wheel speeds to prevent the sim collision handler from freezing the wheels.
2. **Direction**: Turns 135 degrees away from the closer side (left vs right sensor).
3. **Data basis**: 2,884 escape samples showed 120/135 degrees work at ~100% success. 90 degrees never works with the trailer.

### Escape Exclusion

After escaping, the navigator avoids re-targeting frontiers in the direction of the obstacle:
- Exclusion cone: 45 degrees base + 15 degrees per consecutive escape, capped at 120 degrees
- Clears after 30 clean driving ticks (no stalls or obstacles)
- If exclusion filters ALL frontiers, retries without exclusion (fallback)

### Stagnation Detection & Relocation

The robot can get stuck scanning from one position (lighthouse effect). Detection uses coverage rate:

1. Track `spatial_map.get_exploration_progress()` every 60 sim-seconds
2. If gain < 0.5% since last check, enter relocation mode
3. In relocation mode: use `find_best_frontier_cluster()` to pick a large, distant unexplored region
4. Drive there, then resume normal nearest-frontier navigation when coverage rate recovers

**Key implementation detail**: Stagnation state persists across 30-second navigator runs because the mapper reuses the same navigator instance (not recreated each time).

### State Persistence

These instance variables persist across `run()` calls:
- `_last_coverage_pct` - coverage at last check
- `_cumulative_elapsed` - total sim-seconds across all runs
- `_last_coverage_check_elapsed` - cumulative elapsed at last stagnation check
- `_relocating` - whether in relocation mode
- `escapes` - total escape count
- `frontier_targets` - total frontier targets selected

---

## MapperStateMachine (brain/mapper_state_machine.py)

Simple 4-state machine that orchestrates the mapping cycle.

### States

| State | Description | Transitions |
|-------|-------------|-------------|
| MAPPING | Runs FrontierNavigator in 30s segments | → CAPTURING (image interval), → REVIEWING (all frontiers done) |
| CAPTURING | Takes camera image at current position | → MAPPING |
| REVIEWING | Post-session LLM analyzes the map | → DONE |
| DONE | No more frontiers to explore | (terminal) |

### Safety Checks (in main loop)
- **Picked up**: stops motors, waits
- **Low battery**: stops motors, waits (threshold: 3.4V)

### Navigator Persistence

The FrontierNavigator instance is created once and reused across all MAPPING segments. This is critical because stagnation detection needs state to accumulate over 60+ seconds, but each MAPPING segment only runs for 30 seconds.

---

## Spatial Map (memory/spatial_map.py)

100x100 occupancy grid covering a 5000x5000mm area at 50mm resolution.

### Cell States

| State | Value | Meaning |
|-------|-------|---------|
| UNKNOWN | 0 | Not yet observed |
| FREE | 1 | Sensor ray passed through (empty space) |
| OCCUPIED | 2 | Sensor ray endpoint (obstacle) |
| VISITED | 3 | Robot physically drove through |

### Sensor Ray-Tracing

Each tick, the navigator calls `update_from_sensors()` which traces rays from the robot's position through each of the 4 sensors:
- Cells along the ray path: marked FREE
- Cell at the endpoint (if sensor reports an obstacle): marked OCCUPIED
- Cell at the robot's position: marked VISITED

### Frontier Finding

A **frontier** is an UNKNOWN cell adjacent to a FREE or VISITED cell (the boundary between explored and unexplored space).

- `find_nearest_frontier(x, y)` - Returns the closest frontier cell. Supports `exclude_angle` and `exclude_cone` parameters for escape exclusion.
- `find_best_frontier_cluster(x, y)` - BFS flood-fill to find connected frontier components. Scores each cluster by `size / sqrt(distance + 1)` (information-gain heuristic). Returns nearest point of highest-scoring cluster.

### Persistence

Maps save/load via NumPy `.npz` files. Maps persist across sessions (loaded at startup if available).

---

## Post-Session LLM Review

After mapping completes, the system optionally runs an LLM review:

### Components

| File | Purpose |
|------|---------|
| `brain/session_reviewer.py` | Orchestrates the review: builds prompt, calls LLM, parses response |
| `llm/review_prompts.py` | Prompt templates for map analysis |
| `memory/map_annotations.py` | SQLite storage for annotations |

### Flow

1. Generate ASCII map from occupancy grid
2. Build prompt with map + stats (escapes, coverage, time)
3. LLM analyzes and returns structured annotations
4. Annotations stored in SQLite with coordinates and type

### Annotation Types

- Room type identification ("this looks like a living room")
- Landmark detection ("table in center", "wall on east side")
- Doorway/opening detection
- Coverage gap flagging ("northeast corner needs more exploration")

---

## Simulator

The simulator replicates the real robot environment for fast iteration.

### Components

| File | Purpose |
|------|---------|
| `simulator/sim_robot.py` | Simulated robot with 4 ray-cast sensors, physics integration |
| `simulator/physics.py` | 2D physics engine (collision detection, response) |
| `simulator/world.py` | Room definitions with walls and furniture |
| `simulator/run_full_sim.py` | Headless simulation harness with time scaling |
| `simulator/run_sim.py` | Interactive simulation with keyboard control |
| `simulator/renderer.py` | Pygame visualization |

### Time Scaling

The simulation patches `asyncio.sleep` globally to divide sleep times by the time scale factor. At 10x, a 3600-second simulation completes in ~360 seconds of wall clock time.

### Sensor Simulation

The sim robot casts rays for each of the 4 sensors (ToF, ultrasonic left/center/right) using the same angles as the physical sensor geometry defined in `config.py`. Rays check for intersection with world walls and furniture obstacles.

---

## Data Flow

```
ESP32 Sensors ──(UDP)──▶ ExternalSensorReader ──callback──▶ robot.sensors
                                                                  │
pycozmo ──(WiFi)──▶ Robot.pose                                   │
                       │                                           │
                       ▼                                           ▼
              FrontierNavigator.run()
                       │
                       ├── update_map(sensors) ──▶ SpatialMap.update_from_sensors()
                       │                                   │
                       │                                   ▼
                       │                          Occupancy grid updated
                       │
                       ├── find_nearest_frontier() ──▶ SpatialMap.find_nearest_frontier()
                       │
                       ├── drive toward target ──▶ Robot.set_wheels()
                       │
                       └── escape if stuck ──▶ Robot.set_wheels() (reverse-arc)
                                                   │
                                                   ▼
                                          MapperStateMachine
                                                   │
                                                   ├── CAPTURING ──▶ Robot.capture_image()
                                                   │
                                                   └── REVIEWING ──▶ SessionReviewer
                                                                         │
                                                                         ▼
                                                                  MapAnnotationStore (SQLite)
```

---

## Configuration (config.py)

### Key Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `WANDER_SPEED` | 70 mm/s | Forward driving speed |
| `ESCAPE_SPEED` | 100 mm/s | Escape maneuver speed |
| `ESCAPE_ANGLE` | 135 degrees | Escape turn magnitude |
| `LOW_BATTERY_VOLTAGE` | 3.4V | Battery safety threshold |
| `COLLISION_ACCEL_THRESHOLD` | 2500 | Accelerometer delta for collision |
| `IMAGE_CAPTURE_INTERVAL` | 120s | Time between image captures |
| `TRAILER_MODE` | True | Enable arc-based turns |
| `TRAILER_ARC_RATIO` | 0.5 | Inner wheel speed ratio |

### FrontierNavigator Constants (in frontier_navigator.py)

| Constant | Value | Purpose |
|----------|-------|---------|
| `CAUTION_DISTANCE` | 200mm | Obstacle escape trigger distance |
| `FRONTIER_RECHECK_INTERVAL` | 3.0s | How often to pick a new frontier target |
| `TARGET_REACHED_DISTANCE` | 200mm | Distance to consider target reached |
| `STAGNATION_CHECK_INTERVAL` | 60.0s | Coverage rate check interval |
| `STAGNATION_MIN_GAIN` | 0.005 | Minimum coverage gain (0.5%) per interval |
| `RELOCATE_MIN_DISTANCE` | 1500mm | Minimum relocation distance |

---

## Archived Systems

The `archive/` directory contains the old rule-learning system, kept for reference:

| File | What it did |
|------|-------------|
| `learning_coordinator.py` | LLM-based rule proposal pipeline. Analyzed escape patterns every 5 minutes, proposed behavioral rules. |
| `learned_rules.py` | SQLite storage for rules. Lifecycle: proposed → testing → validated → active. |
| `pattern_analyzer.py` | Statistical analysis of recovery patterns for rule proposals. |
| `maneuvers.py` | ZigzagManeuver (never used). |

**Why archived**: The rule-learning system optimized the wrong thing (escape angles) while ignoring the real problem (navigation strategy). 42 rules, 0 measurable impact on mapping performance.
