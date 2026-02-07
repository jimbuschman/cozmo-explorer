# Cozmo Explorer - Project Context

## What This Is
Autonomous exploration robot: Cozmo + ESP32 sensor pod + trailer. Python on PC controls robot over WiFi, ESP32 sends sensor data via UDP. LLM (Ollama/Gemma3) observes and journals in Phase 1, will direct in Phase 2.

## Current State (2026-02-06)

### Hardware
- Cozmo with trailer attached (TRAILER_MODE = True)
- ESP32 sensor pod: ToF + 3x ultrasonic + MPU6050 IMU, sends JSON over UDP port 5000
- Nano relay for ESP32 (must be plugged in for sensors to work)
- Battery drains fast with trailer drag - buck converter on order for external power

### Recent Changes (commit 48dcf8c)
All 8 fixes from comprehensive data review implemented:
1. **escape_turn()** in robot.py - uses 100mm/s arcs instead of broken 30mm/s turns
2. **Emergency turns use escape_turn()** with 120-degree angles (was 90 with slow turn)
3. **Learning pipeline fixed** - proposed rules now promote to "testing" status
4. **Yaw wrapping** in stall detection (350->10 = 20, not 340)
5. **Stall tracking resets** after escapes (last_yaw, last_front_dist)
6. **Tilt baseline** calibrated lazily from first real sensor reading (not startup zeros)
7. **_pick_turn_direction** handles -1 (disconnected) sensors via _valid_distance()
8. **SENSOR_MIN_DISTANCE** removed from config, _valid_distance() simplified
9. **Audio collision suppression** - _audio_playing flag prevents speaker vibration false positives
10. **Tilt threshold 25 degrees** - was 15, too sensitive to driving acceleration

### What Works
- Trailer mode with arc turns (escape success went from 9% to ~100%)
- 120/135 degree turn angles (data confirms high success)
- IMU-based stall detection (yaw + distance change)
- Collision detection via accelerometer (threshold 2500)
- Learning system proposing rules from data patterns
- Rules promoting from proposed -> testing (as of fix #3)
- Tilt baseline calibration from first real ESP32 reading

### Known Issues / Not Yet Verified
- Escape turns (100mm/s arcs) not tested under load yet - battery died before long run
- Need 2-3 minute run to confirm: area coverage improvement, no stuck loops, rules accumulating test data
- Head angle for camera may be looking too high (seeing sensor mount instead of scene)
- LLM keeps proposing similar/duplicate rules each cycle (no dedup)
- Rules from previous sessions lose their _testing_rules stats on restart (coordinator is in-memory only)
- 24 rules stuck in "testing" in DB but not tracked in coordinator memory

### Key Architecture Facts
- robot.turn() in trailer mode = slow 30mm/s arcs (for normal navigation)
- robot.escape_turn() = fast 100mm/s arcs (for escapes - matches working cliff escape)
- Stall detection: 8 checks warmup (1.2s) + 8 checks threshold (1.2s) = ~2.4s to detect
- ESP32 resting pitch is ~17.5 degrees (mounting angle) - tilt detection is relative to this baseline
- Cliff escape uses arc_turn_left/right directly at escape_speed (already worked)
- Learning pipeline: proposed -> testing -> validated -> active (or rejected)

### Data
- 50 sessions, ~33 samples, 55% recovery success rate
- 34 rules total (0 active, ~29 testing, ~5 proposed)
- Best turn angles: 120, 135 degrees. Worst: 90 (0% with trailer)

### Files to Know
- `config.py` - all tunable parameters
- `cozmo_interface/robot.py` - robot control, escape_turn(), collision detection
- `brain/behaviors.py` - WanderBehavior (main driving loop), escape maneuvers, stall detection
- `brain/state_machine.py` - state loop, learning cycle, rule promotion
- `brain/learning_coordinator.py` - LLM-based rule proposal, validation pipeline
- `memory/learned_rules.py` - LearnedRulesStore (SQLite), rule CRUD, apply_rules_to_action()
- `perception/external_sensors.py` - ESP32 UDP/serial reader

### Test Commands
- Full run: `python main.py`
- Quick connection test: `python -m cozmo_interface.robot`
