# Cozmo Explorer - Project Context

## RULES FOR CLAUDE - READ EVERY TIME
1. **REVIEW DATA BEFORE CHANGING CODE.** Do not change code based on assumptions. Read the logs the user provides. Analyze them. Then discuss before editing.
2. **Cozmo is a mapping platform, not a learning agent.** The system (map + LLM + DB) learns. Cozmo just drives around and collects data.
3. **Escape behavior is deterministic.** Reverse-arc (backup while turning) at 100mm/s, 135 degrees away from closer side. Data proves this works. Do NOT add rules or learning to escape logic.
4. **The simulator tests infrastructure, not behavior.** Use it to verify map building works, code doesn't crash, UI renders. Current benchmark: ~56% map coverage in 7200 sim-seconds (2hr sim, 12min wall clock at 10x).
5. **Do not make changes without discussing them first.** Explain what you want to change and why. Wait for approval.
6. **This robot is a mapper/surveyor.** Its job is to build the spatial map. Other robots will use the map later. Platform may change to SMARS.
7. **Stop going in circles.** If you find yourself reverting or rewriting the same code, stop and talk to the user.
8. **Don't take shortcuts.** Think through the proper solution instead of defaulting to the quick/easy answer.

## What This Is
Mapping platform: Cozmo + ESP32 sensor pod + trailer. Python on PC controls robot over WiFi, ESP32 sends sensor data via UDP. LLM (Ollama/Gemma3) reviews completed maps post-session and produces semantic annotations.

## Architecture (2026-02-07)

### Core Principle
"The system learns, not Cozmo." Cozmo drives toward unexplored areas, builds the occupancy grid map, escapes when stuck. After a session, the LLM reviews the map and produces semantic annotations (room type, landmarks, doorways, coverage gaps). These annotations persist in SQLite alongside the spatial map.

### Navigation
- `brain/frontier_navigator.py` - FrontierNavigator: find nearest frontier, drive toward it, update map continuously, escape if stuck. No rules, no random wandering.
- **Escape**: reverse-arc (backs up while turning simultaneously) in 0.05s step loop. Turns 135 degrees away from closer side. Step loop re-sets wheel speeds each tick so sim collision handler can't freeze the robot.
- **Escape exclusion**: after escaping, the navigator avoids picking frontiers in the direction of the obstacle. Exclusion cone widens with consecutive escapes (45° base, +15° per repeat, max 120°). Clears after 30 clean driving checks.
- **Stagnation detection**: tracks map coverage rate across runs. If coverage gain drops below 0.5% per 60s sim-time, triggers relocation mode.
- **Relocation**: when stagnant, uses `find_best_frontier_cluster()` (information-gain heuristic — clusters frontier cells, scores by size/sqrt(distance)) to pick a big unexplored region to drive to. Resumes normal nearest-frontier scanning when coverage rate recovers.

### State Machine
- `brain/mapper_state_machine.py` - MapperStateMachine: 4 states (MAPPING, CAPTURING, REVIEWING, DONE)
  - MAPPING: runs FrontierNavigator in 30s segments. Navigator instance is **persisted** across runs (not recreated) so stagnation/relocation state accumulates.
  - CAPTURING: takes images at new areas for LLM review
  - REVIEWING: post-session LLM analysis of ASCII map + stats
  - DONE: no more frontiers

### LLM Role (post-session only)
- `brain/session_reviewer.py` - Reviews completed ASCII map + session stats
- `llm/review_prompts.py` - Prompts for map analysis
- `memory/map_annotations.py` - SQLite table for semantic annotations (room type, landmarks, doorways, coverage gaps)
- LLM is NOT involved in real-time navigation. It only runs after mapping is complete.

### Archived (in archive/, not deleted)
- `learning_coordinator.py` - Rule proposal/validation pipeline
- `learned_rules.py` - Rule storage/matching/application
- `pattern_analyzer.py` - Recovery pattern analysis
- `maneuvers.py` - ZigzagManeuver

## Hardware
- Cozmo with trailer attached (TRAILER_MODE = True)
- ESP32 sensor pod: ToF + 3x ultrasonic + MPU6050 IMU, sends JSON over UDP port 5000
- Nano relay for ESP32 (must be plugged in for sensors to work)
- Battery drains fast with trailer drag - buck converter on order for external power

## Key Architecture Facts
- robot.turn() in trailer mode = slow 30mm/s arcs (for normal navigation)
- robot.escape_turn() = fast 100mm/s arcs (for escapes)
- FrontierNavigator escape = reverse-arc (negative wheel speeds) in step loop, not escape_turn()
- Stall detection: 4 checks warmup + 5 checks threshold (IMU yaw + distance), skipped during escape cooldown (3s)
- ESP32 resting pitch is ~17.5 degrees (mounting angle) - tilt detection relative to baseline
- Best escape angles: 120, 135 degrees. 90 degrees never works with trailer.
- Collision detection via accelerometer (threshold 2500)
- Tilt threshold 25 degrees with 3 consecutive readings required
- Stagnation detection: 0.5% min coverage gain per 60s sim-time, uses cumulative elapsed across 30s navigator runs
- Relocation: cluster-based frontier selection (find_best_frontier_cluster) with size/sqrt(distance) scoring

## Sim Results (2026-02-07)
- **56.5% map known** in 7200s sim (2hr sim, 12min wall clock at 10x)
- **1.1% visited** (114 cells) — robot physically explores, not just scanning from one spot
- **18 escapes** — mostly from relocation driving into furniture, handled by escape exclusion
- 3 stagnation→relocate→resume cycles during the run
- 0 rules, 0 LLM calls during navigation
- Compare: old rule-based system got 1.6% in 10 hours with 2,884 escapes

### Web Dashboard
- `web/server.py` - aiohttp WebServer: REST API + SSE for live status + PNG map endpoint
- `web/map_renderer.py` - numpy LUT + Pillow: occupancy grid → 400x400 PNG (<1ms render)
- `web/static/index.html` - single-page dashboard (vanilla JS, EventSource for SSE, no build tools)
- `web_main.py` - web-first entry point: starts server on port 8080, opens browser, user clicks Start
- Dashboard auto-starts when running `python main.py` if `WEB_ENABLED=True` in config

## Files to Know
- `config.py` - all tunable parameters (includes WEB_PORT, WEB_ENABLED)
- `cozmo_interface/robot.py` - robot control, escape_turn(), collision detection
- `brain/frontier_navigator.py` - FrontierNavigator (main navigation loop)
- `brain/mapper_state_machine.py` - MapperStateMachine (4-state machine)
- `brain/session_reviewer.py` - post-session LLM map review
- `memory/spatial_map.py` - occupancy grid, frontier finding, ray-tracing
- `memory/map_annotations.py` - semantic annotation storage
- `memory/experience_logger.py` - session data logging
- `perception/external_sensors.py` - ESP32 UDP/serial reader
- `llm/client.py` - Ollama client
- `simulator/sim_robot.py` - physics + ray-cast sensors
- `simulator/run_full_sim.py` - full simulation test harness
- `web/server.py` - aiohttp web server + SSE + API
- `web/map_renderer.py` - occupancy grid PNG renderer
- `web_main.py` - web dashboard entry point

## Test Commands
- **Web dashboard**: `python web_main.py` (opens browser, select mode + world, click Start)
- Full run (real robot): `python main.py` (dashboard at http://localhost:8080 if WEB_ENABLED)
- Sim test (headless): `python -m simulator.run_full_sim --world furnished_room --duration 3600 --time-scale 10`
- Sim test (rendered): `python -m simulator.run_full_sim --world furnished_room --duration 600 --time-scale 5 --render`
- Interactive sim: `python -m simulator.run_sim`
- Quick connection test: `python -m cozmo_interface.robot`
