# Cozmo Explorer - Autonomous Mapping Platform

## Overview

An autonomous mapping platform using a Cozmo robot with an ESP32 sensor pod and trailer. The robot explores rooms, builds an occupancy grid map, and the system (LLM + database) learns about the environment from completed maps.

**Core principle: "The system learns, not Cozmo."** Cozmo drives toward unexplored areas, collects sensor data, and builds the map. After a session, the LLM reviews the map and produces semantic annotations. The robot is a surveyor; intelligence lives in the system around it.

## Origin

This project started as an experiment in embodied AI, building on concepts from the EchoFrontendV2 project (C#/.NET). It went through a rule-based learning phase (archived) before arriving at the current mapping platform architecture, which proved far more effective.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                         YOUR PC                               │
│                                                               │
│  ┌────────────────────────────────────────────────────────┐  │
│  │                MAPPER STATE MACHINE                     │  │
│  │  MAPPING → CAPTURING → REVIEWING → DONE                │  │
│  │     │                                                   │  │
│  │     └─ FrontierNavigator                                │  │
│  │        - find nearest frontier on map                   │  │
│  │        - drive toward it                                │  │
│  │        - update map continuously (sensor ray-tracing)   │  │
│  │        - escape if stuck (reverse-arc, deterministic)   │  │
│  │        - detect stagnation → relocate                   │  │
│  └────────────────────────────────────────────────────────┘  │
│          │                              │                      │
│          ▼                              ▼                      │
│  ┌───────────────┐           ┌──────────────────────────┐    │
│  │  Ollama LLM   │           │  Memory (SQLite)         │    │
│  │  Post-session  │           │  - Spatial map (.npz)    │    │
│  │  map review    │           │  - Map annotations       │    │
│  └───────────────┘           │  - Session logs          │    │
│                               │  - State persistence     │    │
│                               └──────────────────────────┘    │
│          │                                                     │
│          ▼                                                     │
│  ┌──────────────────────────────────────────────────────┐    │
│  │                 pycozmo Client                        │    │
│  │  - Motors/movement      - Camera frames               │    │
│  │  - Sensor readings      - Pose tracking               │    │
│  └──────────────────────────────────────────────────────┘    │
│                         │                                      │
└─────────────────────────┼──────────────────────────────────────┘
                          │ WiFi (UDP)
              ┌───────────┴───────────┐
              │                       │
              ▼                       ▼
       ┌─────────────┐        ┌─────────────┐
       │   COZMO     │        │   ESP32     │
       │   (WiFi)    │        │   (UDP)     │
       │   + trailer │        │   Sensors   │
       └─────────────┘        └─────────────┘
```

## Tech Stack

| Component | Choice | Why |
|-----------|--------|-----|
| Language | Python 3.11+ | Robotics ecosystem, async, pycozmo compatibility |
| Database | SQLite | State persistence, annotations, experience logs |
| LLM | Ollama (local, Gemma 3) | Runs on PC, no API costs, post-session review |
| Robot SDK | pycozmo | Direct WiFi control, no phone app needed |
| Map | NumPy occupancy grid | 50mm resolution, sensor ray-tracing |
| Simulation | Custom (pygame rendering) | Physics + 4 simulated sensors, 10x time scaling |

## Project Structure

```
cozmo-explorer/
├── main.py                         # Entry point (mapping platform)
├── config.py                       # All tunable parameters
├── CLAUDE.md                       # Context for Claude Code sessions
│
├── brain/
│   ├── frontier_navigator.py       # FrontierNavigator - main navigation loop
│   ├── mapper_state_machine.py     # MapperStateMachine - 4-state mapping cycle
│   ├── session_reviewer.py         # Post-session LLM map review
│   ├── behaviors.py                # Base behaviors (DriveDistance, TurnAngle, etc)
│   ├── state_machine.py            # Legacy state machine (unused by mapper)
│   ├── memory_context.py           # Context builder for LLM
│   └── personality.py              # Journal system
│
├── memory/
│   ├── spatial_map.py              # Occupancy grid, frontier finding, ray-tracing
│   ├── map_annotations.py          # SQLite semantic annotations from LLM review
│   ├── experience_logger.py        # Session data logging (SQLite)
│   ├── state_store.py              # Session management, robot state persistence
│   ├── experience_db.py            # ChromaDB (legacy, kept for future use)
│   └── conversation_memory.py      # Token-budgeted memory pools (legacy)
│
├── perception/
│   ├── external_sensors.py         # ESP32 serial/UDP reader
│   ├── camera.py                   # Frame capture
│   └── vision_observer.py          # Vision analysis
│
├── llm/
│   ├── client.py                   # Ollama client
│   ├── review_prompts.py           # Map review prompts (post-session)
│   └── prompts.py                  # Legacy prompts
│
├── cozmo_interface/
│   └── robot.py                    # Robot control wrapper (pycozmo)
│
├── simulator/
│   ├── run_full_sim.py             # Headless simulation harness
│   ├── run_sim.py                  # Interactive simulation
│   ├── sim_robot.py                # Simulated robot (physics + ray-cast sensors)
│   ├── physics.py                  # 2D physics engine
│   ├── world.py                    # World definitions (rooms, furniture)
│   └── renderer.py                 # Pygame visualization
│
├── control/
│   └── manual_controller.py        # Tkinter GUI for manual driving
│
├── audio/
│   └── voice.py                    # TTS + chunked audio playback
│
├── archive/                        # Archived code (old rule-learning system)
│   ├── learning_coordinator.py     # LLM-based rule proposal pipeline
│   ├── learned_rules.py            # Rule storage/matching/application
│   ├── pattern_analyzer.py         # Recovery pattern analysis
│   └── maneuvers.py                # ZigzagManeuver
│
├── docs/                           # Documentation
│   ├── ARCHITECTURE.md             # System architecture details
│   ├── READING_ORDER.md            # Quick-start reading guide
│   ├── HARDWARE_SETUP.md           # Hardware architecture
│   ├── ESP32_INTEGRATION.md        # Sensor pod details
│   ├── SETUP_GUIDE.md              # How to run
│   └── archive/                    # Archived docs (old architecture)
│
├── scripts/
│   ├── check_status.py             # Database health check
│   └── test_charger_bypass.py      # Hardware test
│
└── data/                           # Runtime data (not in git)
    ├── state.db                    # SQLite database
    ├── spatial_map.npz             # Occupancy grid
    ├── mapping_images/             # Images captured during mapping
    ├── cozmo.log                   # Session log
    └── sim_report_*.txt            # Simulation reports
```

## Hardware

| Component | Details |
|-----------|---------|
| Cozmo robot | pycozmo WiFi control, trailer attached |
| ESP32 sensor pod | ToF + 3x ultrasonic + MPU6050 IMU, JSON over UDP port 5000 |
| Arduino Nano relay | Powers ESP32 (must be plugged in) |
| Trailer | Carries sensor pod, requires arc-based turns |

## How It Works

1. **MAPPING**: FrontierNavigator finds the nearest unexplored boundary on the occupancy grid and drives toward it. Sensors ray-trace into the map continuously. If stuck, it executes a deterministic reverse-arc escape (135 degrees away from the closer obstacle).

2. **Stagnation detection**: If map coverage gain drops below 0.5% per 60s, the navigator switches to relocation mode - using cluster-based frontier selection to find a large unexplored region and drive there.

3. **CAPTURING**: Periodically captures images at new positions for later LLM review.

4. **REVIEWING**: After mapping completes, the LLM reviews the ASCII map + session stats and produces semantic annotations (room type, landmarks, doorways, coverage gaps).

5. **Annotations persist** in SQLite alongside the spatial map, building a persistent understanding of the environment that any robot or system can use later.

## Running

```bash
# Real robot
python main.py

# Simulation (headless, fast)
python -m simulator.run_full_sim --world furnished_room --duration 3600 --time-scale 10

# Simulation (rendered, visual)
python -m simulator.run_full_sim --world furnished_room --duration 600 --time-scale 5 --render

# Interactive simulation
python -m simulator.run_sim
```

## Current Performance (2026-02-07)

| Metric | Value |
|--------|-------|
| Map known | 56.5% in 7200s sim (12 min wall clock at 10x) |
| Cells visited | 1.1% (114 cells physically traversed) |
| Escapes | 18 |
| Stagnation relocations | 3 |
| Rules used | 0 (pure frontier navigation) |
| LLM calls during navigation | 0 |

Compare: old rule-based system achieved 1.6% in 10 hours with 2,884 escapes.

## Future Directions

- **Real robot testing** - verify sim results translate to physical hardware
- **Platform migration** - may switch from Cozmo to SMARS for better mobility
- **Multi-room mapping** - extend to explore across doorways
- **Post-session LLM** - develop the annotation pipeline for richer environment understanding
- **Multi-robot fleet** - specialized robots (mapper, photographer, scout) sharing a common map

## Resources

- [pycozmo GitHub](https://github.com/zayfod/pycozmo)
- [pycozmo Documentation](https://pycozmo.readthedocs.io/)
- [Ollama](https://ollama.ai/)
