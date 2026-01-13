# Cozmo Explorer - Autonomous Robot Project

## Overview

An autonomous exploration system for a Cozmo robot, using an LLM for high-level decision making. The robot explores its environment, builds a map, remembers what it's seen, and makes its own decisions about what to investigate next.

This is a fun experiment applying autonomy/self-directed learning concepts to physical robotics.

## Origin

This project builds on concepts from the EchoFrontendV2 project (C#/.NET), which implemented:
- Multi-pool memory management with token budgeting
- Embedding-based semantic recall
- Background job queues for async LLM calls
- Function calling patterns

Those patterns translate well to robotics:
- Memory pools → attention budget for sensors, goals, spatial memory
- Embeddings → "have I seen this before?" recall
- Async queues → non-blocking LLM queries while robot moves
- Function calls → robot queries knowledge, updates beliefs

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      YOUR PC                                 │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │                    BRAIN                              │  │
│  │  State Machine: IDLE → EXPLORE → INVESTIGATE → ...   │  │
│  └──────────────────────────────────────────────────────┘  │
│          │                              │                    │
│          ▼                              ▼                    │
│  ┌───────────────┐              ┌───────────────────────┐  │
│  │  Ollama LLM   │              │  Memory (ChromaDB)    │  │
│  │  "What next?" │              │  "Have I seen this?"  │  │
│  └───────────────┘              └───────────────────────┘  │
│          │                              │                    │
│          └──────────────┬───────────────┘                   │
│                         ▼                                    │
│  ┌──────────────────────────────────────────────────────┐  │
│  │                 pycozmo Client                        │  │
│  │  - Motors/movement                                    │  │
│  │  - Camera frames                                      │  │
│  │  - Sensor readings                                    │  │
│  └──────────────────────────────────────────────────────┘  │
│                         │                                    │
└─────────────────────────┼────────────────────────────────────┘
                          │ WiFi (UDP)
                          ▼
                   ┌─────────────┐
                   │   COZMO     │
                   └─────────────┘
```

## Tech Stack

| Component | Choice | Why |
|-----------|--------|-----|
| Language | Python 3.11+ | Robotics ecosystem, async, pycozmo compatibility |
| Vector DB | ChromaDB (embedded) | Lightweight, no server, handles similarity search |
| Regular DB | SQLite | Map storage, state persistence |
| LLM | Ollama (local) | Runs on PC, no API costs |
| Robot SDK | pycozmo | Direct WiFi control, no phone app needed |

## Project Structure

```
cozmo-explorer/
├── main.py                    # Entry point
├── config.py                  # Settings
│
├── brain/
│   ├── state_machine.py       # Core loop: sense → think → act
│   ├── behaviors.py           # Wander, wall-follow, go-to, look-around
│   └── goal_executor.py       # Break LLM goals into behavior sequences
│
├── memory/
│   ├── spatial_map.py         # Where have I been? What's where?
│   ├── experience_db.py       # ChromaDB - visual/semantic memories
│   └── state_store.py         # SQLite - persistent state
│
├── perception/
│   ├── camera.py              # Frame capture, basic processing
│   ├── sensors.py             # Cliff, accel, gyro aggregation
│   └── vision.py              # Optional: object detection, SLAM
│
├── llm/
│   ├── client.py              # Talk to Ollama
│   └── prompts.py             # How to describe state to LLM
│
└── cozmo_interface/
    ├── robot.py               # Wrapper around pycozmo
    └── display.py             # Show status on Cozmo's face
```

## Cozmo Capabilities (via pycozmo)

| Category | Capabilities |
|----------|--------------|
| Movement | Wheel motors, head angle, lift position |
| Vision | Camera feed (frame capture) |
| Sensors | Cliff detectors, accelerometer, gyroscope |
| Feedback | OLED face display, LEDs, speaker |
| Data | Battery voltage, pose tracking |

## State Machine States

| State | Description |
|-------|-------------|
| IDLE | Waiting, conserving power |
| EXPLORE | Wandering to discover new areas |
| GO_TO | Navigating to a specific point |
| INVESTIGATE | Looking closely at something interesting |
| STUCK | Can't make progress, need help or reroute |
| CHARGING | On charger, waiting for battery |

## Autonomy Framework (from original concept)

The original "Autonomy & Self-Directed Growth Framework" maps to robot behaviors:

| Original Concept | Robot Implementation |
|-----------------|----------------------|
| Self-Driven Goal Formation | "I haven't explored that area yet" |
| Iterative Self-Improvement | "My distance estimates are off, recalibrate" |
| Decision-Making & Prioritization | "Battery low - return to base vs. finish room" |
| Reflection & Course Correction | "I keep getting stuck here, mark as difficult" |
| Intrinsic Curiosity | "What's behind that obstacle?" |

## Implementation Phases

### Phase 1: Foundation ✓
- [x] Project setup (requirements.txt, basic structure)
- [x] pycozmo connection and basic control
- [x] Simple behaviors (drive, turn, stop)
- [x] Camera frame capture

### Phase 2: Basic Autonomy ✓
- [x] State machine framework
- [x] Wander behavior (random exploration)
- [x] Cliff/obstacle avoidance
- [x] Basic spatial tracking (where am I?)

### Phase 3: Memory ✓
- [x] SQLite state persistence
- [x] ChromaDB for experience storage
- [x] Simple occupancy grid or visited-points map
- [x] "Have I been here before?" queries

### Phase 4: LLM Integration ✓
- [x] Ollama client
- [x] State-to-prompt formatting
- [x] Goal parsing from LLM responses
- [x] Periodic "what should I do?" queries

### Phase 5: Polish & Expansion (Future)
- [ ] Better mapping (visual odometry?)
- [ ] Object detection/recognition
- [ ] More sophisticated exploration strategies
- [ ] Multiple robot support (future)

## Connection Steps (pycozmo)

1. Place Cozmo on charging platform
2. Raise and lower lift to display WiFi PSK
3. Connect PC to Cozmo's WiFi network using displayed password
4. Run the application

## Dependencies

```
pycozmo>=0.8.0
chromadb
ollama
httpx
numpy
Pillow
```

## Future: Raspberry Pi Migration

If migrating to a Pi-based custom robot later:
- Brain/memory code transfers directly (pure Python)
- Replace pycozmo interface with custom sensor/motor bridge
- Add ESP32 microcontrollers for real-time control
- Pi handles high-level logic, microcontrollers handle reflexes

## Resources

- [pycozmo GitHub](https://github.com/zayfod/pycozmo)
- [pycozmo Documentation](https://pycozmo.readthedocs.io/)
- [ChromaDB](https://www.trychroma.com/)
- [Ollama](https://ollama.ai/)
