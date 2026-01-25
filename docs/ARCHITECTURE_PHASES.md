# Cozmo Explorer - Phased Architecture

## Core Philosophy

This system is designed to **learn from the ground up**. Intelligence emerges from data, not prompts.

### The Two Layers

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        LLM ("The Explorer")                              │
│   • Curiosity & personality                                             │
│   • Goal-setting & journaling                                           │
│   • OPTIONAL - remove it and robots still function                      │
└───────────────────────────────────┬─────────────────────────────────────┘
                                    │ Intent (not control)
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                    LEARNING SYSTEM ("The Brain")                         │
│   • Autonomous survival & navigation                                    │
│   • Rule learning from experience                                       │
│   • Map building & room recognition                                     │
│   • REQUIRED - this is the core intelligence                            │
└─────────────────────────────────────────────────────────────────────────┘
```

**Key Principle**: The Learning System must work WITHOUT the LLM. The LLM adds purpose and personality, but the robot survives and learns on its own.

---

## Phase 1: Grounded Survival Learning (CURRENT)

### Purpose
Teach the system how to **exist safely** in the physical world.

### Behaviors
- Wander randomly
- Detect obstacles (collision, cliff, distance sensors)
- Recover from problems (back up, turn, retry)
- Log everything (sensors, actions, outcomes, images)
- Learn micro-rules ("when left sensor < 200mm, turn right works better")
- Build crude occupancy maps
- **Trailer mode**: Arc-based turns for pulling a trailer without jackknifing
- **Manual control**: Keyboard control for testing and generating training data

### Data Collected
| Source | Data | Purpose |
|--------|------|---------|
| Cozmo sensors | Cliff, accelerometer, gyro, pose | Safety, collision detection |
| ESP32 pod | ToF, ultrasonics, IMU | Proactive obstacle detection |
| Camera | Before/after images | Visual context for learning |
| Outcomes | Success/failure of actions | Rule validation |

### Learning Focus
- **Sensor → Action → Outcome** relationships
- Which recovery angles work best?
- What sensor patterns precede collisions?
- How to escape different situations?

### LLM Role (Optional)
| Does | Doesn't |
|------|---------|
| Observes and journals | Make real-time navigation decisions |
| Light commentary | Control motor commands |
| Records interesting events | Build maps |

### Success Criteria
- [ ] Robot can wander for 30+ minutes without getting permanently stuck
- [ ] Recovery success rate improves over time (rules are working)
- [ ] 100+ action-outcome pairs logged
- [ ] At least 3-5 validated rules active

### Architecture (Phase 1)

```
                    ┌─────────────────┐
                    │   LLM (Ollama)  │ ← Optional observer
                    │   - Journaling  │
                    │   - Light goals │
                    └────────┬────────┘
                             │ (optional)
                             ▼
┌──────────────────────────────────────────────────────────────┐
│                     STATE MACHINE                             │
│  IDLE ↔ EXPLORING ↔ STUCK ↔ CHARGING                        │
│  (Simple states, exploration-focused)                        │
└──────────────────────────────────────────────────────────────┘
                             │
              ┌──────────────┼──────────────┐
              ▼              ▼              ▼
      ┌─────────────┐ ┌─────────────┐ ┌─────────────┐
      │  Behaviors  │ │  Learning   │ │   Logging   │
      │  - Wander   │ │  - Rules    │ │  - Sensors  │
      │  - Recover  │ │  - Patterns │ │  - Actions  │
      │  - Avoid    │ │  - Validate │ │  - Outcomes │
      └─────────────┘ └─────────────┘ └─────────────┘
                             │
                             ▼
                    ┌─────────────────┐
                    │   COZMO ROBOT   │
                    │   + ESP32 Pod   │
                    └─────────────────┘
```

---

## Phase 2: Place & Context Learning (NEXT)

### Purpose
Teach the system to **recognize and remember places**.

### New Capabilities
- "Have I been here before?" → Place recognition
- "Does this room feel like that one?" → Room clustering
- "What rules work here?" → Location-scoped rules
- Segment maps into meaningful regions

### New Data Structures

#### Place Signatures
```python
@dataclass
class PlaceSignature:
    """A recognizable location pattern"""
    signature_id: int
    sensor_pattern: dict      # Typical sensor readings here
    visual_features: list     # Image embeddings/features
    audio_signature: dict     # Sound patterns (future)
    times_visited: int
    confidence: float
    linked_rules: list[int]   # Rules that work well here
```

#### Room Clusters
```python
@dataclass
class RoomCluster:
    """A collection of similar places = a 'room'"""
    room_id: int
    name: str                 # LLM-assigned or auto-generated
    place_signatures: list[int]
    boundary_signatures: list[int]  # Edge/transition places
    typical_obstacles: list[str]
    navigation_rules: list[int]
```

### Learning Focus
- Cluster experiences by similarity → "places"
- Group places into "rooms"
- Learn room-specific rules
- Detect room transitions
- Transfer rules: "Room B feels like Room A, try those rules"

### LLM Role (Active)
| Does | Doesn't |
|------|---------|
| Labels rooms ("this seems like a kitchen") | Direct motor control |
| Hypothesizes patterns ("rooms with soft floors...") | Real-time navigation |
| Cross-room reflection | Build the clusters (system does that) |
| Names places for journal | - |

### Success Criteria
- [ ] System recognizes 3+ distinct "places" in a room
- [ ] System clusters places into room(s)
- [ ] Location-scoped rules outperform global rules
- [ ] Robot can answer "have I been here before?" with 70%+ accuracy
- [ ] Rules transfer successfully to new-but-similar areas

### Architecture (Phase 2)

```
                    ┌─────────────────┐
                    │   LLM (Ollama)  │
                    │   - Labeling    │
                    │   - Hypotheses  │
                    │   - Reflection  │
                    └────────┬────────┘
                             │
                             ▼
┌──────────────────────────────────────────────────────────────┐
│                     PLACE RECOGNITION                         │
│  - Signature matching                                        │
│  - Room clustering                                           │
│  - Transition detection                                      │
└──────────────────────────────────────────────────────────────┘
                             │
                             ▼
┌──────────────────────────────────────────────────────────────┐
│                     STATE MACHINE                             │
│  + RECOGNIZING state                                         │
│  + Location-aware goal setting                               │
└──────────────────────────────────────────────────────────────┘
                             │
              ┌──────────────┼──────────────┐
              ▼              ▼              ▼
      ┌─────────────┐ ┌─────────────┐ ┌─────────────┐
      │  Behaviors  │ │  Learning   │ │   Logging   │
      │  (same)     │ │  + Scoped   │ │  + Place ID │
      │             │ │    rules    │ │  + Room ID  │
      └─────────────┘ └─────────────┘ └─────────────┘
```

---

## Phase 3: Purposeful Multi-Agent Exploration (FUTURE)

### Purpose
Turn curiosity into **coordinated action** across multiple robots.

### New Capabilities
- Multiple Cozmos with different sensor loadouts
- Task dispatch ("Camera bot, go photograph location X")
- Collaborative mapping
- Specialized roles (scout, photographer, mapper)
- Intent-driven exploration ("fully map the kitchen")

### Robot Roles (Example Fleet)

| Robot | Sensors | Role |
|-------|---------|------|
| Scout | ESP32 pod (ToF, ultrasonics, IMU) | Fast exploration, find interesting spots |
| Camera | High-res camera, good lighting | Detailed visual documentation |
| Mapper | Odometry-focused, careful movement | Precise map building |
| Audio | Microphone array | Sound-based localization (future) |

### Communication Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         CENTRAL COORDINATOR                              │
│   • Shared map & experience database                                    │
│   • Task queue & dispatch                                               │
│   • LLM for mission planning                                            │
│   • Runs on main PC                                                     │
└─────────────────────────────────────────────────────────────────────────┘
           │              │              │              │
           │    WiFi 1    │    WiFi 2    │    WiFi 3    │
           ▼              ▼              ▼              ▼
      ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐
      │ Cozmo 1 │    │ Cozmo 2 │    │ Cozmo 3 │    │ Cozmo N │
      │ Scout   │    │ Camera  │    │ Mapper  │    │  ...    │
      └─────────┘    └─────────┘    └─────────┘    └─────────┘
```

### Task Examples

**Scenario: Scout finds something interesting**
```
1. Scout Cozmo exploring kitchen
2. Detects unusual sensor pattern at (x, y)
3. Logs: "Novel signature, needs visual data"
4. Coordinator sees request
5. LLM decides: "Worth investigating"
6. Coordinator dispatches Camera Cozmo
7. Camera Cozmo navigates to (x, y) using learned rules
8. Takes photos, returns to base
9. System integrates visual + sensor data
10. LLM journals: "Found chair leg, updating room model"
```

### LLM Role (Mission Commander)
| Does | Doesn't |
|------|---------|
| Plans exploration missions | Direct individual motor commands |
| Prioritizes tasks | Navigate for the robots |
| Maintains narrative memory | Build maps (system does that) |
| Coordinates robot roles | Make split-second decisions |
| Long-horizon goals | - |

### Success Criteria
- [ ] 2+ robots successfully share a map
- [ ] Task dispatch works (Robot A requests, Robot B executes)
- [ ] Collaborative mapping produces better maps than single robot
- [ ] LLM can plan and execute multi-step missions
- [ ] System handles robot failures gracefully

---

## Data Flow Across Phases

### Phase 1 Data
```
Sensors → Actions → Outcomes → Rules
                 ↓
            Crude Map (occupancy)
```

### Phase 2 Data
```
Sensors → Actions → Outcomes → Rules (now scoped!)
    ↓                            ↑
Place Signatures ──→ Room Clusters
    ↓
Semantic Map (rooms, regions, transitions)
```

### Phase 3 Data
```
┌─────────────────────────────────────────────────────────┐
│                   SHARED DATABASE                        │
│  • Multi-robot experiences                              │
│  • Unified map (merged from all robots)                 │
│  • Task history                                         │
│  • Cross-robot rule validation                          │
└─────────────────────────────────────────────────────────┘
         ↑              ↑              ↑
     Robot 1        Robot 2        Robot N
```

---

## Hardware Evolution

### Phase 1 (Current)
- 1 Cozmo + ESP32 sensor pod
- PC runs everything
- Single WiFi connection (PC → Cozmo)
- ESP32 via USB serial

### Phase 2 (Same hardware, smarter software)
- Same setup
- More sophisticated data analysis
- Better use of existing sensors

### Phase 3 (Multi-robot)
- Multiple Cozmos (each with own WiFi network)
- PC needs multiple WiFi adapters OR
- Network bridging solution
- Centralized coordinator
- Shared database (SQLite → PostgreSQL?)

---

## File Structure Evolution

### Phase 1 (Current)
```
cozmo-explorer/
├── brain/
│   ├── behaviors.py          # Wander, recover, avoid
│   ├── state_machine.py      # Simple states
│   └── learning_coordinator.py
├── control/
│   └── manual_controller.py  # Tkinter GUI for manual control
├── memory/
│   ├── experience_logger.py  # Log everything
│   ├── pattern_analyzer.py   # Find patterns
│   ├── learned_rules.py      # Store/apply rules
│   └── spatial_map.py        # Crude occupancy
└── perception/
    └── external_sensors.py   # ESP32 reader
```

### Phase 2 (Add)
```
├── memory/
│   ├── place_recognition.py  # NEW: Place signatures
│   ├── room_clustering.py    # NEW: Group into rooms
│   └── semantic_map.py       # NEW: Meaningful map
└── brain/
    └── location_context.py   # NEW: Where am I?
```

### Phase 3 (Add)
```
├── coordination/
│   ├── task_dispatcher.py    # NEW: Multi-robot tasks
│   ├── robot_registry.py     # NEW: Track fleet
│   └── shared_memory.py      # NEW: Distributed state
└── network/
    ├── robot_bridge.py       # NEW: Multi-WiFi handling
    └── message_protocol.py   # NEW: Inter-robot comms
```

---

## Summary Table

| Aspect | Phase 1 | Phase 2 | Phase 3 |
|--------|---------|---------|---------|
| **Goal** | Survive | Recognize places | Coordinate fleet |
| **Robots** | 1 | 1 | Multiple |
| **Maps** | Crude occupancy | Semantic rooms | Shared, merged |
| **Rules** | Global | Location-scoped | Cross-robot validated |
| **LLM** | Observer | Labeler/hypothesizer | Mission commander |
| **Learning** | Micro-rules | Place patterns | Collaborative |
