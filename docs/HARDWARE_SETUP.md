# Cozmo Explorer - Hardware Architecture

## Overview

This document describes the hardware setup for the Cozmo Explorer system.

**Key Principle: Grow into complexity, don't start with it.**

---

## Current Setup (Phase 1) - USE THIS NOW

```
┌─────────────────────────────────────────────────────────────────┐
│                         MAIN PC                                  │
│                                                                  │
│  ┌────────────────┐  ┌────────────────┐  ┌─────────────────┐   │
│  │  Python App    │  │  Ollama LLM    │  │  SQLite DB      │   │
│  │  (main.py)     │  │  (localhost)   │  │  (local files)  │   │
│  └───────┬────────┘  └────────────────┘  └─────────────────┘   │
│          │                                                       │
│          ├── pycozmo (over Cozmo's WiFi network)                │
│          │                                                       │
│          └── USB Serial (ESP32 sensor pod)                      │
│                                                                  │
└──────────┼───────────────────────────────────────────────────────┘
           │
           ├─────────────────┐
           │                 │
           ▼                 ▼
    ┌─────────────┐   ┌─────────────┐
    │   COZMO     │   │   ESP32     │
    │   (WiFi)    │   │   (USB)     │
    └─────────────┘   └─────────────┘
```

**This is simple and it works.** Don't add complexity until you need it.

---

## Future Setup (Phase 3) - BUILD TOWARD THIS

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                              ROBOT LAYER                                             │
│                                                                                      │
│  ┌─────────────────────┐    ┌─────────────────────┐    ┌─────────────────────┐     │
│  │     COZMO #1        │    │     COZMO #2        │    │     COZMO #N        │     │
│  │  (Built-in sensors) │    │  (Built-in sensors) │    │  (Built-in sensors) │     │
│  └──────────┬──────────┘    └──────────┬──────────┘    └──────────┬──────────┘     │
│             │                          │                          │                 │
│  ┌──────────┴──────────┐    ┌──────────┴──────────┐    ┌──────────┴──────────┐     │
│  │   ESP32 / Arduino   │    │   ESP32 / Arduino   │    │   ESP32 / Arduino   │     │
│  │   (External sensors)│    │   (External sensors)│    │   (External sensors)│     │
│  │   ToF, Ultrasonic,  │    │   Different sensor  │    │   Different sensor  │     │
│  │   IMU, etc.         │    │   configuration     │    │   configuration     │     │
│  └──────────┬──────────┘    └──────────┬──────────┘    └──────────┬──────────┘     │
│             │                          │                          │                 │
│             └──────────────────────────┼──────────────────────────┘                 │
│                                        │                                            │
│                                   WiFi │                                            │
│                                        ▼                                            │
└────────────────────────────────────────┼────────────────────────────────────────────┘
                                         │
┌────────────────────────────────────────┼────────────────────────────────────────────┐
│                              EDGE LAYER                                              │
│                                        │                                            │
│                           ┌────────────┴────────────┐                               │
│                           │    RASPBERRY PI 4B+     │                               │
│                           │                         │                               │
│                           │  • Receives sensor data │                               │
│                           │  • Timestamps messages  │                               │
│                           │  • Buffers/queues data  │                               │
│                           │  • Forwards to main PC  │                               │
│                           │                         │                               │
│                           └────────────┬────────────┘                               │
│                                        │                                            │
│                                Ethernet│                                            │
│                                        ▼                                            │
└────────────────────────────────────────┼────────────────────────────────────────────┘
                                         │
┌────────────────────────────────────────┼────────────────────────────────────────────┐
│                              BRAIN LAYER                                             │
│                                        │                                            │
│                           ┌────────────┴────────────┐                               │
│                           │       MAIN PC           │                               │
│                           │   (Currently: Your PC)  │                               │
│                           │   (Future: Older PC)    │                               │
│                           │                         │                               │
│                           │  • Learning System      │                               │
│                           │  • State Machine        │                               │
│                           │  • Map Building         │                               │
│                           │  • Database (SQLite)    │                               │
│                           │  • Rule Engine          │                               │
│                           │                         │                               │
│                           └────────────┬────────────┘                               │
│                                        │                                            │
│                              (Optional)│ Network                                    │
│                                        ▼                                            │
│                           ┌─────────────────────────┐                               │
│                           │      LLM SERVER         │                               │
│                           │   (Currently: Same PC)  │                               │
│                           │   (Future: Separate PC) │                               │
│                           │                         │                               │
│                           │  • Ollama               │                               │
│                           │  • gemma3:latest        │                               │
│                           └─────────────────────────┘                               │
│                                                                                      │
└──────────────────────────────────────────────────────────────────────────────────────┘
```

---

## Layer Details

### Robot Layer

Each robot consists of:

**Cozmo Robot (Built-in)**
- Cliff IR sensors (front underside)
- Accelerometer
- Gyroscope
- Camera
- Wheel encoders (odometry)
- Battery sensor

**ESP32/Arduino (External)**
- Connected via USB to Cozmo's charging platform or mounted separately
- Communicates via WiFi to Raspberry Pi
- Possible sensors:
  - VL53L0X Time-of-Flight (front distance)
  - HC-SR04 Ultrasonics (left/center/right)
  - MPU6050 IMU (pitch/roll/yaw)
  - Additional sensors as needed

**Data Output Format (JSON over WiFi)**
```json
{
  "robot_id": "scout",
  "ts_ms": 12345,
  "tof_mm": 150,
  "ultra_l_mm": 200,
  "ultra_c_mm": 180,
  "ultra_r_mm": 220,
  "mpu": {
    "ax_g": 0.01,
    "ay_g": -0.02,
    "az_g": 1.0,
    "pitch": 2.5,
    "roll": -1.2,
    "yaw": 45.0
  }
}
```

---

### Edge Layer (Raspberry Pi 4B+)

**Purpose**: Buffer, timestamp, and forward sensor data

**Responsibilities** (future multi-robot setup):
1. Receive WiFi data from multiple ESP32/Arduino units
2. Add server timestamp to each message
3. Buffer messages in queue (handle network hiccups)
4. Forward to main PC over Ethernet
5. Handle reconnection if robots disconnect

**Note**: Currently not used. ESP32 sends UDP directly to the PC. The Pi becomes useful when adding a second robot or needing more reliable data buffering.

**Why a Pi?**
- Reliable WiFi receiver
- Can handle multiple robot connections
- Ethernet to main PC (more reliable than WiFi)
- Low power, always-on
- Can run lightweight preprocessing

**Software Components**:
```
┌─────────────────────────────────────────┐
│           RASPBERRY PI                   │
│                                          │
│  ┌──────────────────────────────────┐   │
│  │     WiFi Receiver Service        │   │
│  │  • UDP/TCP listener per robot    │   │
│  │  • JSON parsing                  │   │
│  │  • Add timestamp                 │   │
│  └──────────────┬───────────────────┘   │
│                 │                        │
│                 ▼                        │
│  ┌──────────────────────────────────┐   │
│  │     Message Queue                │   │
│  │  • Redis or simple file queue    │   │
│  │  • Buffers during network issues │   │
│  └──────────────┬───────────────────┘   │
│                 │                        │
│                 ▼                        │
│  ┌──────────────────────────────────┐   │
│  │     Ethernet Forwarder           │   │
│  │  • TCP connection to main PC     │   │
│  │  • Reliable delivery             │   │
│  │  • Reconnection handling         │   │
│  └──────────────────────────────────┘   │
│                                          │
└─────────────────────────────────────────┘
```

**Message Format (Pi → Main PC)**:
```json
{
  "robot_id": "scout",
  "robot_ts_ms": 12345,
  "pi_ts_iso": "2024-01-15T10:30:00.123Z",
  "sensors": {
    "tof_mm": 150,
    "ultra_l_mm": 200,
    "ultra_c_mm": 180,
    "ultra_r_mm": 220,
    "pitch": 2.5,
    "roll": -1.2,
    "yaw": 45.0
  }
}
```

---

### Brain Layer (Main PC)

**Current**: Your main PC
**Future**: Older dedicated PC

**Responsibilities**:
1. Run the main cozmo-explorer Python code
2. MapperStateMachine + FrontierNavigator (autonomous mapping)
3. Spatial map building and storage (occupancy grid)
4. Database (SQLite - state, annotations, experience logs)
5. Post-session LLM review (Ollama)
6. Coordinate multiple robots (future)

**Why separate from Pi?**
- More CPU/RAM for map processing
- Larger storage for database and images
- Can run LLM locally
- Easier to develop on

---

### LLM Layer (Optional, Separate)

**Current**: Ollama on main PC
**Future**: Could move to separate PC

**Why separate later?**
- LLM inference is CPU/GPU heavy
- Doesn't need to be low-latency
- Can run on older hardware with GPU
- Main PC can focus on real-time robot control

**Connection**: HTTP API (Ollama default: `http://localhost:11434`)

---

## Hardware Progression - When to Add What

### Current Setup ← YOU ARE HERE
```
1 Cozmo + trailer + ESP32 ──WiFi UDP──> Main PC (runs everything + Ollama)
```
- **Hardware**: PC + Cozmo + ESP32 via WiFi UDP (port 5000)
- **Software**: MapperStateMachine + FrontierNavigator (autonomous mapping)
- **Why this works**: Simple, untethered, ESP32 broadcasts sensor data over WiFi
- **Don't add yet**: Pi, separate LLM server

### Future: Multi-Robot (When adding second robot OR wanting robustness)
```
Multiple Cozmos + ESP32s ──WiFi──> Raspberry Pi ──Ethernet──> Main PC
```
- **Trigger**: Second robot, or reliability issues, or place recognition needs more data
- **Add**: Raspberry Pi as edge aggregator
- **Why Pi now**: Multiple WiFi sources, buffering, central timestamp

### Future: Scaling Up
```
Multiple Cozmos + ESP32s ──WiFi──> Raspberry Pi ──Ethernet──> Main PC
                                                                  │
                                                     ┌────────────┘
                                                     ▼
                                              Separate LLM PC
```
- **Trigger**: Main PC is overloaded, or want dedicated always-on system
- **Add**: Dedicated brain PC, separate LLM server
- **Why**: Offload LLM inference, dedicated robot control

---

## Network Configuration

### Robot WiFi
- Each ESP32/Arduino connects to a local WiFi network
- NOT Cozmo's WiFi (that's for pycozmo control)
- Could be a dedicated robot WiFi network or home WiFi

### Cozmo Control
- PC connects to Cozmo's WiFi for pycozmo commands
- This is separate from sensor data path
- Challenge: PC needs multiple WiFi adapters for multiple Cozmos

### Sensor Data Path
```
ESP32 ──(WiFi)──> Home Router ──(WiFi)──> Raspberry Pi ──(Ethernet)──> Main PC
```

### Alternative: Direct ESP32 to Pi
```
ESP32 ──(WiFi, direct)──> Raspberry Pi (as access point)
```
Pi acts as WiFi access point for robots.

---

## Hardware Shopping List

### Current Setup (Have This)
- [x] 1 Cozmo robot + trailer
- [x] 1 ESP32 DevKit + sensors (ToF, ultrasonics, IMU)
- [x] 1 Arduino Nano (relay for ESP32 power)
- [x] Main PC

### Future: Buy When Adding Second Robot
- [ ] Raspberry Pi 4B+ (4GB or 8GB)
- [ ] Ethernet cable
- [ ] MicroSD card (32GB+)
- [ ] Pi power supply
- [ ] Additional Cozmo robot
- [ ] Additional ESP32/Arduino sensor unit
- [ ] USB WiFi adapter (for second Cozmo control from PC)

### Future: Buy When Scaling
- [ ] More Cozmo robots
- [ ] More sensor units
- [ ] More USB WiFi adapters
- [ ] Dedicated PC for brain (optional)
- [ ] Dedicated PC for LLM (optional)

---

## Data Flow Summary

```
1. Cozmo moves, ESP32 reads sensors
2. ESP32 sends JSON over WiFi
3. Pi receives, timestamps, queues
4. Pi forwards to Main PC over Ethernet
5. Main PC:
   - Logs to database
   - Updates state machine
   - Runs learning analysis
   - (Optionally) Queries LLM
6. Main PC sends commands back to Cozmo (via pycozmo over Cozmo's WiFi)
```

---

## Configuration

### ESP32 WiFi Config
```cpp
// In ESP32 firmware
const char* WIFI_SSID = "YourHomeWiFi";
const char* WIFI_PASS = "password";
const char* PI_HOST = "192.168.1.100";  // Pi's IP
const int PI_PORT = 5000;
```

### Raspberry Pi Config
```python
# pi_config.py
LISTEN_PORT = 5000
MAIN_PC_HOST = "192.168.1.50"  # Main PC IP
MAIN_PC_PORT = 6000
BUFFER_SIZE = 1000  # Max queued messages
```

### Main PC Config
```python
# config.py (existing)
PI_HOST = "192.168.1.100"
PI_PORT = 6000
OLLAMA_HOST = "http://localhost:11434"  # or separate PC IP
```

---

## Future: Robot Roles

Different Cozmos with different sensor configurations:

| Robot ID | Role | Sensors | Purpose |
|----------|------|---------|---------|
| `scout` | Explorer | ToF, ultrasonics, IMU | Fast exploration, find interesting spots |
| `camera` | Photographer | High-res camera | Detailed visual documentation |
| `mapper` | Mapper | Precise odometry sensors | Accurate map building |
| `audio` | Listener | Microphones | Sound-based features (future) |

Each robot's ESP32 includes `robot_id` in messages so the system knows which robot sent what data.
