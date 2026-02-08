# Cozmo Explorer - Setup & Running Guide

## Hardware Required

| Component | Quantity | Notes |
|-----------|----------|-------|
| Cozmo Robot | 1 | Must be charged, off the charger to explore |
| Computer | 1 | Windows/Linux/Mac with WiFi and USB |
| ESP32 Sensor Pod | 1 | ToF + 3x ultrasonic + MPU6050 IMU |
| Arduino Nano | 1 | Relay for ESP32 power (must be plugged in) |
| Trailer | 1 | Carries sensor pod, attached to Cozmo |

## Network Topology

```
┌─────────────────────────────────────────────────────────────────┐
│                         YOUR COMPUTER                            │
│                                                                  │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐   │
│  │   Python     │  │   Ollama     │  │   ESP32 UDP          │   │
│  │   main.py    │  │   (LLM)     │  │   (port 5000)         │   │
│  └──────┬───────┘  └──────────────┘  └──────────┬───────────┘   │
│         │                                        │               │
│         │ WiFi (Cozmo's network)                 │ WiFi (UDP)    │
│         │                                        │               │
└─────────┼────────────────────────────────────────┼───────────────┘
          │                                        │
          ▼                                        ▼
    ┌───────────┐                          ┌─────────────┐
    │   COZMO   │◄─────── sits on ────────►│ ESP32 POD   │
    │  + trailer │                          │ (sensors)   │
    └───────────┘                          └─────────────┘
```

**Key Point:** Your computer connects to Cozmo's WiFi network directly (not your home WiFi). This means you lose internet access while connected to Cozmo unless you have a second network adapter.

## Software Required

### On Your Computer

1. **Python 3.8+**
   ```bash
   python --version
   ```

2. **Project Dependencies**
   ```bash
   cd cozmo-explorer
   pip install -r requirements.txt
   ```

3. **Ollama** (for post-session LLM review) - Optional
   - Download from: https://ollama.ai
   - Install and run:
     ```bash
     ollama serve
     ```
   - Pull the model:
     ```bash
     ollama pull gemma3:latest
     ```

4. **Serial Driver** (for ESP32 if using USB serial mode)
   - Windows: Usually auto-installs, or get CP210x/CH340 driver
   - Linux: Usually works out of box, may need `sudo usermod -a -G dialout $USER`

## Step-by-Step Setup

### Step 1: Prepare Cozmo

1. Turn on Cozmo (lift and lower his arm)
2. Wait for him to wake up and show the WiFi name on his face
3. Note the WiFi name: `Cozmo_XXXXXX`

### Step 2: Connect Computer to Cozmo's WiFi

1. Open WiFi settings on your computer
2. Connect to `Cozmo_XXXXXX` network
3. **Note:** You will lose internet access while connected

### Step 3: Connect ESP32 Sensor Pod

The ESP32 pod communicates via WiFi UDP by default (port 5000).

1. Plug in the Arduino Nano relay (powers the ESP32)
2. The ESP32 should auto-connect to your network and start broadcasting UDP

To override connection settings:
```bash
# UDP mode (default)
set EXT_SENSOR_MODE=udp
set EXT_SENSOR_UDP_PORT=5000

# Serial mode (if using USB cable)
set EXT_SENSOR_MODE=serial
set EXT_SENSOR_PORT=COM3
```

### Step 4: Start Ollama (Optional)

In a **separate terminal** (before connecting to Cozmo's WiFi):

```bash
ollama serve
```

If you can't run Ollama, the system skips the post-session review but mapping works fine.

### Step 5: Run Cozmo Explorer

```bash
cd cozmo-explorer
python main.py
```

### Expected Output

```
08:30:15 | INFO     | __main__            | ============================================================
08:30:15 | INFO     | __main__            |   COZMO EXPLORER - Mapping Platform
08:30:15 | INFO     | __main__            | ============================================================
08:30:15 | INFO     | __main__            | Connecting to state store...
08:30:15 | INFO     | __main__            | Initializing experience logger...
08:30:16 | INFO     | __main__            | Checking LLM availability...
08:30:16 | INFO     | __main__            | LLM ready: gemma3:latest (post-session review)
08:30:16 | INFO     | __main__            | Connecting to Cozmo...
08:30:18 | INFO     | cozmo_interface.robot | Connected to Cozmo!
08:30:18 | INFO     | __main__            | Robot connected!
08:30:18 | INFO     | __main__            | Connecting external sensors via WiFi UDP (port 5000)...
08:30:20 | INFO     | __main__            | External sensors connected!
08:30:20 | INFO     | __main__            | Initialization complete!
08:30:20 | INFO     | __main__            | Mode: autonomous, with trailer
08:30:20 | INFO     | __main__            | Started mapping session 1
```

### Step 6: Let It Map

- Place Cozmo on the floor in a room with obstacles
- The robot will automatically:
  - Drive toward unexplored areas (frontiers)
  - Build the occupancy grid map from sensor data
  - Escape obstacles with reverse-arc maneuvers
  - Relocate if coverage rate stalls

### Periodic Status (every 30 seconds)

```
08:31:00 | INFO     | __main__            | ----------------------------------------
08:31:00 | INFO     | __main__            | Mapper | Control: AUTO TRAILER
08:31:00 | INFO     | __main__            | State: MAPPING
08:31:00 | INFO     | __main__            | Position: (150, 230)
08:31:00 | INFO     | __main__            | Battery: 3.72V
08:31:00 | INFO     | __main__            | Map: 2.3% visited, 15.7% known
08:31:00 | INFO     | __main__            | Escapes: 3 | Targets: 12
08:31:00 | INFO     | __main__            | Sensors: F=450mm L=200mm R=380mm
08:31:00 | INFO     | __main__            | ----------------------------------------
```

### Step 7: Stop Gracefully

Press `Ctrl+C` to stop. The system will:
1. Save the spatial map
2. Save session stats
3. Run post-session LLM review (if available)
4. Disconnect from robot and sensors

### Manual Control Mode (Optional)

Set `MANUAL_CONTROL_ENABLED = True` in `config.py` to launch a GUI:

**Keyboard Controls:**
| Key | Action |
|-----|--------|
| Arrow Up/Down | Forward/Backward |
| Arrow Left/Right | Turn (arc turn with trailer) |
| A/D | Gentle arc turns |
| Space | Stop |
| M | Toggle Manual/Auto mode |
| Escape | Exit |

## Running the Simulator

No hardware needed - tests the mapping system in a virtual room.

```bash
# Fast headless run (3600 sim-seconds = ~6 min at 10x)
python -m simulator.run_full_sim --world furnished_room --duration 3600 --time-scale 10

# Visual run with pygame rendering
python -m simulator.run_full_sim --world furnished_room --duration 600 --time-scale 5 --render

# Interactive sim (keyboard control)
python -m simulator.run_sim
```

### Sim Output

The simulator produces a report in `data/sim_report_*.txt` with:
- Map coverage percentages
- Escape count
- ASCII visualization of the map
- Cell state breakdown

## Troubleshooting

### "Failed to connect to Cozmo"
- Make sure your computer is connected to Cozmo's WiFi
- Make sure Cozmo is awake (not sleeping)
- Try restarting Cozmo

### "External sensors not available"
- Check that Arduino Nano relay is plugged in
- Check if ESP32 is broadcasting on the expected port
- Try serial mode: `set EXT_SENSOR_MODE=serial` and `set EXT_SENSOR_PORT=COM3`

### "LLM not available"
- Make sure Ollama is running (`ollama serve`)
- Make sure you pulled the model (`ollama pull gemma3:latest`)
- System works fine without LLM - it just skips the post-session review

### Low battery warnings
- Battery drains faster with trailer attached
- Buck converter for external power is planned
- Charge between sessions

## Data Locations

| Data | Location |
|------|----------|
| SQLite Database | `data/state.db` |
| Spatial Map | `data/spatial_map.npz` |
| Mapping Images | `data/mapping_images/` |
| Log File | `data/cozmo.log` |
| Sim Reports | `data/sim_report_*.txt` |
