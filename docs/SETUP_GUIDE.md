# Cozmo Explorer - Setup & Running Guide

## Hardware Required

| Component | Quantity | Notes |
|-----------|----------|-------|
| Cozmo Robot | 1 | Must be charged, off the charger to explore |
| Computer | 1 | Windows/Linux/Mac with WiFi and USB |
| ESP32 Sensor Pod | 1 | Optional but recommended for proactive obstacle detection |
| USB Cable | 1 | For ESP32 connection |

## Network Topology

```
┌─────────────────────────────────────────────────────────────────┐
│                         YOUR COMPUTER                            │
│                                                                  │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐   │
│  │   Python     │  │   Ollama     │  │   ESP32 Serial       │   │
│  │   main.py    │  │   (LLM)      │  │   (COM3 / ttyUSB0)   │   │
│  └──────┬───────┘  └──────────────┘  └──────────┬───────────┘   │
│         │                                        │               │
│         │ WiFi (Cozmo's network)                 │ USB Cable     │
│         │                                        │               │
└─────────┼────────────────────────────────────────┼───────────────┘
          │                                        │
          ▼                                        ▼
    ┌───────────┐                          ┌─────────────┐
    │   COZMO   │◄─────── sits on ────────►│ ESP32 POD   │
    │   ROBOT   │                          │ (sensors)   │
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

3. **Ollama** (for LLM) - Optional but recommended
   - Download from: https://ollama.ai
   - Install and run:
     ```bash
     ollama serve
     ```
   - Pull the model:
     ```bash
     ollama pull gemma3:latest
     ```

4. **Serial Driver** (for ESP32)
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

### Step 3: Connect ESP32 Sensor Pod (Optional)

1. Plug ESP32 into USB port
2. Find the serial port:
   - **Windows:** Open Device Manager → Ports → Note `COM3` or similar
   - **Linux:** `ls /dev/ttyUSB*` → Note `/dev/ttyUSB0` or similar
   - **Mac:** `ls /dev/tty.usb*`

3. Set environment variable if not default:
   ```bash
   # Windows
   set EXT_SENSOR_PORT=COM3

   # Linux/Mac
   export EXT_SENSOR_PORT=/dev/ttyUSB0
   ```

### Step 4: Start Ollama (Optional)

In a **separate terminal** (if you have internet on another adapter, or do this before connecting to Cozmo):

```bash
ollama serve
```

Leave this running. If you can't run Ollama, the system will fall back to statistical-only learning.

### Step 5: Run Cozmo Explorer

```bash
cd cozmo-explorer
python main.py
```

### Expected Output

```
08:30:15 | INFO     | __main__            | ============================================================
08:30:15 | INFO     | __main__            |   COZMO EXPLORER
08:30:15 | INFO     | __main__            | ============================================================
08:30:15 | INFO     | __main__            | Connecting to state store...
08:30:15 | INFO     | memory.experience_logger | ExperienceLogger connected: data\state.db
08:30:15 | INFO     | memory.learned_rules | LearnedRulesStore connected: data\state.db
08:30:15 | INFO     | __main__            | Learning coordinator initialized
08:30:15 | INFO     | __main__            | Connecting to experience database...
08:30:16 | INFO     | __main__            | Loaded 0 previous experiences
08:30:16 | INFO     | __main__            | Checking LLM availability...
08:30:16 | INFO     | llm.client          | LLM client ready: http://localhost:11434 using gemma3:latest
08:30:16 | INFO     | __main__            | LLM ready: gemma3:latest
08:30:16 | INFO     | __main__            | Connecting to Cozmo...
08:30:16 | INFO     | __main__            | (Make sure PC is connected to Cozmo's WiFi)
08:30:18 | INFO     | cozmo_interface.robot | Connected to Cozmo!
08:30:18 | INFO     | __main__            | Robot connected!
08:30:18 | INFO     | __main__            | Connecting external sensors on COM3...
08:30:20 | INFO     | __main__            | External sensors connected!
08:30:20 | INFO     | __main__            | Initialization complete!
```

### Step 6: Let It Explore

- Place Cozmo on the floor in a room with obstacles
- Let it wander and bump into things
- Watch the logs for:
  - `Collision detected` / `Stall detected`
  - `escape_stall` / `escape_cliff` actions
  - `Captured before/after image`

### Step 7: Stop Gracefully

Press `Ctrl+C` to stop. You'll see:

```
08:45:30 | INFO     | __main__            | Interrupt received, stopping...
08:45:31 | INFO     | __main__            | Shutting down...
08:45:31 | INFO     | __main__            | Learning system: 5 rules, 23 samples
08:45:31 | INFO     | __main__            |   Rules by status: {'proposed': 2, 'active': 1}
08:45:32 | INFO     | __main__            | Shutdown complete
```

## Verifying Data Collection

After running for a while, check that data is being collected:

### Check Database Tables

```bash
# Windows (install sqlite3 or use Python)
python -c "import sqlite3; c=sqlite3.connect('data/state.db'); print('Snapshots:', c.execute('SELECT COUNT(*) FROM sensor_snapshots').fetchone()[0]); print('Actions:', c.execute('SELECT COUNT(*) FROM action_events').fetchone()[0])"
```

### Check Images

```bash
# Windows
dir data\learning_images

# Linux/Mac
ls -la data/learning_images/
```

### Check Logs

```bash
# View the log file
# Windows
type data\cozmo.log

# Linux/Mac
cat data/cozmo.log
```

## Troubleshooting

### "Failed to connect to Cozmo"

- Make sure your computer is connected to Cozmo's WiFi
- Make sure Cozmo is awake (not sleeping)
- Try restarting Cozmo

### "External sensors not available"

- Check USB connection
- Check serial port name (COM3 vs COM4, etc.)
- Check if another program is using the port

### "LLM not available"

- Make sure Ollama is running (`ollama serve`)
- Make sure you pulled the model (`ollama pull gemma3:latest`)
- System will still work without LLM (statistical fallback)

### No collisions/stalls being detected

- Make sure external sensors are connected (better detection)
- Push Cozmo gently into obstacles to trigger collision detection
- Check logs for accelerometer readings

## Running Multiple Sessions

Each time you run `main.py`:
1. A new session is created
2. Data accumulates in the same database
3. Learned rules persist across sessions
4. The learning system picks up where it left off

## Recommended Test Sequence

| Session | Duration | Goal |
|---------|----------|------|
| 1 | 5-10 min | Verify data collection works |
| 2 | 10-15 min | Accumulate ~20+ recovery events |
| 3 | 15-20 min | Learning cycle should run, rules proposed |
| 4+ | 15-20 min | Rules get tested and validated |

## Data Locations

| Data | Location |
|------|----------|
| SQLite Database | `data/state.db` |
| Spatial Map | `data/spatial_map.npz` |
| Learning Images | `data/learning_images/` |
| Log File | `data/cozmo.log` |
| ChromaDB (experiences) | `data/chroma/` |

## Quick Health Check Script

Save this as `check_status.py` and run it:

```python
import sqlite3
from pathlib import Path

db_path = Path("data/state.db")
if not db_path.exists():
    print("No database found. Run main.py first.")
    exit()

conn = sqlite3.connect(str(db_path))

print("=== LEARNING SYSTEM STATUS ===\n")

# Sensor snapshots
count = conn.execute("SELECT COUNT(*) FROM sensor_snapshots").fetchone()[0]
print(f"Sensor snapshots: {count}")

# Action events
count = conn.execute("SELECT COUNT(*) FROM action_events").fetchone()[0]
print(f"Action events: {count}")

# Actions by type
print("\nActions by type:")
for row in conn.execute("SELECT action_type, COUNT(*) FROM action_events GROUP BY action_type"):
    print(f"  {row[0]}: {row[1]}")

# Outcomes by type
print("\nOutcomes by type:")
for row in conn.execute("SELECT outcome_type, COUNT(*) FROM outcome_events GROUP BY outcome_type"):
    print(f"  {row[0]}: {row[1]}")

# Rules
print("\nLearned rules:")
for row in conn.execute("SELECT name, status, times_applied, times_successful FROM learned_rules"):
    rate = f"{row[3]/row[2]:.0%}" if row[2] > 0 else "n/a"
    print(f"  {row[0]}: {row[1]} (applied: {row[2]}, success: {rate})")

# Images
images_dir = Path("data/learning_images")
if images_dir.exists():
    image_count = len(list(images_dir.glob("*.jpg")))
    print(f"\nLearning images captured: {image_count}")

conn.close()
```

Run with:
```bash
python check_status.py
```
