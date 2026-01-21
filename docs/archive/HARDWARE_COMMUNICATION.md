# Hardware & Communication Architecture

## The Challenge

Each Cozmo robot creates its own WiFi network. To control a Cozmo, your computer must connect to that specific Cozmo's WiFi. This creates challenges for multi-robot setups.

```
Problem:
  PC ──WiFi──> Cozmo_ABC123   ✓ Works
  PC ──WiFi──> Cozmo_XYZ789   ✗ Can't connect to two WiFi networks!
```

---

## Current Setup (Phase 1)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              YOUR PC                                     │
│                                                                          │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────────────────┐ │
│  │  Python App    │  │  Ollama LLM    │  │  SQLite + ChromaDB         │ │
│  │  (main.py)     │  │  (localhost)   │  │  (data/)                   │ │
│  └───────┬────────┘  └────────────────┘  └────────────────────────────┘ │
│          │                                                               │
│          │ pycozmo (UDP)                                                │
│          ▼                                                               │
│  ┌────────────────┐                                                     │
│  │  WiFi Adapter  │───── Cozmo's WiFi Network ─────┐                   │
│  │  (built-in)    │                                 │                   │
│  └────────────────┘                                 │                   │
│                                                     │                   │
│          │ USB Serial                               │                   │
│          ▼                                          │                   │
│  ┌────────────────┐                                 │                   │
│  │  ESP32 Pod     │                                 │                   │
│  │  (COM3)        │                                 │                   │
│  └────────────────┘                                 │                   │
│                                                     │                   │
└─────────────────────────────────────────────────────┼───────────────────┘
                                                      │
                                                      ▼
                                              ┌─────────────┐
                                              │   COZMO     │
                                              │   ROBOT     │
                                              └─────────────┘
```

**Components:**
- 1 PC (Windows/Linux/Mac)
- 1 Cozmo robot
- 1 ESP32 sensor pod (ToF, ultrasonics, IMU)
- PC connects to Cozmo's WiFi (loses internet)
- ESP32 connects via USB serial

---

## Multi-Robot Options

### Option A: Multiple WiFi Adapters (Simplest)

Add USB WiFi adapters to connect to multiple Cozmos simultaneously.

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              YOUR PC                                     │
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                     COORDINATOR                                  │   │
│  │  - Manages all robot connections                                │   │
│  │  - Shared database                                              │   │
│  │  - Task dispatch                                                │   │
│  │  - LLM interface                                                │   │
│  └──────────────────────────┬──────────────────────────────────────┘   │
│                              │                                          │
│         ┌────────────────────┼────────────────────┐                    │
│         │                    │                    │                    │
│         ▼                    ▼                    ▼                    │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐              │
│  │ WiFi Adptr 1│     │ WiFi Adptr 2│     │ WiFi Adptr 3│              │
│  │ (built-in)  │     │ (USB)       │     │ (USB)       │              │
│  └──────┬──────┘     └──────┬──────┘     └──────┬──────┘              │
│         │                   │                   │                      │
└─────────┼───────────────────┼───────────────────┼──────────────────────┘
          │                   │                   │
          ▼                   ▼                   ▼
    Cozmo_AAA            Cozmo_BBB           Cozmo_CCC
    (Scout)              (Camera)            (Mapper)
```

**Pros:**
- Single PC controls everything
- Shared memory/database is easy
- Lowest latency

**Cons:**
- Need 1 WiFi adapter per robot
- Windows/Linux WiFi management can be tricky
- Limited by USB bandwidth if many adapters

**Implementation:**
```python
# config.py
ROBOTS = {
    "scout": {
        "wifi_interface": "wlan0",      # Built-in
        "cozmo_ssid": "Cozmo_AAA123",
        "esp32_port": "COM3",
        "role": "exploration",
        "sensors": ["tof", "ultrasonics", "imu"]
    },
    "camera": {
        "wifi_interface": "wlan1",      # USB adapter 1
        "cozmo_ssid": "Cozmo_BBB456",
        "esp32_port": None,             # No external sensors
        "role": "photography",
        "sensors": ["camera"]
    },
    "mapper": {
        "wifi_interface": "wlan2",      # USB adapter 2
        "cozmo_ssid": "Cozmo_CCC789",
        "esp32_port": "COM4",
        "role": "mapping",
        "sensors": ["tof", "ultrasonics", "imu", "camera"]
    }
}
```

---

### Option B: Distributed with Central Coordinator

Multiple PCs/Raspberry Pis, each controlling one robot, connected via LAN.

```
                         ┌─────────────────────────┐
                         │    CENTRAL SERVER       │
                         │    (Your Main PC)       │
                         │                         │
                         │  - Shared Database      │
                         │  - Task Queue           │
                         │  - LLM (Ollama)         │
                         │  - Coordinator          │
                         └───────────┬─────────────┘
                                     │
                          Home Network / Ethernet
                                     │
          ┌──────────────────────────┼──────────────────────────┐
          │                          │                          │
          ▼                          ▼                          ▼
┌─────────────────┐        ┌─────────────────┐        ┌─────────────────┐
│   ROBOT NODE 1  │        │   ROBOT NODE 2  │        │   ROBOT NODE 3  │
│   (Raspberry Pi)│        │   (Raspberry Pi)│        │   (Laptop)      │
│                 │        │                 │        │                 │
│  - Robot driver │        │  - Robot driver │        │  - Robot driver │
│  - Local buffer │        │  - Local buffer │        │  - Local buffer │
└────────┬────────┘        └────────┬────────┘        └────────┬────────┘
         │                          │                          │
    Cozmo WiFi                 Cozmo WiFi                 Cozmo WiFi
         │                          │                          │
         ▼                          ▼                          ▼
   ┌───────────┐             ┌───────────┐             ┌───────────┐
   │  Cozmo 1  │             │  Cozmo 2  │             │  Cozmo 3  │
   │  Scout    │             │  Camera   │             │  Mapper   │
   └───────────┘             └───────────┘             └───────────┘
```

**Pros:**
- Scales well (add more Pis for more robots)
- Each node is simple
- Central coordinator has full picture
- Can use Ethernet for reliable coordination

**Cons:**
- More hardware (need Pi per robot)
- Network latency for coordination
- More complex deployment

**Communication Protocol:**
```python
# Message types between nodes and coordinator

# Node → Coordinator
{
    "type": "sensor_update",
    "robot_id": "scout",
    "timestamp": "2024-01-15T10:30:00Z",
    "sensors": {...},
    "pose": {"x": 100, "y": 200, "angle": 1.5}
}

{
    "type": "event",
    "robot_id": "scout",
    "event": "collision_detected",
    "details": {...}
}

{
    "type": "task_complete",
    "robot_id": "camera",
    "task_id": "photo_123",
    "result": {"images": ["path1.jpg", "path2.jpg"]}
}

# Coordinator → Node
{
    "type": "task_assign",
    "robot_id": "camera",
    "task_id": "photo_456",
    "task": {
        "action": "go_to_and_photograph",
        "target": {"x": 500, "y": 300},
        "priority": 1
    }
}

{
    "type": "command",
    "robot_id": "scout",
    "command": "return_to_base"
}
```

---

### Option C: Hybrid (Recommended for Your Setup)

Start with Option A (multiple adapters on one PC), design for Option B.

```
Phase 1-2: Single PC, single robot
Phase 3 early: Single PC, multiple WiFi adapters, 2-3 robots
Phase 3 later: Distributed nodes if needed
```

**Code Architecture (Future-Proof):**
```python
# Abstract robot interface - works for local or remote

class RobotInterface(ABC):
    """Abstract interface for controlling a robot"""

    @abstractmethod
    async def connect(self) -> bool:
        pass

    @abstractmethod
    async def get_sensors(self) -> SensorData:
        pass

    @abstractmethod
    async def drive(self, speed: float, duration: float = None):
        pass

    @abstractmethod
    async def turn(self, angle: float):
        pass

    @abstractmethod
    async def capture_image(self) -> Image:
        pass


class LocalCozmoRobot(RobotInterface):
    """Direct pycozmo connection (current implementation)"""
    pass


class RemoteCozmoRobot(RobotInterface):
    """Proxy to a robot controlled by a remote node"""

    def __init__(self, node_url: str, robot_id: str):
        self.node_url = node_url
        self.robot_id = robot_id

    async def drive(self, speed: float, duration: float = None):
        await self._send_command({
            "type": "drive",
            "speed": speed,
            "duration": duration
        })
```

---

## ESP32 Sensor Pod Communication

### Current: USB Serial

```
PC ←──USB Serial──→ ESP32
                      │
                      ├── ToF Sensor (I2C)
                      ├── Ultrasonics x3 (GPIO)
                      └── IMU (I2C)
```

**Baud Rate:** 115200
**Format:** JSON lines
```json
{"ts_ms":12345,"tof_mm":150,"ultra_l_mm":200,"ultra_c_mm":180,"ultra_r_mm":220,"mpu":{...}}
```

### Future: Multiple ESP32 Pods

Each robot could have its own sensor pod:

```
PC ←──USB Hub──┬── ESP32 Pod 1 (Scout's sensors)
               ├── ESP32 Pod 2 (Mapper's sensors)
               └── ESP32 Pod 3 (spare)
```

**Challenge:** Identifying which ESP32 is which.

**Solution:** Each ESP32 has a unique ID in its messages:
```json
{"robot_id":"scout","ts_ms":12345,"tof_mm":150,...}
```

Or use USB serial port mapping:
```python
# config.py
ESP32_MAPPING = {
    "COM3": "scout",   # Or /dev/ttyUSB0 on Linux
    "COM4": "mapper",
    "COM5": "spare"
}
```

---

## Network Topology (Phase 3)

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                                    YOUR PC                                           │
│                                                                                      │
│  ┌──────────────────────────────────────────────────────────────────────────────┐  │
│  │                            COORDINATOR                                        │  │
│  │                                                                               │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │  │
│  │  │ Task Queue  │  │ Shared DB   │  │  LLM        │  │ Robot Registry      │ │  │
│  │  │             │  │ (PostgreSQL │  │  (Ollama)   │  │ - scout: connected  │ │  │
│  │  │ - photo@xyz │  │  or SQLite) │  │             │  │ - camera: connected │ │  │
│  │  │ - map@abc   │  │             │  │             │  │ - mapper: charging  │ │  │
│  │  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────────────┘ │  │
│  └──────────────────────────────────────────────────────────────────────────────┘  │
│                                                                                      │
│  ┌─────────────────────┐  ┌─────────────────────┐  ┌─────────────────────┐        │
│  │ ROBOT DRIVER: Scout │  │ ROBOT DRIVER: Camera│  │ ROBOT DRIVER: Mapper│        │
│  │                     │  │                     │  │                     │        │
│  │ WiFi: wlan0         │  │ WiFi: wlan1         │  │ WiFi: wlan2         │        │
│  │ ESP32: COM3         │  │ ESP32: none         │  │ ESP32: COM4         │        │
│  │ Status: exploring   │  │ Status: idle        │  │ Status: charging    │        │
│  └──────────┬──────────┘  └──────────┬──────────┘  └──────────┬──────────┘        │
│             │                        │                        │                    │
└─────────────┼────────────────────────┼────────────────────────┼────────────────────┘
              │                        │                        │
         Cozmo WiFi               Cozmo WiFi               Cozmo WiFi
              │                        │                        │
              ▼                        ▼                        ▼
        ┌───────────┐            ┌───────────┐            ┌───────────┐
        │  Scout    │            │  Camera   │            │  Mapper   │
        │  Cozmo    │            │  Cozmo    │            │  Cozmo    │
        │           │            │           │            │           │
        │ ESP32 pod │            │ (no pod)  │            │ ESP32 pod │
        └───────────┘            └───────────┘            └───────────┘
```

---

## Hardware Shopping List

### Current (Phase 1)
- [x] 1 Cozmo robot
- [x] 1 ESP32 DevKit
- [x] 1 VL53L0X ToF sensor
- [x] 3 HC-SR04 Ultrasonic sensors
- [x] 1 MPU6050 IMU
- [x] Mounting hardware

### Phase 3 Additions
| Item | Quantity | Purpose | Est. Cost |
|------|----------|---------|-----------|
| Additional Cozmo robots | 2-3 | Different roles | $50-100 each (used) |
| USB WiFi adapters | 2-3 | Multi-robot connection | $15-20 each |
| USB Hub (powered) | 1 | Connect all adapters | $20-30 |
| Additional ESP32 kits | 1-2 | More sensor pods | $30-50 each |
| Raspberry Pi 4 (optional) | 2-3 | Distributed nodes | $50-70 each |

---

## Software Architecture (Multi-Robot Ready)

### File: `coordination/robot_manager.py`

```python
"""
Robot Manager - Handles multiple robot connections
"""
from typing import Dict, Optional
from dataclasses import dataclass
from enum import Enum
import asyncio


class RobotRole(Enum):
    SCOUT = "scout"       # Fast exploration, lots of sensors
    CAMERA = "camera"     # Photography specialist
    MAPPER = "mapper"     # Careful, precise movement
    GENERAL = "general"   # No specialization


class RobotStatus(Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    IDLE = "idle"
    BUSY = "busy"
    CHARGING = "charging"
    ERROR = "error"


@dataclass
class RobotConfig:
    robot_id: str
    wifi_interface: str
    cozmo_ssid: str
    esp32_port: Optional[str]
    role: RobotRole
    sensors: list


class RobotManager:
    """Manages connections to multiple robots"""

    def __init__(self, configs: Dict[str, RobotConfig]):
        self.configs = configs
        self.robots: Dict[str, CozmoRobot] = {}
        self.statuses: Dict[str, RobotStatus] = {}

    async def connect_all(self):
        """Connect to all configured robots"""
        tasks = []
        for robot_id, config in self.configs.items():
            tasks.append(self._connect_robot(robot_id, config))
        await asyncio.gather(*tasks, return_exceptions=True)

    async def _connect_robot(self, robot_id: str, config: RobotConfig):
        """Connect to a single robot"""
        self.statuses[robot_id] = RobotStatus.CONNECTING
        try:
            # Set WiFi interface to connect to this Cozmo's network
            await self._configure_wifi(config.wifi_interface, config.cozmo_ssid)

            # Create robot instance
            robot = CozmoRobot()
            await robot.connect()

            # Connect ESP32 if configured
            if config.esp32_port:
                robot.connect_external_sensors(config.esp32_port)

            self.robots[robot_id] = robot
            self.statuses[robot_id] = RobotStatus.IDLE

        except Exception as e:
            self.statuses[robot_id] = RobotStatus.ERROR
            raise

    def get_robot(self, robot_id: str) -> Optional[CozmoRobot]:
        """Get a robot by ID"""
        return self.robots.get(robot_id)

    def get_available_robots(self, role: RobotRole = None) -> list:
        """Get robots that are idle and ready for tasks"""
        available = []
        for robot_id, status in self.statuses.items():
            if status == RobotStatus.IDLE:
                if role is None or self.configs[robot_id].role == role:
                    available.append(robot_id)
        return available

    def get_status_summary(self) -> dict:
        """Get status of all robots"""
        return {
            robot_id: {
                "status": status.value,
                "role": self.configs[robot_id].role.value,
                "sensors": self.configs[robot_id].sensors
            }
            for robot_id, status in self.statuses.items()
        }
```

### File: `coordination/task_dispatcher.py`

```python
"""
Task Dispatcher - Assigns tasks to appropriate robots
"""
from dataclasses import dataclass
from typing import Optional, List
from datetime import datetime
from enum import Enum
import asyncio


class TaskType(Enum):
    EXPLORE = "explore"
    PHOTOGRAPH = "photograph"
    MAP_AREA = "map_area"
    GO_TO = "go_to"
    RETURN_BASE = "return_base"


class TaskStatus(Enum):
    PENDING = "pending"
    ASSIGNED = "assigned"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass
class Task:
    task_id: str
    task_type: TaskType
    parameters: dict
    priority: int = 1
    preferred_role: Optional[RobotRole] = None
    assigned_robot: Optional[str] = None
    status: TaskStatus = TaskStatus.PENDING
    created_at: datetime = None
    completed_at: datetime = None
    result: dict = None


class TaskDispatcher:
    """Dispatches tasks to robots based on role and availability"""

    def __init__(self, robot_manager: RobotManager):
        self.robot_manager = robot_manager
        self.task_queue: List[Task] = []
        self.active_tasks: Dict[str, Task] = {}  # robot_id -> task

    def add_task(self, task: Task):
        """Add task to queue"""
        task.created_at = datetime.now()
        self.task_queue.append(task)
        self.task_queue.sort(key=lambda t: t.priority)

    async def dispatch_pending(self):
        """Assign pending tasks to available robots"""
        for task in list(self.task_queue):
            if task.status != TaskStatus.PENDING:
                continue

            # Find best robot for this task
            robot_id = self._find_best_robot(task)
            if robot_id:
                await self._assign_task(task, robot_id)

    def _find_best_robot(self, task: Task) -> Optional[str]:
        """Find the best available robot for a task"""
        # Get available robots
        available = self.robot_manager.get_available_robots()

        if not available:
            return None

        # Prefer robots with matching role
        if task.preferred_role:
            for robot_id in available:
                if self.robot_manager.configs[robot_id].role == task.preferred_role:
                    return robot_id

        # Otherwise, use any available robot
        return available[0]

    async def _assign_task(self, task: Task, robot_id: str):
        """Assign a task to a robot"""
        task.assigned_robot = robot_id
        task.status = TaskStatus.ASSIGNED
        self.task_queue.remove(task)
        self.active_tasks[robot_id] = task

        # Send task to robot
        robot = self.robot_manager.get_robot(robot_id)
        # ... execute task
```

---

## WiFi Management (Platform-Specific)

### Windows

```python
import subprocess

def connect_wifi_windows(interface: str, ssid: str, password: str = None):
    """Connect a specific WiFi adapter to a network (Windows)"""
    # Create profile XML
    profile = f"""
    <WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
        <name>{ssid}</name>
        <SSIDConfig>
            <SSID>
                <name>{ssid}</name>
            </SSID>
        </SSIDConfig>
        <connectionType>ESS</connectionType>
        <connectionMode>manual</connectionMode>
        <MSM>
            <security>
                <authEncryption>
                    <authentication>open</authentication>
                    <encryption>none</encryption>
                </authEncryption>
            </security>
        </MSM>
    </WLANProfile>
    """

    # Add profile
    with open("temp_profile.xml", "w") as f:
        f.write(profile)

    subprocess.run(["netsh", "wlan", "add", "profile",
                   f"filename=temp_profile.xml", f"interface={interface}"])

    # Connect
    subprocess.run(["netsh", "wlan", "connect",
                   f"name={ssid}", f"interface={interface}"])
```

### Linux

```python
import subprocess

def connect_wifi_linux(interface: str, ssid: str):
    """Connect a specific WiFi adapter to a network (Linux)"""
    # Using nmcli (NetworkManager)
    subprocess.run([
        "nmcli", "device", "wifi", "connect", ssid,
        "ifname", interface
    ])
```

---

## Data Synchronization

### Shared Database Options

**Option 1: SQLite with file locking (Phase 1-2)**
- Simple, no setup
- Works for single PC
- May have issues with heavy concurrency

**Option 2: PostgreSQL (Phase 3)**
- Better concurrency
- Network accessible
- More setup required

**Option 3: Redis for real-time + PostgreSQL for persistence**
- Best of both worlds
- Redis for fast sensor updates
- PostgreSQL for long-term storage

### Example: Shared Experience Database

```python
# All robots write to the same database
# Each entry tagged with robot_id

class SharedExperienceLogger(ExperienceLogger):
    """Experience logger that tags data with robot ID"""

    def __init__(self, db_path: str, robot_id: str):
        super().__init__(db_path)
        self.robot_id = robot_id

    def log_sensor_snapshot(self, sensors: dict, **kwargs) -> int:
        """Log sensor snapshot with robot ID"""
        sensors['robot_id'] = self.robot_id
        return super().log_sensor_snapshot(sensors, **kwargs)
```

---

## Summary

| Phase | Robots | Hardware | Communication |
|-------|--------|----------|---------------|
| 1 | 1 | PC + Cozmo + ESP32 | Direct pycozmo |
| 2 | 1 | Same | Same |
| 3 early | 2-3 | PC + USB WiFi adapters | Multi-pycozmo |
| 3 later | 3+ | PC + Pis (optional) | Distributed + shared DB |

**Key Design Decisions:**
1. Abstract robot interface for future flexibility
2. Tag all data with robot_id
3. Centralized coordinator, distributed drivers
4. Design for multi-robot even if running single robot now
