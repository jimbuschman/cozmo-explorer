"""
Configuration settings for Cozmo Explorer

Architecture: Cozmo is a mapping platform. The system (map + LLM + DB) learns.
Cozmo drives around, collects sensor data, builds the map. LLM reviews
completed maps post-session and produces semantic annotations.
"""
from pathlib import Path

# Project paths
PROJECT_ROOT = Path(__file__).parent
DATA_DIR = PROJECT_ROOT / "data"
DATA_DIR.mkdir(exist_ok=True)

# Database paths
SQLITE_PATH = DATA_DIR / "state.db"
CHROMA_PATH = DATA_DIR / "chroma"

# =============================================================================
# LLM SETTINGS
# =============================================================================

ENABLE_LLM = True  # Set to False to disable LLM entirely
OLLAMA_HOST = "http://localhost:11434"
OLLAMA_MODEL = "gemma3:latest"
EMBEDDING_MODEL = "nomic-embed-text"
TOKEN_BUDGET = 32000
LLM_TIMEOUT = 60.0  # seconds to wait for LLM response

# =============================================================================
# ROBOT SETTINGS
# =============================================================================

COZMO_LOG_LEVEL = "INFO"
ENABLE_VOICE = True  # TTS enabled - audio truncated to 2s max for pycozmo buffer

# Lift-for-camera: raise lift before image capture to clear sensor pod from view.
LIFT_FOR_CAMERA = True

# =============================================================================
# MAPPING / NAVIGATION
# =============================================================================

# Drive speeds
WANDER_SPEED = 70.0  # mm/s (increased for trailer drag)
TURN_SPEED = 30.0    # deg/s
ESCAPE_SPEED = 100.0 # mm/s for escape maneuvers

# Escape parameters (deterministic - data shows 135° works at ~100%)
ESCAPE_ANGLE = 135           # degrees
ESCAPE_REVERSE_ARC_DURATION = 3.0  # seconds of reverse-arc escape (backup while turning)

# Safety
LOW_BATTERY_VOLTAGE = 3.4  # volts - warn below this
COLLISION_ACCEL_THRESHOLD = 2500  # Accelerometer delta for collision detection

# Image capture during mapping
IMAGE_CAPTURE_INTERVAL = 120.0  # seconds between captures

# Memory settings
MAX_EXPERIENCES = 1000
EXPERIENCE_RELEVANCE_THRESHOLD = 0.7

# =============================================================================
# EXTERNAL SENSOR SETTINGS (ESP32 pod)
# =============================================================================

EXT_SENSOR_MODE = "udp"  # "serial" for USB tethered, "udp" for WiFi wireless
EXT_SENSOR_PORT = "/dev/ttyUSB0"  # Linux default, use "COM3" etc on Windows
EXT_SENSOR_BAUD = 115200
EXT_SENSOR_UDP_PORT = 5000  # Port to listen on for ESP32 UDP packets

# =============================================================================
# IMU MOUNTING ORIENTATION
# =============================================================================
# Current defaults: board mounted component-side-down (Z inverted, Y inverted)

IMU_ACCEL_X_SIGN = 1.0
IMU_ACCEL_Y_SIGN = -1.0
IMU_ACCEL_Z_SIGN = -1.0

# =============================================================================
# SENSOR GEOMETRY (per-robot physical layout)
# =============================================================================
#            (front)
#       UL ------- UR       Ultrasonics L/R angled 15° outward
#           ToF              ToF center, pointing straight ahead
#           UC               Ultrasonic center, straight ahead
#          [COZMO]
#           MPU              MPU6050 on Cozmo's back
#         [TRAILER]
#            (rear)

SENSOR_GEOMETRY = {
    "ultra_left": {
        "height_mm": 10,
        "angle_deg": 15,
        "tilt_deg": 0,
    },
    "ultra_center": {
        "height_mm": 36.5,
        "angle_deg": 0,
        "tilt_deg": 0,
    },
    "ultra_right": {
        "height_mm": 10,
        "angle_deg": -15,
        "tilt_deg": 0,
    },
    "tof": {
        "height_mm": 62,
        "angle_deg": 0,
        "tilt_deg": 0,
    },
    "mpu": {
        "distance_from_front_mm": 120,
    },
}

# =============================================================================
# TRAILER MODE
# =============================================================================

TRAILER_MODE = True  # Set True when Cozmo has a trailer attached
TRAILER_ARC_RATIO = 0.5  # Default inner wheel speed ratio (0.5 = 50% of outer)

# =============================================================================
# MANUAL CONTROL MODE
# =============================================================================

MANUAL_CONTROL_ENABLED = False
