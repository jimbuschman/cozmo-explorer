"""
Configuration settings for Cozmo Explorer
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
# PHASE CONFIGURATION
# =============================================================================
# Phase 1: Pure Survival Mode
#   - Robot explores and learns micro-rules autonomously
#   - LLM only observes and journals (doesn't set goals)
#   - System works fully without LLM
#
# Phase 2+: LLM-Directed Mode
#   - LLM actively sets goals and directs exploration
#   - Place recognition and room clustering active
# =============================================================================

PHASE1_PURE_SURVIVAL = True  # Set to False for Phase 2+ behavior

# Derived settings based on phase
LLM_GOAL_SETTING_ENABLED = not PHASE1_PURE_SURVIVAL  # LLM sets goals in Phase 2+
LLM_OBSERVER_MODE = PHASE1_PURE_SURVIVAL  # LLM only journals in Phase 1

# =============================================================================
# LLM SETTINGS
# =============================================================================

ENABLE_LLM = True  # Set to False to disable LLM entirely (pure autonomous mode)
OLLAMA_HOST = "http://localhost:11434"
OLLAMA_MODEL = "gemma3:latest"  # Gemma3 has larger context window
EMBEDDING_MODEL = "nomic-embed-text"

# Token budget (Gemma3 supports up to 128k, but we'll use 32k to be safe)
TOKEN_BUDGET = 32000

# Robot settings
COZMO_LOG_LEVEL = "INFO"
ENABLE_VOICE = True  # TTS enabled - audio truncated to 2s max for pycozmo buffer

# Lift-for-camera: raise lift before image capture to clear sensor pod from view.
# Only needed if sensors are mounted in front of the camera. Set False if camera is unobstructed.
LIFT_FOR_CAMERA = True

# Behavior settings
WANDER_SPEED = 70.0  # mm/s (increased for trailer drag)
TURN_SPEED = 30.0    # deg/s
ESCAPE_SPEED = 100.0 # mm/s for escape maneuvers (needs more power to back out with trailer)
CLIFF_THRESHOLD = 10  # mm
LOW_BATTERY_VOLTAGE = 3.4  # volts - warn below this
COLLISION_ACCEL_THRESHOLD = 2500  # Accelerometer delta for collision detection
                                  # Lower = more sensitive (800 too low, triggers on motor noise)
                                  # Higher = only real impacts detected

# LLM query settings
LLM_QUERY_INTERVAL = 30.0  # seconds between "what should I do?" queries (Phase 2+)
LLM_OBSERVER_INTERVAL = 60.0  # seconds between observation-only queries (Phase 1)
LLM_TIMEOUT = 60.0  # seconds to wait for LLM response

# Memory settings
MAX_EXPERIENCES = 1000  # max items in ChromaDB before pruning
EXPERIENCE_RELEVANCE_THRESHOLD = 0.7  # similarity threshold for "seen before"

# External sensor settings (ESP32 pod)
# Mode: "serial" (USB tethered) or "udp" (WiFi wireless)
EXT_SENSOR_MODE = "udp"  # "serial" for USB tethered, "udp" for WiFi wireless

# Serial mode settings
EXT_SENSOR_PORT = "/dev/ttyUSB0"  # Linux default, use "COM3" etc on Windows
EXT_SENSOR_BAUD = 115200

# UDP mode settings (for WiFi)
EXT_SENSOR_UDP_PORT = 5000  # Port to listen on for ESP32 UDP packets

# =============================================================================
# IMU MOUNTING ORIENTATION
# =============================================================================
# Axis sign corrections for MPU6050 based on how the sensor pod is mounted.
# Adjust per-robot if sensor orientation differs.
#
# When the sensor pod is at rest on a flat surface in its mounted position,
# az_g should read ~+1.0g. If it reads ~-1.0g, set IMU_ACCEL_Z_SIGN = -1.0.
# Same logic applies to X and Y axes.
#
# Current defaults: board mounted component-side-down (Z inverted, Y inverted)

IMU_ACCEL_X_SIGN = 1.0
IMU_ACCEL_Y_SIGN = -1.0
IMU_ACCEL_Z_SIGN = -1.0

# =============================================================================
# SENSOR GEOMETRY (per-robot physical layout)
# =============================================================================
# All measurements relative to Cozmo's front-center at ground level.
# X = forward, Y = left, Z = up. Distances in mm, angles in degrees.
#
# Robot dimensions (Cozmo + sensor pod + trailer):
#   Length: ~240mm, Width: ~115mm (at front)
#
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
        "height_mm": 10,         # Height off ground
        "angle_deg": 15,         # Angled 15° outward (left)
        "tilt_deg": 0,           # Slight upward tilt (negligible)
    },
    "ultra_center": {
        "height_mm": 36.5,       # Height off ground
        "angle_deg": 0,          # Pointing straight ahead
        "tilt_deg": 0,
    },
    "ultra_right": {
        "height_mm": 10,         # Height off ground
        "angle_deg": -15,        # Angled 15° outward (right)
        "tilt_deg": 0,
    },
    "tof": {
        "height_mm": 62,         # Height off ground
        "angle_deg": 0,          # Pointing straight ahead
        "tilt_deg": 0,
    },
    "mpu": {
        "distance_from_front_mm": 120,  # ~120mm back from front
    },
}

# =============================================================================
# TRAILER MODE
# =============================================================================
# When enabled, disables in-place turns and uses arc-based turns instead.
# This prevents trailer jackknifing during turns.

TRAILER_MODE = True  # Set True when Cozmo has a trailer attached
TRAILER_ARC_RATIO = 0.5  # Default inner wheel speed ratio (0.5 = 50% of outer)

# Learnable arc ratios - the learning system can propose rules using these
ARC_RATIOS = {
    "gentle": 0.5,   # Wide turn, good for long trailers
    "medium": 0.6,   # Moderate turn
    "tight": 0.7,    # Tighter arc, when space is limited
}

# =============================================================================
# MANUAL CONTROL MODE
# =============================================================================
# Allows direct control of the robot via keyboard for testing and training data.

MANUAL_CONTROL_ENABLED = False  # Start in manual mode if True
