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

# Behavior settings
WANDER_SPEED = 50.0  # mm/s
TURN_SPEED = 30.0    # deg/s
CLIFF_THRESHOLD = 10  # mm
LOW_BATTERY_VOLTAGE = 3.4  # volts - warn below this

# LLM query settings
LLM_QUERY_INTERVAL = 30.0  # seconds between "what should I do?" queries (Phase 2+)
LLM_OBSERVER_INTERVAL = 60.0  # seconds between observation-only queries (Phase 1)
LLM_TIMEOUT = 60.0  # seconds to wait for LLM response

# Memory settings
MAX_EXPERIENCES = 1000  # max items in ChromaDB before pruning
EXPERIENCE_RELEVANCE_THRESHOLD = 0.7  # similarity threshold for "seen before"

# External sensor settings (ESP32 pod)
EXT_SENSOR_PORT = "/dev/ttyUSB0"  # Linux default, use "COM3" etc on Windows
EXT_SENSOR_BAUD = 115200
