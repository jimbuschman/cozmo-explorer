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

# LLM settings
OLLAMA_HOST = "http://localhost:11434"
OLLAMA_MODEL = "llama3.2:latest"  # Change to your preferred model
EMBEDDING_MODEL = "nomic-embed-text"

# Robot settings
COZMO_LOG_LEVEL = "INFO"

# Behavior settings
WANDER_SPEED = 50.0  # mm/s
TURN_SPEED = 30.0    # deg/s
CLIFF_THRESHOLD = 10  # mm
LOW_BATTERY_VOLTAGE = 3.4  # volts - warn below this

# LLM query settings
LLM_QUERY_INTERVAL = 30.0  # seconds between "what should I do?" queries
LLM_TIMEOUT = 60.0  # seconds to wait for LLM response

# Memory settings
MAX_EXPERIENCES = 1000  # max items in ChromaDB before pruning
EXPERIENCE_RELEVANCE_THRESHOLD = 0.7  # similarity threshold for "seen before"
