# Phase 2: Place & Context Learning - Component Design

This document sketches the new components needed for Phase 2.

## Overview

Phase 2 answers: "Have I been here before?" and "What rules work here?"

### New Components
1. **PlaceSignature** - Recognizable location fingerprints
2. **PlaceRecognition** - Match current location to known places
3. **RoomCluster** - Group places into "rooms"
4. **ScopedRules** - Rules that apply to specific locations
5. **SemanticMap** - Meaningful map with regions and transitions

---

## Component 1: Place Signatures

### File: `memory/place_signature.py`

```python
"""
Place Signatures - Recognizable location fingerprints

A place signature captures what a location "feels like" based on:
- Sensor patterns (distances, IMU readings)
- Visual features (image embeddings)
- Audio patterns (future)
- Temporal patterns (how sensors change as you approach)
"""
from dataclasses import dataclass, field
from typing import List, Dict, Optional
from datetime import datetime
import numpy as np

@dataclass
class PlaceSignature:
    """A recognizable location pattern"""

    # Identity
    signature_id: Optional[int] = None
    name: str = ""  # LLM-assigned or auto-generated

    # Location (approximate - from odometry)
    center_x: float = 0.0
    center_y: float = 0.0
    radius: float = 50.0  # mm, how "big" is this place

    # Sensor signature (what sensors typically read here)
    sensor_pattern: Dict = field(default_factory=dict)
    # Example: {
    #   "ext_tof_mm": {"mean": 250, "std": 30},
    #   "ext_ultra_l_mm": {"mean": 180, "std": 25},
    #   "ext_ultra_r_mm": {"mean": 400, "std": 50},
    # }

    # Visual signature (embeddings from images taken here)
    visual_embeddings: List[np.ndarray] = field(default_factory=list)

    # Approach signatures (sensor patterns when entering from different directions)
    approach_patterns: Dict = field(default_factory=dict)
    # Example: {"north": [...], "east": [...]}

    # Statistics
    times_visited: int = 0
    last_visited: Optional[datetime] = None
    confidence: float = 0.0  # How sure are we this is a real place?

    # Linked data
    linked_rules: List[int] = field(default_factory=list)  # Rules that work here
    room_id: Optional[int] = None  # Which room cluster this belongs to
    images: List[str] = field(default_factory=list)  # Image paths

    def matches(self, sensor_reading: Dict, threshold: float = 0.7) -> float:
        """
        Calculate how well a sensor reading matches this signature.
        Returns confidence score 0.0 - 1.0
        """
        if not self.sensor_pattern:
            return 0.0

        matches = []
        for sensor, pattern in self.sensor_pattern.items():
            if sensor in sensor_reading:
                value = sensor_reading[sensor]
                mean = pattern.get('mean', 0)
                std = pattern.get('std', 1)

                if std > 0:
                    # How many standard deviations away?
                    z_score = abs(value - mean) / std
                    # Convert to 0-1 score (z=0 -> 1.0, z=2 -> ~0.1)
                    score = np.exp(-0.5 * z_score ** 2)
                    matches.append(score)

        return np.mean(matches) if matches else 0.0

    def update_with_reading(self, sensor_reading: Dict):
        """Update signature with new sensor reading (online learning)"""
        for sensor, value in sensor_reading.items():
            if sensor not in self.sensor_pattern:
                self.sensor_pattern[sensor] = {
                    'mean': value,
                    'std': 0.0,
                    'n': 1
                }
            else:
                # Running mean and std update
                pattern = self.sensor_pattern[sensor]
                n = pattern.get('n', 1)
                old_mean = pattern['mean']

                # Update mean
                new_mean = old_mean + (value - old_mean) / (n + 1)

                # Update std (Welford's algorithm)
                if n > 1:
                    old_std = pattern['std']
                    new_std = np.sqrt(
                        ((n - 1) * old_std ** 2 + (value - old_mean) * (value - new_mean)) / n
                    )
                    pattern['std'] = new_std

                pattern['mean'] = new_mean
                pattern['n'] = n + 1

        self.times_visited += 1
        self.last_visited = datetime.now()


class PlaceSignatureStore:
    """SQLite storage for place signatures"""

    def __init__(self, db_path: str):
        self.db_path = db_path
        self._init_tables()

    def _init_tables(self):
        """Create tables for place signatures"""
        # Implementation: CREATE TABLE place_signatures (...)
        pass

    def add_signature(self, sig: PlaceSignature) -> int:
        """Add new signature, return ID"""
        pass

    def find_matching(self, sensor_reading: Dict, threshold: float = 0.6) -> List[PlaceSignature]:
        """Find signatures that match current reading"""
        pass

    def get_by_location(self, x: float, y: float, radius: float = 100) -> List[PlaceSignature]:
        """Find signatures near a location"""
        pass

    def get_by_room(self, room_id: int) -> List[PlaceSignature]:
        """Get all signatures in a room"""
        pass
```

---

## Component 2: Place Recognition

### File: `memory/place_recognition.py`

```python
"""
Place Recognition - "Have I been here before?"

Uses place signatures to recognize current location.
"""
import logging
from typing import Optional, Tuple, List
from dataclasses import dataclass
from datetime import datetime

from memory.place_signature import PlaceSignature, PlaceSignatureStore

logger = logging.getLogger(__name__)


@dataclass
class RecognitionResult:
    """Result of place recognition"""
    recognized: bool
    signature: Optional[PlaceSignature]
    confidence: float
    is_new_place: bool
    nearby_places: List[PlaceSignature]


class PlaceRecognizer:
    """
    Recognizes places based on sensor patterns.

    Flow:
    1. Get current sensor reading
    2. Compare to known signatures
    3. If match found -> return that place
    4. If no match -> potentially create new signature
    """

    # Thresholds
    RECOGNITION_THRESHOLD = 0.7  # Confidence needed to recognize
    NEW_PLACE_THRESHOLD = 0.3   # Below this, definitely new place
    MIN_VISITS_FOR_CONFIDENCE = 5  # Visits before signature is "trusted"

    def __init__(self, signature_store: PlaceSignatureStore):
        self.store = signature_store
        self._current_place: Optional[PlaceSignature] = None
        self._place_entry_time: Optional[datetime] = None

    def recognize(self, sensor_reading: dict, pose: Tuple[float, float]) -> RecognitionResult:
        """
        Attempt to recognize current location.

        Args:
            sensor_reading: Current sensor values
            pose: Current (x, y) position estimate

        Returns:
            RecognitionResult with match info
        """
        # Get candidate signatures (nearby + high-confidence ones)
        candidates = self._get_candidates(pose)

        if not candidates:
            return RecognitionResult(
                recognized=False,
                signature=None,
                confidence=0.0,
                is_new_place=True,
                nearby_places=[]
            )

        # Score each candidate
        scores = []
        for sig in candidates:
            score = sig.matches(sensor_reading)
            scores.append((sig, score))

        # Sort by score
        scores.sort(key=lambda x: x[1], reverse=True)
        best_sig, best_score = scores[0]

        # Decide if recognized
        if best_score >= self.RECOGNITION_THRESHOLD:
            # Update the signature with new reading
            best_sig.update_with_reading(sensor_reading)
            self._current_place = best_sig

            return RecognitionResult(
                recognized=True,
                signature=best_sig,
                confidence=best_score,
                is_new_place=False,
                nearby_places=[s for s, _ in scores[1:5]]
            )

        elif best_score <= self.NEW_PLACE_THRESHOLD:
            # Definitely a new place
            return RecognitionResult(
                recognized=False,
                signature=None,
                confidence=best_score,
                is_new_place=True,
                nearby_places=[s for s, _ in scores[:5]]
            )

        else:
            # Uncertain - might be new, might be known but different angle
            return RecognitionResult(
                recognized=False,
                signature=best_sig,  # Best guess
                confidence=best_score,
                is_new_place=False,  # Not confident enough to create new
                nearby_places=[s for s, _ in scores[:5]]
            )

    def create_new_signature(
        self,
        sensor_reading: dict,
        pose: Tuple[float, float],
        image_path: Optional[str] = None
    ) -> PlaceSignature:
        """Create a new place signature at current location"""
        sig = PlaceSignature(
            center_x=pose[0],
            center_y=pose[1],
            times_visited=1,
            last_visited=datetime.now(),
            confidence=0.1  # Low initial confidence
        )

        sig.update_with_reading(sensor_reading)

        if image_path:
            sig.images.append(image_path)

        sig_id = self.store.add_signature(sig)
        sig.signature_id = sig_id

        logger.info(f"Created new place signature {sig_id} at ({pose[0]:.0f}, {pose[1]:.0f})")
        return sig

    def _get_candidates(self, pose: Tuple[float, float]) -> List[PlaceSignature]:
        """Get candidate signatures for matching"""
        # Get nearby signatures (within 500mm)
        nearby = self.store.get_by_location(pose[0], pose[1], radius=500)

        # Also include high-confidence signatures (might recognize from far)
        # This handles cases where odometry has drifted

        return nearby

    def get_current_place(self) -> Optional[PlaceSignature]:
        """Get the currently recognized place"""
        return self._current_place
```

---

## Component 3: Room Clustering

### File: `memory/room_clustering.py`

```python
"""
Room Clustering - Group places into "rooms"

A room is a collection of nearby, similar places that form a coherent space.
"""
from dataclasses import dataclass, field
from typing import List, Dict, Optional
from datetime import datetime

from memory.place_signature import PlaceSignature


@dataclass
class RoomCluster:
    """A collection of places that form a 'room'"""

    room_id: Optional[int] = None
    name: str = ""  # "kitchen", "hallway", etc. (LLM-assigned)

    # Places in this room
    place_ids: List[int] = field(default_factory=list)

    # Boundary places (transitions to other rooms)
    boundary_place_ids: List[int] = field(default_factory=list)

    # Room characteristics (aggregated from places)
    typical_distances: Dict = field(default_factory=dict)
    floor_type: str = ""  # "carpet", "hardwood", etc.
    lighting: str = ""    # "bright", "dim", etc.

    # Navigation info
    connected_rooms: List[int] = field(default_factory=list)
    navigation_rules: List[int] = field(default_factory=list)

    # Statistics
    times_entered: int = 0
    total_time_spent: float = 0.0  # seconds
    last_visited: Optional[datetime] = None

    # LLM observations
    observations: List[str] = field(default_factory=list)


class RoomClusterer:
    """
    Clusters places into rooms.

    Approach:
    1. Use spatial proximity (places near each other)
    2. Use sensor similarity (places that "feel" similar)
    3. Detect boundaries (places where sensor patterns change sharply)
    """

    def __init__(self, place_store, room_store):
        self.place_store = place_store
        self.room_store = room_store

    def cluster_places(self) -> List[RoomCluster]:
        """
        Run clustering on all places to identify rooms.

        Called periodically or when enough new places exist.
        """
        # Get all place signatures
        places = self.place_store.get_all()

        if len(places) < 5:
            return []  # Not enough data

        # Step 1: Spatial clustering (DBSCAN or similar)
        spatial_clusters = self._spatial_cluster(places)

        # Step 2: Refine with sensor similarity
        refined_clusters = self._refine_with_sensors(spatial_clusters)

        # Step 3: Identify boundaries
        for cluster in refined_clusters:
            self._identify_boundaries(cluster)

        # Step 4: Store/update rooms
        rooms = []
        for cluster in refined_clusters:
            room = self._cluster_to_room(cluster)
            rooms.append(room)

        return rooms

    def _spatial_cluster(self, places: List[PlaceSignature]) -> List[List[PlaceSignature]]:
        """Cluster places by spatial proximity"""
        # Use DBSCAN or simple distance-based clustering
        # Places within ~1m of each other likely same room
        pass

    def _refine_with_sensors(self, clusters) -> List[List[PlaceSignature]]:
        """Split clusters if sensor patterns are very different"""
        # Two places might be spatially close but feel different
        # (e.g., doorway between rooms)
        pass

    def _identify_boundaries(self, cluster: List[PlaceSignature]):
        """Find places at the edge of a room"""
        # Boundary places have:
        # - High sensor variance
        # - Neighbors in different clusters
        # - Sharp transitions in sensor readings
        pass

    def _cluster_to_room(self, cluster: List[PlaceSignature]) -> RoomCluster:
        """Convert a cluster of places to a RoomCluster"""
        pass

    def detect_room_transition(
        self,
        prev_place: PlaceSignature,
        curr_place: PlaceSignature
    ) -> Optional[Tuple[int, int]]:
        """
        Detect if robot has moved between rooms.

        Returns (from_room_id, to_room_id) if transition detected.
        """
        if prev_place.room_id != curr_place.room_id:
            return (prev_place.room_id, curr_place.room_id)
        return None
```

---

## Component 4: Scoped Rules

### Modifications to existing `memory/learned_rules.py`

The current schema already has `scope` and `location_id` fields. Phase 2 activates them:

```python
# In LearnedRulesStore

def apply_rules_to_action(
    self,
    base_action: dict,
    sensor_context: dict,
    location_id: Optional[int] = None,  # NEW: Current place/room
    room_id: Optional[int] = None       # NEW: Current room cluster
) -> Tuple[dict, Optional[int]]:
    """
    Apply matching rules to modify an action.

    Priority order:
    1. Location-specific rules (exact place match)
    2. Room-specific rules (same room)
    3. Global rules
    """
    # Get candidate rules
    location_rules = self.get_rules_for_location(location_id) if location_id else []
    room_rules = self.get_rules_for_room(room_id) if room_id else []
    global_rules = self.get_global_rules()

    # Find matching rules at each scope
    for rules in [location_rules, room_rules, global_rules]:
        matching = [r for r in rules if self._rule_matches(r, sensor_context)]
        if matching:
            # Use the best matching rule at this scope
            best = self._select_best_rule(matching)
            return self._apply_rule(base_action, best), best.id

    return base_action, None

def get_rules_for_location(self, location_id: int) -> List[LearnedRule]:
    """Get rules scoped to a specific place"""
    # SELECT * FROM learned_rules WHERE scope='location' AND location_id=?
    pass

def get_rules_for_room(self, room_id: int) -> List[LearnedRule]:
    """Get rules scoped to a room"""
    # SELECT * FROM learned_rules WHERE scope='room' AND room_id=?
    pass

def get_global_rules(self) -> List[LearnedRule]:
    """Get rules that apply everywhere"""
    # SELECT * FROM learned_rules WHERE scope='global'
    pass
```

---

## Component 5: Integration with State Machine

### File: `brain/state_machine.py` (Phase 2 additions)

```python
# Add to __init__
self.place_recognizer: Optional[PlaceRecognizer] = None
self.room_clusterer: Optional[RoomClusterer] = None
self.current_place: Optional[PlaceSignature] = None
self.current_room: Optional[RoomCluster] = None

# Add new method
async def _update_location_context(self):
    """Update place/room recognition (Phase 2)"""
    if not self.place_recognizer:
        return

    # Get current sensor reading
    sensor_reading = self._get_sensor_context()
    pose = (self.robot.pose.x, self.robot.pose.y)

    # Attempt recognition
    result = self.place_recognizer.recognize(sensor_reading, pose)

    if result.recognized:
        self.current_place = result.signature
        logger.debug(f"Recognized place: {result.signature.name or result.signature.signature_id}")

        # Check for room transition
        if self.current_place.room_id != self.current_room?.room_id:
            old_room = self.current_room
            self.current_room = self.room_clusterer.get_room(self.current_place.room_id)
            if old_room:
                logger.info(f"Room transition: {old_room.name} -> {self.current_room.name}")

    elif result.is_new_place:
        # Create new place signature
        image_path = await self._capture_place_image()
        new_sig = self.place_recognizer.create_new_signature(
            sensor_reading, pose, image_path
        )
        self.current_place = new_sig
        logger.info(f"Discovered new place at ({pose[0]:.0f}, {pose[1]:.0f})")

# Modify _execute_state to include location context
async def _execute_state(self):
    """Execute behavior for current state"""
    # Update location context (Phase 2)
    await self._update_location_context()

    # ... rest of existing code
```

---

## Data Flow (Phase 2)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          SENSORS                                         │
│  Cozmo internal + ESP32 external + Camera                               │
└───────────────────────────────────┬─────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      PLACE RECOGNITION                                   │
│  "Is this a known place?" → Match against signatures                    │
│  If new → Create signature                                              │
│  If known → Update signature, get linked rules                          │
└───────────────────────────────────┬─────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      ROOM CLUSTERING                                     │
│  Group signatures into rooms                                            │
│  Detect transitions                                                     │
│  Aggregate room-level patterns                                          │
└───────────────────────────────────┬─────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      SCOPED RULE APPLICATION                            │
│  1. Location-specific rules (this exact spot)                           │
│  2. Room-specific rules (this room)                                     │
│  3. Global rules (everywhere)                                           │
└───────────────────────────────────┬─────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      BEHAVIOR EXECUTION                                  │
│  Apply selected rules to modify actions                                 │
│  Log outcomes with location context                                     │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## LLM Integration (Phase 2)

### New Prompt: Room Labeling

```python
ROOM_LABELING_PROMPT = """
Based on the following observations, suggest a name for this room/area.

Place signatures in this cluster:
{place_summaries}

Typical sensor readings:
- Average front distance: {avg_front_dist}mm
- Average left distance: {avg_left_dist}mm
- Average right distance: {avg_right_dist}mm

Visual observations:
{visual_descriptions}

Suggest:
1. A short name for this room (e.g., "kitchen", "hallway", "living room corner")
2. Key characteristics that define this space
3. Any hypotheses about what's in this room

Respond in JSON:
{
  "name": "...",
  "characteristics": ["...", "..."],
  "hypotheses": ["...", "..."]
}
"""
```

### New Prompt: Rule Transfer Hypothesis

```python
RULE_TRANSFER_PROMPT = """
I'm in a new room that feels similar to a room I've been in before.

Previous room: {old_room_name}
- Characteristics: {old_characteristics}
- Rules that worked well: {old_rules}

New room observations:
- Sensor patterns: {new_sensors}
- Similarities to old room: {similarities}
- Differences: {differences}

Should I try the rules from {old_room_name} in this new space?
Which rules are most likely to transfer?
Which might need adjustment?

Respond in JSON:
{
  "transfer_recommended": true/false,
  "rules_to_try": [rule_ids],
  "rules_to_avoid": [rule_ids],
  "reasoning": "..."
}
"""
```

---

## Success Metrics (Phase 2)

| Metric | Target | How to Measure |
|--------|--------|----------------|
| Place recognition accuracy | >70% | Test with known locations |
| False positive rate | <20% | New places incorrectly matched |
| Room clustering quality | Subjective | Manual review of clusters |
| Scoped rule improvement | >10% over global | Compare success rates |
| Rule transfer success | >50% | When applying old rules to new rooms |
