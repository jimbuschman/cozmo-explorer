"""
Cozmo Personality & Journal

Gives Cozmo an alien explorer personality and maintains an exploration journal.
Think: a tiny alien scientist sent to study Earth from ground level.
"""
import logging
import json
from typing import Optional, List
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import config

logger = logging.getLogger(__name__)


@dataclass
class JournalEntry:
    """A single journal entry"""
    timestamp: datetime
    entry_type: str  # "observation", "discovery", "thought", "concern", "milestone"
    title: str
    content: str
    location: Optional[tuple] = None  # (x, y)
    mood: str = "curious"  # curious, excited, confused, concerned, amazed, tired
    importance: int = 1  # 1-5


class CozmoPersonality:
    """
    Manages Cozmo's identity and exploration journal.

    Keeps a persistent journal of observations, discoveries, and thoughts.
    The identity prompt can be customized later for different personalities.
    """

    # The core identity prompt that shapes all responses
    # Keep it simple for now - can add personality flavor later
    IDENTITY = """You are Cozmo, a small autonomous robot exploring indoor environments.

YOUR CAPABILITIES:
- You can drive, turn, and look around
- You have a camera to see your surroundings
- You're about 10cm tall, so you see things from ground level
- You can detect cliffs/edges and avoid them

YOUR TASK:
- Explore the environment and document what you find
- Make decisions about where to go next
- Remember what you've seen for future reference

GUIDELINES:
- Be curious and thorough in exploration
- Note interesting objects or areas
- Avoid obstacles and dangerous edges
- Keep track of where you've been and haven't been

Respond with JSON when making decisions:
{
    "action": "explore|investigate|go_to|idle",
    "target": "optional target description",
    "reasoning": "why this action",
    "speak": "optional short phrase to say"
}
"""

    def __init__(self, journal_path: Optional[Path] = None):
        self.journal_path = journal_path or (config.DATA_DIR / "exploration_journal.json")
        self.journal: List[JournalEntry] = []
        self._current_mood = "curious"
        self._discoveries_today = 0

        # Load existing journal
        self._load_journal()

    def _load_journal(self):
        """Load journal from disk"""
        if self.journal_path.exists():
            try:
                with open(self.journal_path, 'r') as f:
                    data = json.load(f)
                    self.journal = [
                        JournalEntry(
                            timestamp=datetime.fromisoformat(e['timestamp']),
                            entry_type=e['entry_type'],
                            title=e['title'],
                            content=e['content'],
                            location=tuple(e['location']) if e.get('location') else None,
                            mood=e.get('mood', 'curious'),
                            importance=e.get('importance', 1)
                        )
                        for e in data
                    ]
                logger.info(f"Loaded {len(self.journal)} journal entries")
            except Exception as e:
                logger.error(f"Failed to load journal: {e}")

    def _save_journal(self):
        """Save journal to disk"""
        try:
            self.journal_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.journal_path, 'w') as f:
                json.dump([
                    {
                        'timestamp': e.timestamp.isoformat(),
                        'entry_type': e.entry_type,
                        'title': e.title,
                        'content': e.content,
                        'location': list(e.location) if e.location else None,
                        'mood': e.mood,
                        'importance': e.importance
                    }
                    for e in self.journal
                ], f, indent=2)
        except Exception as e:
            logger.error(f"Failed to save journal: {e}")

    def add_entry(
        self,
        entry_type: str,
        title: str,
        content: str,
        location: tuple = None,
        importance: int = 1
    ):
        """Add a new journal entry"""
        entry = JournalEntry(
            timestamp=datetime.now(),
            entry_type=entry_type,
            title=title,
            content=content,
            location=location,
            mood=self._current_mood,
            importance=importance
        )
        self.journal.append(entry)
        self._save_journal()

        if entry_type == "discovery":
            self._discoveries_today += 1

        logger.info(f"Journal: [{entry_type}] {title}")

    def log_observation(self, description: str, location: tuple = None):
        """Log a visual observation"""
        self.add_entry(
            entry_type="observation",
            title="Visual scan",
            content=description,
            location=location
        )

    def log_discovery(self, what: str, details: str, location: tuple = None):
        """Log a new discovery"""
        self._current_mood = "excited"
        self.add_entry(
            entry_type="discovery",
            title=f"Discovered: {what}",
            content=details,
            location=location,
            importance=3
        )

    def log_thought(self, thought: str):
        """Log a thought or musing"""
        self.add_entry(
            entry_type="thought",
            title="Personal reflection",
            content=thought,
            importance=1
        )

    def log_concern(self, concern: str, location: tuple = None):
        """Log a concern or warning"""
        self._current_mood = "concerned"
        self.add_entry(
            entry_type="concern",
            title="Caution noted",
            content=concern,
            location=location,
            importance=2
        )

    def log_milestone(self, milestone: str, details: str):
        """Log a significant milestone"""
        self._current_mood = "excited"
        self.add_entry(
            entry_type="milestone",
            title=milestone,
            content=details,
            importance=5
        )

    def get_recent_entries(self, count: int = 5) -> List[JournalEntry]:
        """Get most recent journal entries"""
        return self.journal[-count:]

    def get_entries_by_type(self, entry_type: str, count: int = 10) -> List[JournalEntry]:
        """Get entries of a specific type"""
        filtered = [e for e in self.journal if e.entry_type == entry_type]
        return filtered[-count:]

    def get_session_summary(self) -> str:
        """Generate a summary of the current session for the LLM context"""
        if not self.journal:
            return "No journal entries yet. Beginning expedition."

        recent = self.get_recent_entries(3)
        discoveries = len([e for e in self.journal if e.entry_type == "discovery"])

        lines = [
            f"Expedition Log Summary:",
            f"- Total entries: {len(self.journal)}",
            f"- Discoveries: {discoveries}",
            f"- Current mood: {self._current_mood}",
            "",
            "Recent log entries:"
        ]

        for entry in recent:
            time_str = entry.timestamp.strftime("%H:%M")
            lines.append(f"  [{time_str}] {entry.title}: {entry.content[:80]}...")

        return "\n".join(lines)

    def set_mood(self, mood: str):
        """Update current mood"""
        valid_moods = ["curious", "excited", "confused", "concerned", "amazed", "tired"]
        if mood in valid_moods:
            self._current_mood = mood

    def get_identity_prompt(self) -> str:
        """Get the full identity prompt for LLM"""
        return self.IDENTITY

    def get_mission_status(self) -> str:
        """Get a mission status report"""
        session_entries = [e for e in self.journal if e.timestamp.date() == datetime.now().date()]

        return f"""
Mission Status Report:
- Entries today: {len(session_entries)}
- Total discoveries: {len([e for e in self.journal if e.entry_type == 'discovery'])}
- Current mood: {self._current_mood}
- Journal entries: {len(self.journal)} total
"""


# Pre-written lines Cozmo can say in different situations
PERSONALITY_LINES = {
    "startup": [
        "Starting up. Ready to explore.",
        "Systems online.",
        "Let's see what's around.",
    ],
    "exploring": [
        "Looking around.",
        "Exploring the area.",
        "Checking this out.",
    ],
    "discovery": [
        "Found something interesting.",
        "That's new.",
        "Noting this down.",
    ],
    "cliff_warning": [
        "Edge detected, backing up.",
        "Cliff ahead, turning around.",
        "That's a drop, avoiding it.",
    ],
    "stuck": [
        "Can't get through here.",
        "Trying another way.",
        "Obstacle detected.",
    ],
    "low_battery": [
        "Battery getting low.",
        "Need to charge soon.",
        "Running low on power.",
    ],
    "found_something": [
        "What's this?",
        "Something here.",
        "Taking a closer look.",
    ],
    "confused": [
        "Not sure what that is.",
        "Hmm, interesting.",
        "Can't identify this.",
    ],
    "happy": [
        "This is nice.",
        "Good spot.",
        "Like this area.",
    ],
}


def get_random_line(situation: str) -> str:
    """Get a random personality line for a situation"""
    import random
    lines = PERSONALITY_LINES.get(situation, ["Hmm..."])
    return random.choice(lines)
