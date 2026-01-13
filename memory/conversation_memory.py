"""
Conversation Memory Manager

Ported from EchoFrontendV2 C# implementation.
Manages LLM context with token-budgeted memory pools and automatic summarization.
"""
import logging
import asyncio
from typing import Optional, List, Dict, Any, Callable
from dataclasses import dataclass, field
from datetime import datetime
from collections import deque
from enum import Enum

logger = logging.getLogger(__name__)


def estimate_tokens(text: str) -> int:
    """Rough token estimation (chars / 4)"""
    if not text:
        return 0
    return max(1, len(text) // 4)


@dataclass
class MemoryItem:
    """A single item in a memory pool"""
    text: str
    role: str  # "system", "user", "assistant", "observation", "decision"
    timestamp: datetime = field(default_factory=datetime.now)
    priority_score: float = 1.0
    estimated_tokens: int = 0
    pool_name: str = ""
    metadata: Dict[str, Any] = field(default_factory=dict)
    dumped: bool = False  # Flag for items that have been archived/summarized

    def __post_init__(self):
        if self.estimated_tokens == 0:
            self.estimated_tokens = estimate_tokens(self.text)


@dataclass
class PoolConfig:
    """Configuration for a memory pool"""
    percentage: float  # Percentage of global budget
    priority: int  # Rollover priority (higher = gets more overflow tokens)
    hard_cap: Optional[int] = None  # Optional maximum tokens


class MemoryPool:
    """
    A pool of memory items with token budget management.
    """

    def __init__(self, name: str, max_tokens: int, hard_cap: Optional[int] = None):
        self.name = name
        self.max_tokens = max_tokens
        self.hard_cap = hard_cap
        self._items: List[MemoryItem] = []

    @property
    def used_tokens(self) -> int:
        return sum(item.estimated_tokens for item in self._items)

    @property
    def available_tokens(self) -> int:
        return self.max_tokens - self.used_tokens

    def add(self, item: MemoryItem) -> bool:
        """Add an item to the pool. Returns False if duplicate."""
        # Check for duplicates
        if any(existing.text == item.text for existing in self._items):
            return False

        item.pool_name = self.name
        self._items.append(item)
        # Sort by priority (highest first)
        self._items.sort(key=lambda x: x.priority_score, reverse=True)
        return True

    def get_items(self, max_tokens: Optional[int] = None) -> List[MemoryItem]:
        """Get items up to token limit"""
        limit = min(max_tokens or self.max_tokens, self.hard_cap or self.max_tokens)

        selected = []
        used = 0
        for item in self._items:
            if used + item.estimated_tokens <= limit:
                selected.append(item)
                used += item.estimated_tokens
            else:
                break
        return selected

    def get_oldest(self, count: int) -> List[MemoryItem]:
        """Get the N oldest items"""
        sorted_by_time = sorted(self._items, key=lambda x: x.timestamp)
        return sorted_by_time[:count]

    def remove_items(self, items: List[MemoryItem]):
        """Remove specific items from the pool"""
        for item in items:
            if item in self._items:
                self._items.remove(item)

    def clear(self):
        """Clear all items"""
        self._items.clear()


class ConversationMemory:
    """
    Manages LLM conversation context with multiple memory pools.

    Pools for Cozmo:
    - Core: Robot identity, personality, capabilities (static)
    - ActiveSession: Current exploration decisions and observations
    - RecentHistory: Summarized past sessions
    - Recall: Semantic memories from ChromaDB
    - Buffer: Spatial awareness, sensor state, overflow

    When ActiveSession gets too full, oldest items are summarized
    and moved to RecentHistory.
    """

    OVERHEAD_TOKENS = 1000  # Reserve for system prompt, formatting, etc.

    def __init__(
        self,
        global_budget: int = 32000,
        summarizer: Optional[Callable] = None
    ):
        self.global_budget = global_budget
        self.summarizer = summarizer  # Async function to summarize text

        self._pools: Dict[str, MemoryPool] = {}
        self._config: Dict[str, PoolConfig] = {}

        # Default pool configuration for Cozmo
        self._setup_default_pools()

    def _setup_default_pools(self):
        """Set up default memory pools for Cozmo explorer"""
        # Core: Identity and personality (static, small)
        self.configure_pool("Core", percentage=0.10, priority=0, hard_cap=2048)

        # ActiveSession: Current conversation/decisions (largest, highest priority)
        self.configure_pool("ActiveSession", percentage=0.35, priority=3)

        # RecentHistory: Summarized past sessions (medium priority)
        self.configure_pool("RecentHistory", percentage=0.15, priority=2)

        # Recall: Semantic memories from database (large cap for embeddings)
        self.configure_pool("Recall", percentage=0.30, priority=1, hard_cap=8192)

        # Buffer: Overflow and spatial context
        self.configure_pool("Buffer", percentage=0.10, priority=1)

        self.initialize_pools()

    def configure_pool(
        self,
        name: str,
        percentage: float,
        priority: int,
        hard_cap: Optional[int] = None
    ):
        """Configure a memory pool"""
        self._config[name] = PoolConfig(
            percentage=percentage,
            priority=priority,
            hard_cap=hard_cap
        )

    def initialize_pools(self):
        """Initialize pools based on configuration"""
        total_allocated = 0

        for name, config in self._config.items():
            base_budget = int(self.global_budget * config.percentage)
            capped_budget = min(base_budget, config.hard_cap) if config.hard_cap else base_budget

            self._pools[name] = MemoryPool(name, capped_budget, config.hard_cap)
            total_allocated += capped_budget

        # Distribute unused tokens to expandable pools
        unused = self.global_budget - total_allocated
        if unused > 0:
            expandable = sorted(
                [(name, cfg) for name, cfg in self._config.items() if cfg.priority > 0],
                key=lambda x: x[1].priority,
                reverse=True
            )
            if expandable:
                bonus_per_pool = unused // len(expandable)
                for name, _ in expandable:
                    self._pools[name].max_tokens += bonus_per_pool

        logger.info(f"Initialized {len(self._pools)} memory pools with {self.global_budget} token budget")

    def add_memory(self, pool_name: str, item: MemoryItem) -> bool:
        """Add a memory item to a pool, trimming if necessary"""
        if pool_name not in self._pools:
            logger.warning(f"Unknown pool: {pool_name}")
            return False

        pool = self._pools[pool_name]

        # Check if we need to trim
        if pool.used_tokens + item.estimated_tokens > pool.max_tokens:
            asyncio.create_task(self._trim_pool(pool_name))

        return pool.add(item)

    async def _trim_pool(self, pool_name: str):
        """Trim a pool by summarizing oldest items"""
        pool = self._pools[pool_name]

        while pool.used_tokens > pool.max_tokens:
            oldest = pool.get_oldest(4)
            if not oldest:
                break

            # If this is ActiveSession, summarize before removing
            if pool_name == "ActiveSession" and self.summarizer:
                combined_text = " ".join(item.text for item in oldest)
                try:
                    summary = await self.summarizer(combined_text)
                    if summary:
                        # Add summary to RecentHistory
                        self.add_memory("RecentHistory", MemoryItem(
                            text=summary,
                            role="system",
                            priority_score=1.5,
                            metadata={"source": "summarized_session"}
                        ))
                        logger.info(f"Summarized {len(oldest)} items from ActiveSession")
                except Exception as e:
                    logger.error(f"Summarization failed: {e}")

            pool.remove_items(oldest)

    def add_observation(self, text: str, priority: float = 1.0):
        """Add a visual observation to ActiveSession"""
        self.add_memory("ActiveSession", MemoryItem(
            text=f"[Observation] {text}",
            role="observation",
            priority_score=priority
        ))

    def add_decision(self, action: str, reasoning: str, priority: float = 2.0):
        """Add a decision to ActiveSession"""
        self.add_memory("ActiveSession", MemoryItem(
            text=f"[Decision] {action}: {reasoning}",
            role="decision",
            priority_score=priority
        ))

    def add_user_input(self, text: str):
        """Add user/external input"""
        self.add_memory("ActiveSession", MemoryItem(
            text=text,
            role="user",
            priority_score=1.5
        ))

    def add_response(self, text: str):
        """Add assistant response"""
        self.add_memory("ActiveSession", MemoryItem(
            text=text,
            role="assistant",
            priority_score=1.0
        ))

    def set_core_memory(self, identity: str):
        """Set the core identity/personality (replaces existing)"""
        self._pools["Core"].clear()
        self.add_memory("Core", MemoryItem(
            text=identity,
            role="system",
            priority_score=10.0  # Highest priority
        ))

    def add_recall(self, text: str, relevance: float = 1.0):
        """Add a recalled memory from semantic search"""
        self.add_memory("Recall", MemoryItem(
            text=f"[Memory] {text}",
            role="system",
            priority_score=relevance
        ))

    def set_spatial_context(self, context: str):
        """Set current spatial/sensor context in Buffer"""
        # Clear old spatial context
        buffer = self._pools["Buffer"]
        buffer._items = [i for i in buffer._items if "[Spatial]" not in i.text]

        self.add_memory("Buffer", MemoryItem(
            text=f"[Spatial] {context}",
            role="system",
            priority_score=0.5
        ))

    def gather_context(self, available_tokens: Optional[int] = None) -> List[MemoryItem]:
        """
        Gather memory items from all pools up to token budget.

        Returns items organized by pool for building the prompt.
        """
        budget = available_tokens or (self.global_budget - self.OVERHEAD_TOKENS)
        remaining = budget
        result = []

        # Gather from pools in order of importance
        pool_order = ["Core", "ActiveSession", "RecentHistory", "Recall", "Buffer"]

        for pool_name in pool_order:
            if pool_name not in self._pools:
                continue

            pool = self._pools[pool_name]
            items = pool.get_items(remaining)

            for item in items:
                if remaining >= item.estimated_tokens:
                    result.append(item)
                    remaining -= item.estimated_tokens

        return result

    def format_for_prompt(self, items: List[MemoryItem]) -> str:
        """Format gathered memory items into a prompt string"""
        if not items:
            return ""

        sections = {}
        for item in items:
            pool = item.pool_name or "Unknown"
            if pool not in sections:
                sections[pool] = []
            sections[pool].append(item.text)

        lines = []

        section_labels = {
            "Core": "=== IDENTITY ===",
            "ActiveSession": "=== CURRENT SESSION ===",
            "RecentHistory": "=== RECENT HISTORY ===",
            "Recall": "=== RELEVANT MEMORIES ===",
            "Buffer": "=== SPATIAL AWARENESS ==="
        }

        for pool_name in ["Core", "ActiveSession", "RecentHistory", "Recall", "Buffer"]:
            if pool_name in sections:
                lines.append(section_labels.get(pool_name, f"=== {pool_name.upper()} ==="))
                for text in sections[pool_name]:
                    lines.append(f"  {text}")
                lines.append("")

        return "\n".join(lines)

    def get_usage_summary(self) -> Dict[str, Dict[str, int]]:
        """Get token usage summary for all pools"""
        return {
            name: {
                "used": pool.used_tokens,
                "max": pool.max_tokens,
                "items": len(pool._items)
            }
            for name, pool in self._pools.items()
        }

    def print_usage(self):
        """Print current memory usage"""
        logger.info("=== MEMORY USAGE ===")
        for name, pool in self._pools.items():
            logger.info(f"  {name:15}: {pool.used_tokens:5} / {pool.max_tokens:5} tokens ({len(pool._items)} items)")
        logger.info("====================")

    def clear_session(self):
        """Clear ActiveSession (e.g., when starting fresh)"""
        self._pools["ActiveSession"].clear()
        logger.info("Cleared ActiveSession")

    def clear_all(self):
        """Clear all pools except Core"""
        for name, pool in self._pools.items():
            if name != "Core":
                pool.clear()
        logger.info("Cleared all memory pools (except Core)")
