# Phase 1: Pure Survival Mode - Code Changes

This document outlines changes to make the current system strictly Phase 1 focused.

## Overview

The current implementation is ~85% Phase 1 aligned. The main adjustments:
1. Make LLM goal-setting optional (default to pure exploration)
2. Simplify state machine (fewer states, exploration-focused)
3. Ensure system works fully without LLM
4. Focus logging on survival-relevant data

---

## Change 1: Optional LLM Goal-Setting

### Current Behavior
The LLM is queried periodically and sets goals (explore, investigate, go_to).

### Phase 1 Behavior
LLM observes and journals only. Robot explores by default.

### File: `config.py`

Add configuration flag:
```python
# Phase 1: Pure survival mode
# When True, LLM only observes/journals, doesn't set goals
PHASE1_PURE_SURVIVAL = True

# LLM provides goals (Phase 2+)
LLM_GOAL_SETTING_ENABLED = not PHASE1_PURE_SURVIVAL
```

### File: `brain/state_machine.py`

Modify `_should_query_llm()`:
```python
async def _should_query_llm(self) -> bool:
    """Determine if we should ask the LLM for guidance"""
    # Phase 1: LLM doesn't set goals, only observes
    if not config.LLM_GOAL_SETTING_ENABLED:
        return False

    if self.llm_client is None:
        return False

    # ... rest of existing logic
```

Modify `_do_idle()` to always explore:
```python
async def _do_idle(self):
    """Idle state - wait and occasionally look around"""
    await asyncio.sleep(1.0)

    # Phase 1: Always explore, don't wait for LLM
    if self.context.current_goal is None:
        self.set_goal(Goal(
            description="Explore and learn",
            goal_type="explore"
        ))
        await self._change_state(RobotState.EXPLORING)
```

---

## Change 2: Simplified State Machine

### Current States
```
IDLE, EXPLORING, INVESTIGATING, GOING_TO, STUCK, CHARGING, WAITING_FOR_LLM, ERROR
```

### Phase 1 States (Simplified)
```
IDLE, EXPLORING, STUCK, CHARGING, ERROR
```

INVESTIGATING and GOING_TO are Phase 2+ features (purposeful movement).

### File: `brain/state_machine.py`

For Phase 1, you can leave all states but simplify usage:
```python
class RobotState(Enum):
    """High-level robot states"""
    IDLE = auto()           # Waiting
    EXPLORING = auto()      # Wandering and learning (PRIMARY Phase 1 state)
    INVESTIGATING = auto()  # Phase 2+: Looking at something
    GOING_TO = auto()       # Phase 2+: Navigation
    STUCK = auto()          # Recovery mode
    CHARGING = auto()       # On charger
    WAITING_FOR_LLM = auto() # Phase 2+: Waiting for guidance
    ERROR = auto()          # Something went wrong
```

In Phase 1 config mode, the robot cycles between:
```
IDLE → EXPLORING → (STUCK if problems) → EXPLORING → ...
         ↓
      CHARGING (if battery low)
```

---

## Change 3: LLM as Observer Only (Phase 1)

### File: `brain/state_machine.py`

Add a periodic observation call (not goal-setting):
```python
async def _run_loop(self):
    """Main state machine loop"""
    while self._running:
        if await self._check_safety():
            continue

        # Phase 1: LLM observes but doesn't direct
        if config.PHASE1_PURE_SURVIVAL:
            # Periodically let LLM observe and journal (non-blocking)
            if await self._should_llm_observe():
                asyncio.create_task(self._llm_observe_only())
        else:
            # Phase 2+: LLM provides guidance
            if await self._should_query_llm():
                await self._query_llm_for_guidance()

        # Learning cycle runs regardless
        if await self._should_run_learning():
            await self._run_learning_cycle()

        await self._execute_state()
        await asyncio.sleep(0.1)

async def _should_llm_observe(self) -> bool:
    """Check if LLM should make an observation (Phase 1)"""
    if self.llm_client is None:
        return False

    # Less frequent than goal-setting - every 60 seconds
    if self.context.last_llm_query is None:
        return True

    elapsed = (datetime.now() - self.context.last_llm_query).total_seconds()
    return elapsed > 60.0

async def _llm_observe_only(self):
    """Let LLM observe and journal without setting goals (Phase 1)"""
    try:
        context_snapshot = await self.memory_context.build_context(
            robot=self.robot,
            current_state=self.state.name,
            exploration_time=self.context.exploration_time,
            stuck_count=self.context.stuck_count
        )

        # Simpler prompt for observation only
        observation_prompt = f"""
You are observing a robot learning to navigate. Do NOT give commands.
Simply observe and note anything interesting for the journal.

Current state: {self.state.name}
Exploration time: {self.context.exploration_time:.1f}s
Stuck count: {self.context.stuck_count}
Position: ({self.robot.pose.x:.0f}, {self.robot.pose.y:.0f})

Recent activity summary available.

Write a brief journal observation (1-2 sentences). Focus on:
- What the robot seems to be learning
- Any patterns you notice
- Interesting behaviors

Respond with just the observation, no commands.
"""

        observation = await self.llm_client._chat(observation_prompt)

        # Log to journal only, don't set goals
        if observation and self.memory_context:
            self.memory_context.add_to_journal(
                entry_type="observation",
                content=observation.strip()
            )

        self.context.last_llm_query = datetime.now()

    except Exception as e:
        logger.debug(f"LLM observation failed: {e}")
```

---

## Change 4: Ensure No-LLM Operation

### File: `main.py`

Make LLM truly optional in initialization:
```python
async def initialize(self):
    # ... existing code ...

    # LLM is optional
    self.llm_client = None
    if config.ENABLE_LLM:
        logger.info("Checking LLM availability...")
        try:
            from llm.client import LLMClient
            self.llm_client = LLMClient()
            if await self.llm_client.check_health():
                logger.info(f"LLM ready: {config.OLLAMA_MODEL}")
            else:
                logger.warning("LLM not available - running without")
                self.llm_client = None
        except Exception as e:
            logger.warning(f"LLM init failed: {e} - running without")
            self.llm_client = None
    else:
        logger.info("LLM disabled by config")

    # ... rest of init - should work fine with llm_client=None
```

### File: `config.py`

Add LLM enable flag:
```python
# LLM Configuration
ENABLE_LLM = True  # Set to False to disable LLM entirely
OLLAMA_HOST = "http://localhost:11434"
OLLAMA_MODEL = "gemma3:latest"
```

---

## Change 5: Focus Data Collection on Survival

The current logging is already good, but ensure Phase 1 focus:

### File: `memory/experience_logger.py`

Add method to get Phase 1 summary:
```python
def get_phase1_summary(self) -> dict:
    """Get summary of Phase 1 survival learning progress"""
    return {
        'total_snapshots': self._count_table('sensor_snapshots'),
        'total_actions': self._count_table('action_events'),
        'total_outcomes': self._count_table('outcome_events'),
        'recovery_stats': {
            'escape_stall': self.get_recovery_statistics('escape_stall'),
            'escape_cliff': self.get_recovery_statistics('escape_cliff'),
        },
        'collision_precursors': len(self.get_collision_precursors()),
        'images_captured': self._count_images(),
    }

def _count_table(self, table: str) -> int:
    cursor = self.conn.execute(f"SELECT COUNT(*) FROM {table}")
    return cursor.fetchone()[0]

def _count_images(self) -> int:
    from pathlib import Path
    images_dir = Path(config.DATA_DIR) / "learning_images"
    if images_dir.exists():
        return len(list(images_dir.glob("*.jpg")))
    return 0
```

---

## Change 6: Phase 1 Status Display

### File: `main.py`

Modify `_print_status()` for Phase 1 focus:
```python
def _print_status(self):
    """Print periodic status update - Phase 1 focused"""
    if not self.robot:
        return

    sensors = self.robot.sensors
    pose = self.robot.pose

    logger.info("=" * 50)
    logger.info(f"State: {self.state_machine.state.name if self.state_machine else 'N/A'}")
    logger.info(f"Position: ({pose.x:.0f}, {pose.y:.0f}) @ {pose.angle * 57.3:.0f}°")
    logger.info(f"Battery: {sensors.battery_voltage:.2f}V")

    # External sensors
    if sensors.ext_connected:
        dists = sensors.get_obstacle_distances()
        logger.info(f"Distances: F={dists['front']}mm L={dists['left']}mm R={dists['right']}mm")

    # Phase 1 Learning Progress
    if self.experience_logger:
        stats = self.experience_logger.get_recovery_statistics("escape_stall")
        rules_count = len(self.rules_store.get_active_rules()) if self.rules_store else 0
        logger.info(f"Learning: {stats.get('total', 0)} recoveries, {rules_count} active rules")
        if stats.get('success_rate'):
            logger.info(f"Recovery success rate: {stats['success_rate']:.0%}")

    # LLM status
    llm_status = "active" if self.llm_client else "disabled"
    mode = "observer" if config.PHASE1_PURE_SURVIVAL else "director"
    logger.info(f"LLM: {llm_status} ({mode} mode)")

    logger.info("=" * 50)
```

---

## Summary of Changes

| File | Change | Purpose |
|------|--------|---------|
| `config.py` | Add `PHASE1_PURE_SURVIVAL`, `LLM_GOAL_SETTING_ENABLED`, `ENABLE_LLM` | Configuration flags |
| `brain/state_machine.py` | Modify `_should_query_llm()` | Disable LLM goals in Phase 1 |
| `brain/state_machine.py` | Add `_llm_observe_only()` | LLM journals without directing |
| `brain/state_machine.py` | Simplify `_do_idle()` | Always explore in Phase 1 |
| `main.py` | Make LLM optional in init | Graceful no-LLM operation |
| `main.py` | Update `_print_status()` | Phase 1 focused display |
| `memory/experience_logger.py` | Add `get_phase1_summary()` | Track Phase 1 progress |

---

## Testing Phase 1 Pure Mode

### With LLM (observer mode)
```bash
# config.py: PHASE1_PURE_SURVIVAL = True, ENABLE_LLM = True
python main.py
# Robot explores, LLM journals observations
```

### Without LLM (pure survival)
```bash
# config.py: PHASE1_PURE_SURVIVAL = True, ENABLE_LLM = False
python main.py
# Robot explores and learns, no LLM at all
```

### Phase 2+ mode (LLM directs)
```bash
# config.py: PHASE1_PURE_SURVIVAL = False, ENABLE_LLM = True
python main.py
# Robot asks LLM what to do (current behavior)
```
