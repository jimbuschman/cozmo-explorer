# Cozmo Explorer - Documentation Reading Order

## For New Claude Sessions

When starting a new Claude Code session on this project, read these files in order to get up to speed quickly.

---

## Priority 1: Essential Context (Read These First)

### 1. `CLAUDE.md` (root folder)
**What**: Rules for Claude, current architecture summary, key files, test commands
**Why**: This is the primary context document. Contains everything needed for day-to-day work.
**Key points**:
- "The system learns, not Cozmo" - mapping platform architecture
- FrontierNavigator + MapperStateMachine are the active code
- Escape behavior is deterministic (no rules)
- LLM only runs post-session for map review
- Current sim benchmark: 56.5% coverage in 7200s

### 2. `PLAN.md` (root folder)
**What**: Project overview, architecture diagram, file structure, how it works
**Why**: Understand what this project is and how all the pieces fit together
**Key points**: Architecture diagram, complete file tree, hardware list, current performance numbers

### 3. `config.py`
**What**: All tunable parameters
**Why**: See current settings for speeds, thresholds, sensor geometry, modes
**Key settings**: `TRAILER_MODE = True`, `ESCAPE_ANGLE = 135`, `WANDER_SPEED = 70`

---

## Priority 2: Detailed Architecture

### 4. `docs/ARCHITECTURE.md`
**What**: Detailed system architecture with diagrams and data flow
**Why**: Deep understanding of FrontierNavigator, escape mechanics, stagnation detection, spatial map, post-session LLM review
**Key sections**: FrontierNavigator loop, escape exclusion, stagnation/relocation, spatial map cell states, configuration reference

---

## Priority 3: Reference (Read When Needed)

### `docs/HARDWARE_SETUP.md`
**What**: Hardware architecture and communication topology
**Why**: Understanding the physical setup (Cozmo, ESP32, trailer, networking)
**When**: Working on sensor integration or hardware changes

### `docs/ESP32_INTEGRATION.md`
**What**: ESP32 sensor pod software integration (data format, Python reader, how navigator uses sensors)
**Why**: Understanding how sensor data flows into the mapping system
**When**: Working on sensor code or debugging sensor issues

### `docs/nano_esp32_sensor_integration.md`
**What**: Hardware wiring, pin assignments, Arduino/ESP32 firmware code
**Why**: Complete firmware reference with wiring diagrams and C++ code
**When**: Working on the physical sensor pod, reflashing firmware, or changing wiring

### `docs/SETUP_GUIDE.md`
**What**: How to set up and run the system (real robot + simulator)
**Why**: Getting the system running, troubleshooting
**When**: First-time setup, adding new hardware, or something isn't working

---

## Quick Start for Claude

If you're a new Claude session and need to understand this project fast:

```
1. Read CLAUDE.md (3 min) - Rules, architecture, key files, commands
2. Skim PLAN.md (3 min) - Architecture diagram, file structure
3. Skim config.py (1 min) - Current settings
```

Total: ~7 minutes to understand the project well enough to help.

For deeper work, also read `docs/ARCHITECTURE.md` for the detailed technical design.

---

## Key Concepts Summary

### The Architecture
```
MapperStateMachine (4 states: MAPPING, CAPTURING, REVIEWING, DONE)
        │
        └─ FrontierNavigator (persistent instance)
              │
              ├─ Drives toward nearest frontier
              ├─ Updates occupancy grid via sensor ray-tracing
              ├─ Escapes obstacles (deterministic reverse-arc)
              ├─ Detects stagnation (coverage rate)
              └─ Relocates to big unexplored clusters when stagnant
```

### LLM Role
```
During mapping: LLM is NOT involved (zero calls)
After mapping:  LLM reviews ASCII map + stats → semantic annotations
```

### Key Numbers
- 56.5% map coverage in 7200s sim-time (12 min wall clock at 10x)
- 18 escapes per session (reverse-arc at 135 degrees)
- 3 stagnation relocations per session
- 0 rules, 0 LLM calls during navigation

---

## Archive

Older documents about the previous rule-learning architecture are in `docs/archive/`:
- `ARCHITECTURE_PHASES.md` - Old 3-phase learning approach
- `LEARNING_SYSTEM.md` - Old PatternAnalyzer/LearningCoordinator pipeline
- `PHASE1_CODE_CHANGES.md` - Old Phase 1 modifications
- `PHASE2_COMPONENTS.md` - Sketches for place recognition (never built)
- `revised system desciprtion.txt` - Old "rules from patterns" philosophy
- `PLAN_original.md` - Original project overview
- `HARDWARE_COMMUNICATION.md` - Old multi-robot communication options

The archived code is in `archive/`:
- `learning_coordinator.py`, `learned_rules.py`, `pattern_analyzer.py`, `maneuvers.py`

---

## File Locations Quick Reference

| File | Location | Purpose |
|------|----------|---------|
| Claude context | `CLAUDE.md` | Rules + architecture for Claude sessions |
| Project overview | `PLAN.md` | Architecture, structure, how to run |
| Config | `config.py` | Current settings |
| Architecture | `docs/ARCHITECTURE.md` | Detailed system design |
| Hardware | `docs/HARDWARE_SETUP.md` | Physical architecture |
| ESP32 (software) | `docs/ESP32_INTEGRATION.md` | Sensor pod software integration |
| ESP32 (hardware) | `docs/nano_esp32_sensor_integration.md` | Wiring, firmware, pin assignments |
| Setup | `docs/SETUP_GUIDE.md` | How to run |
