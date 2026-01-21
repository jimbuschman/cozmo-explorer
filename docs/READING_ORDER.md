# Cozmo Explorer - Documentation Reading Order

## For New Claude Sessions

When starting a new Claude Code session on this project, read these files in order to get up to speed quickly.

---

## Priority 1: Essential Context (Read These First)

### 1. `PLAN.md` (root folder)
**What**: Original project overview and architecture
**Why**: Understand what this project is - an autonomous Cozmo robot using LLM for exploration
**Key points**: Architecture diagram, tech stack, project structure, implementation phases

### 2. `docs/revised system desciprtion.txt`
**What**: Core philosophy and design principles
**Why**: Understand the "why" - data-first learning, LLM as observer not pilot
**Key points**:
- Learning System = the brain (required, works without LLM)
- LLM = curiosity/personality (optional, adds purpose)
- Phased approach to intelligence

### 3. `docs/ARCHITECTURE_PHASES.md`
**What**: Detailed phased implementation plan
**Why**: Understand current phase (Phase 1) and what's coming
**Key points**:
- Phase 1: Grounded Survival Learning (CURRENT)
- Phase 2: Place & Context Learning (NEXT)
- Phase 3: Multi-Agent Exploration (FUTURE)

---

## Priority 2: Current Implementation Details

### 4. `config.py`
**What**: Configuration with phase settings
**Why**: See what mode the system is in (Phase 1 survival vs Phase 2+ directed)
**Key settings**:
- `PHASE1_PURE_SURVIVAL = True` → Robot explores autonomously
- `ENABLE_LLM = True/False` → LLM enabled/disabled
- External sensor port settings

### 5. `docs/LEARNING_SYSTEM.md`
**What**: Detailed learning system architecture
**Why**: Understand how the robot learns from experience
**Key points**: Data collection → Pattern analysis → Rule proposal → Validation → Activation

### 6. `docs/HARDWARE_SETUP.md`
**What**: Hardware architecture and communication
**Why**: Understand the physical setup (robots, sensors, Pi, main PC)
**Key points**: Data flow from sensors → Pi → Main system

---

## Priority 3: Reference (Read When Needed)

### `docs/ESP32_INTEGRATION.md`
**What**: ESP32 sensor pod integration details
**Why**: Specific implementation of external sensors
**When**: Working on sensor code or hardware

### `docs/PHASE1_CODE_CHANGES.md`
**What**: Code changes made for Phase 1 pure survival mode
**Why**: Understanding Phase 1 specific implementation
**When**: Debugging or modifying Phase 1 behavior

### `docs/PHASE2_COMPONENTS.md`
**What**: Sketches for Phase 2 components (place recognition, room clustering)
**Why**: Planning for Phase 2 implementation
**When**: Ready to start Phase 2

### `docs/SETUP_GUIDE.md`
**What**: How to set up and run the system
**Why**: Getting the system running
**When**: First-time setup or troubleshooting

---

## Quick Start for Claude

If you're a new Claude session and need to understand this project fast:

```
1. Read PLAN.md (5 min) - What is this project?
2. Read revised system desciprtion.txt (3 min) - What's the philosophy?
3. Read ARCHITECTURE_PHASES.md sections on Phase 1 (5 min) - What are we doing now?
4. Skim config.py (2 min) - What mode is it in?
```

Total: ~15 minutes to understand the project well enough to help.

---

## Key Concepts Summary

### The Two Layers
```
LLM ("The Explorer") - Optional, adds curiosity/personality
        ↓
Learning System ("The Brain") - Required, learns from experience
        ↓
Robot Fleet - Cozmo robots with sensors
```

### Current State (Phase 1)
- Robot wanders, collides, recovers, logs everything
- Learning system proposes and validates micro-rules
- LLM observes and journals (doesn't direct)
- System works fully without LLM

### Data Flow
```
Cozmo + ESP32 sensors → WiFi → Raspberry Pi (buffer) → Ethernet → Main PC (brain)
```

---

## Archive

Older or superseded documents are in `docs/archive/`:
- `HARDWARE_COMMUNICATION.md` - Generic multi-robot options (superseded by HARDWARE_SETUP.md)

---

## File Locations Quick Reference

| File | Location | Purpose |
|------|----------|---------|
| Project overview | `PLAN.md` | What is this? |
| Philosophy | `docs/revised system desciprtion.txt` | Why this approach? |
| Phases | `docs/ARCHITECTURE_PHASES.md` | Phase 1/2/3 details |
| Config | `config.py` | Current settings |
| Learning details | `docs/LEARNING_SYSTEM.md` | How learning works |
| Hardware | `docs/HARDWARE_SETUP.md` | Physical architecture |
| ESP32 | `docs/ESP32_INTEGRATION.md` | Sensor pod details |
| Phase 1 code | `docs/PHASE1_CODE_CHANGES.md` | Phase 1 modifications |
| Phase 2 design | `docs/PHASE2_COMPONENTS.md` | Future components |
| Setup | `docs/SETUP_GUIDE.md` | How to run |
