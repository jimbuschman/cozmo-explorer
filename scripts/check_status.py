#!/usr/bin/env python
"""
Learning System Status Checker

Quick diagnostic to see what data has been collected and what rules exist.
Run from project root: python scripts/check_status.py
"""
import sqlite3
from pathlib import Path
from datetime import datetime

# Find data directory
script_dir = Path(__file__).parent
project_root = script_dir.parent
data_dir = project_root / "data"
db_path = data_dir / "state.db"

print("=" * 60)
print("  COZMO EXPLORER - LEARNING SYSTEM STATUS")
print("=" * 60)
print()

if not db_path.exists():
    print(f"ERROR: No database found at {db_path}")
    print("Run main.py first to create the database.")
    exit(1)

conn = sqlite3.connect(str(db_path))
conn.row_factory = sqlite3.Row

# Check if learning tables exist
tables = [row[0] for row in conn.execute(
    "SELECT name FROM sqlite_master WHERE type='table'"
).fetchall()]

if "sensor_snapshots" not in tables:
    print("WARNING: sensor_snapshots table not found.")
    print("The learning system tables may not be initialized yet.")
    print("Run main.py to initialize.")
    exit(1)

# === DATA COLLECTION ===
print("DATA COLLECTION")
print("-" * 40)

# Sensor snapshots
try:
    count = conn.execute("SELECT COUNT(*) FROM sensor_snapshots").fetchone()[0]
    print(f"Sensor snapshots:     {count}")
except:
    print("Sensor snapshots:     (table not found)")

# Action events
try:
    count = conn.execute("SELECT COUNT(*) FROM action_events").fetchone()[0]
    print(f"Action events:        {count}")
except:
    print("Action events:        (table not found)")

# Outcome events
try:
    count = conn.execute("SELECT COUNT(*) FROM outcome_events").fetchone()[0]
    print(f"Outcome events:       {count}")
except:
    print("Outcome events:       (table not found)")

# Sessions
try:
    count = conn.execute("SELECT COUNT(*) FROM sessions").fetchone()[0]
    print(f"Sessions:             {count}")
except:
    pass

print()

# === ACTIONS BREAKDOWN ===
print("ACTIONS BY TYPE")
print("-" * 40)
try:
    rows = conn.execute("""
        SELECT action_type, COUNT(*) as cnt
        FROM action_events
        GROUP BY action_type
        ORDER BY cnt DESC
    """).fetchall()
    if rows:
        for row in rows:
            print(f"  {row['action_type']:20} {row['cnt']:>5}")
    else:
        print("  (no actions recorded yet)")
except Exception as e:
    print(f"  (error: {e})")

print()

# === OUTCOMES BREAKDOWN ===
print("OUTCOMES BY TYPE")
print("-" * 40)
try:
    rows = conn.execute("""
        SELECT outcome_type, COUNT(*) as cnt
        FROM outcome_events
        GROUP BY outcome_type
        ORDER BY cnt DESC
    """).fetchall()
    if rows:
        for row in rows:
            print(f"  {row['outcome_type']:20} {row['cnt']:>5}")
    else:
        print("  (no outcomes recorded yet)")
except Exception as e:
    print(f"  (error: {e})")

print()

# === RECOVERY SUCCESS RATES ===
print("RECOVERY SUCCESS RATES")
print("-" * 40)
try:
    for action_type in ["escape_stall", "escape_cliff"]:
        row = conn.execute("""
            SELECT
                COUNT(*) as total,
                SUM(CASE WHEN o.outcome_type = 'success' THEN 1 ELSE 0 END) as success
            FROM action_events a
            LEFT JOIN outcome_events o ON a.id = o.action_event_id
            WHERE a.action_type = ?
        """, (action_type,)).fetchone()

        total = row['total'] or 0
        success = row['success'] or 0
        rate = f"{success/total:.1%}" if total > 0 else "n/a"
        print(f"  {action_type:20} {success:>3}/{total:<3} ({rate})")
except Exception as e:
    print(f"  (error: {e})")

print()

# === LEARNED RULES ===
print("LEARNED RULES")
print("-" * 40)
try:
    rows = conn.execute("""
        SELECT name, status, times_applied, times_successful, evidence_summary
        FROM learned_rules
        ORDER BY status, name
    """).fetchall()

    if rows:
        for row in rows:
            applied = row['times_applied'] or 0
            successful = row['times_successful'] or 0
            rate = f"{successful/applied:.0%}" if applied > 0 else "n/a"
            print(f"  [{row['status']:10}] {row['name']}")
            print(f"               Applied: {applied}, Success: {rate}")
            if row['evidence_summary']:
                evidence = row['evidence_summary'][:60] + "..." if len(row['evidence_summary'] or "") > 60 else row['evidence_summary']
                print(f"               Evidence: {evidence}")
            print()
    else:
        print("  (no rules yet - need more data or run learning cycle)")
except Exception as e:
    print(f"  (error: {e})")

# === IMAGES ===
print("LEARNING IMAGES")
print("-" * 40)
images_dir = data_dir / "learning_images"
if images_dir.exists():
    jpg_files = list(images_dir.glob("*.jpg"))
    print(f"  Total images:       {len(jpg_files)}")

    # Count by type
    before_count = len([f for f in jpg_files if "_before_" in f.name])
    after_count = len([f for f in jpg_files if "_after_" in f.name])
    print(f"  Before images:      {before_count}")
    print(f"  After images:       {after_count}")

    # Most recent
    if jpg_files:
        newest = max(jpg_files, key=lambda f: f.stat().st_mtime)
        mtime = datetime.fromtimestamp(newest.stat().st_mtime)
        print(f"  Most recent:        {newest.name}")
        print(f"                      ({mtime.strftime('%Y-%m-%d %H:%M:%S')})")
else:
    print("  (no images directory yet)")

print()

# === RECENT ACTIVITY ===
print("RECENT ACTIVITY (last 5 actions)")
print("-" * 40)
try:
    rows = conn.execute("""
        SELECT
            a.timestamp,
            a.action_type,
            a.trigger,
            o.outcome_type
        FROM action_events a
        LEFT JOIN outcome_events o ON a.id = o.action_event_id
        ORDER BY a.timestamp DESC
        LIMIT 5
    """).fetchall()

    if rows:
        for row in rows:
            ts = row['timestamp'][:19] if row['timestamp'] else "?"
            outcome = row['outcome_type'] or "?"
            print(f"  {ts}  {row['action_type']:15} -> {outcome}")
    else:
        print("  (no recent activity)")
except Exception as e:
    print(f"  (error: {e})")

print()
print("=" * 60)

# === RECOMMENDATIONS ===
try:
    action_count = conn.execute("SELECT COUNT(*) FROM action_events").fetchone()[0]
    rule_count = conn.execute("SELECT COUNT(*) FROM learned_rules WHERE status = 'active'").fetchone()[0]

    print("RECOMMENDATIONS")
    print("-" * 40)

    if action_count < 20:
        print(f"  - Need {20 - action_count} more recovery events before learning analysis runs")
        print("    (Let Cozmo bump into more obstacles)")
    elif rule_count == 0:
        print("  - Enough data collected! Learning cycle should run soon.")
        print("    (Wait for 'Running learning analysis cycle' in logs)")
    else:
        print(f"  - System is learning! {rule_count} active rule(s)")
        print("    (Watch for 'Selected rule' messages in logs)")
except:
    pass

print()

conn.close()
