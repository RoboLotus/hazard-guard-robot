"""Portable rosbag2 SQLite topic summary without loading message payloads."""

from __future__ import annotations

import sqlite3
from pathlib import Path


def summarize_sqlite_bag(bag_directory: Path) -> dict:
    databases = sorted(bag_directory.glob("*.db3"))
    if not databases:
        return {"storage_summary": "sqlite3 bag database not found", "topic_metrics": []}
    uri = f"file:{databases[0].as_posix()}?mode=ro"
    try:
        with sqlite3.connect(uri, uri=True) as connection:
            rows = connection.execute(
                """
                SELECT topics.name, topics.type, COUNT(messages.id),
                       MIN(messages.timestamp), MAX(messages.timestamp)
                FROM messages JOIN topics ON messages.topic_id = topics.id
                GROUP BY topics.id ORDER BY topics.name
                """
            ).fetchall()
    except sqlite3.Error as exc:
        return {"storage_summary": f"sqlite3 summary unavailable: {exc}", "topic_metrics": []}
    metrics = []
    for name, message_type, count, first_ns, last_ns in rows:
        duration = max(0.0, (last_ns - first_ns) / 1_000_000_000) if count > 1 else 0.0
        metrics.append(
            {
                "name": name,
                "type": message_type,
                "messages": count,
                "duration_seconds": round(duration, 3),
                "average_rate_hz": round(count / duration, 3) if duration else None,
            }
        )
    return {"storage_summary": "sqlite3", "topic_metrics": metrics}
