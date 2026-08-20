import sqlite3

from hazard_guard_bag_recorder.summary import summarize_sqlite_bag


def test_summary_counts_messages_without_deserializing_payloads(tmp_path):
    database = tmp_path / "bag_0.db3"
    with sqlite3.connect(database) as connection:
        connection.execute("CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT, type TEXT)")
        connection.execute("CREATE TABLE messages (id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER)")
        connection.execute("INSERT INTO topics VALUES (1, '/scan', 'sensor_msgs/msg/LaserScan')")
        connection.executemany("INSERT INTO messages VALUES (?, 1, ?)", [(1, 1_000_000_000), (2, 2_000_000_000)])
    summary = summarize_sqlite_bag(tmp_path)
    assert summary["topic_metrics"] == [
        {"name": "/scan", "type": "sensor_msgs/msg/LaserScan", "messages": 2, "duration_seconds": 1.0, "average_rate_hz": 2.0}
    ]
