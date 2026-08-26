import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from pc.multisensor.obstacle_pass_tracker import ObstaclePassTracker


def snap(zone, status):
    return {zone: {"status": status}}


tracker = ObstaclePassTracker(clear_confirm_frames=3)

assert tracker.start("LEFT")
assert tracker.side_zone == "RIGHT_SIDE"

# Front may become clear, but obstacle has not reached side yet.
assert tracker.update(snap("RIGHT_SIDE", "CLEAR")) is False
assert tracker.state == tracker.WAIT_SIDE

# Obstacle reaches the watched side.
assert tracker.update(snap("RIGHT_SIDE", "DANGER")) is False
assert tracker.state == tracker.TRACK_SIDE

# Must remain clear for 3 consecutive frames.
assert tracker.update(snap("RIGHT_SIDE", "CLEAR")) is False
assert tracker.update(snap("RIGHT_SIDE", "CLEAR")) is False
assert tracker.update(snap("RIGHT_SIDE", "CLEAR")) is True
assert tracker.state == tracker.PASSED

# RIGHT avoidance must watch LEFT_SIDE.
tracker.reset()
assert tracker.start("RIGHT")
assert tracker.side_zone == "LEFT_SIDE"
assert tracker.update(snap("LEFT_SIDE", "CAUTION")) is False
assert tracker.update(snap("LEFT_SIDE", "CLEAR")) is False
assert tracker.update(snap("LEFT_SIDE", "CLEAR")) is False
assert tracker.update(snap("LEFT_SIDE", "CLEAR")) is True

print("[PASS] ObstaclePassTracker tests passed")
