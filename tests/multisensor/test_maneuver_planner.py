import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "pc"))

from multisensor.maneuver_planner import ManeuverPlanner


planner = ManeuverPlanner()


def make_snapshot():
    return {
        "LEFT_FRONT": {"status": "CLEAR", "distance_m": 6.0},
        "CENTER_FRONT": {"status": "CLEAR", "distance_m": 6.0},
        "RIGHT_FRONT": {"status": "CLEAR", "distance_m": 6.0},
        "LEFT_SIDE": {"status": "CLEAR", "distance_m": 6.0},
        "RIGHT_SIDE": {"status": "CLEAR", "distance_m": 6.0},
        "REAR": {"status": "CLEAR", "distance_m": 6.0},
    }


s = make_snapshot()
assert planner.choose(s) == "FORWARD"

s = make_snapshot()
s["CENTER_FRONT"]["status"] = "CAUTION"
assert planner.choose(s) == "FORWARD"

s = make_snapshot()
s["CENTER_FRONT"]["status"] = "DANGER"
s["RIGHT_SIDE"]["status"] = "DANGER"
assert planner.choose(s) == "LEFT"

s = make_snapshot()
s["CENTER_FRONT"]["status"] = "DANGER"
s["LEFT_SIDE"]["status"] = "DANGER"
assert planner.choose(s) == "RIGHT"

s = make_snapshot()
s["CENTER_FRONT"]["status"] = "DANGER"
s["LEFT_SIDE"]["status"] = "DANGER"
s["RIGHT_SIDE"]["status"] = "DANGER"
assert planner.choose(s) == "STOP"

s = make_snapshot()
s["CENTER_FRONT"]["status"] = "DANGER"
s["LEFT_FRONT"]["distance_m"] = 4.0
s["LEFT_SIDE"]["distance_m"] = 4.0
s["RIGHT_FRONT"]["distance_m"] = 5.0
s["RIGHT_SIDE"]["distance_m"] = 5.0
assert planner.choose(s) == "RIGHT"

s = make_snapshot()
s["CENTER_FRONT"]["status"] = "INVALID"
assert planner.choose(s) == "STOP"

assert planner.choose({}) == "STOP"

s = make_snapshot()
s["RIGHT_FRONT"]["status"] = "DANGER"
s["RIGHT_FRONT"]["distance_m"] = 0.8
assert planner.choose(s) == "LEFT"

s = make_snapshot()
s["LEFT_FRONT"]["status"] = "DANGER"
s["LEFT_FRONT"]["distance_m"] = 0.8
assert planner.choose(s) == "RIGHT"

print("[PASS] ManeuverPlanner offline tests passed")
