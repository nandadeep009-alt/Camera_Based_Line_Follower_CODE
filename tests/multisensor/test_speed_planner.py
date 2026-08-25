import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "pc"))

from multisensor.speed_planner import SpeedPlanner


planner = SpeedPlanner(
    drive_speed_mps=3.0,
    slow_speed_mps=1.0,
)

assert planner.target_speed("DRIVE_ALLOWED") == 3.0
assert planner.target_speed("SLOW") == 1.0
assert planner.target_speed("STOP") == 0.0
assert planner.target_speed("FAIL_SAFE_STOP") == 0.0
assert planner.target_speed("UNKNOWN") == 0.0
assert planner.target_speed(None) == 0.0

bad_configs = [
    {"drive_speed_mps": -1.0, "slow_speed_mps": 1.0},
    {"drive_speed_mps": 3.0, "slow_speed_mps": -1.0},
    {"drive_speed_mps": 1.0, "slow_speed_mps": 2.0},
    {"drive_speed_mps": math.nan, "slow_speed_mps": 1.0},
    {"drive_speed_mps": math.inf, "slow_speed_mps": 1.0},
]

for config in bad_configs:
    try:
        SpeedPlanner(**config)
        raise AssertionError("Invalid speed configuration accepted")
    except ValueError:
        pass

assert planner.target_speed("DRIVE_ALLOWED", "LEFT") == 0.8
assert planner.target_speed("DRIVE_ALLOWED", "RIGHT") == 0.8
assert planner.target_speed("DRIVE_ALLOWED", "REVERSE") == 0.5
assert planner.target_speed("DRIVE_ALLOWED", "STOP") == 0.0
assert planner.target_speed("SLOW", "LEFT") == 0.8
assert planner.target_speed("SLOW", "RIGHT") == 0.8
assert planner.target_speed("SLOW", "REVERSE") == 0.5
assert planner.target_speed("DRIVE_ALLOWED", "UNKNOWN") == 0.0

print("[PASS] SpeedPlanner offline safety tests passed")
