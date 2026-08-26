import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from pc.multisensor.trajectory_planner import TrajectoryPlanner


planner = TrajectoryPlanner()

left = planner.create_plan(
    "LEFT",
    speed_mps=4.17,
    obstacle_distance_m=12.0,
)

right = planner.create_plan(
    "RIGHT",
    speed_mps=4.17,
    obstacle_distance_m=12.0,
)

assert left["length_m"] == 24.0
assert right["length_m"] == 24.0

# Starts and finishes on the original road trajectory.
assert abs(planner.lateral_offset(left, 0.0)) < 1e-9
assert abs(planner.lateral_offset(left, 24.0)) < 1e-9

# Maximum displacement is around the obstacle.
assert planner.lateral_offset(left, 12.0) > 2.4
assert planner.lateral_offset(right, 12.0) < -2.4

# Initial steering direction.
left_angle = planner.steering_angle(
    left,
    progress_m=0.0,
    camera_angle=90.0,
)

right_angle = planner.steering_angle(
    right,
    progress_m=0.0,
    camera_angle=90.0,
)

assert left_angle < 90.0
assert right_angle > 90.0

# Steering is bounded.
assert 78.0 <= left_angle <= 102.0
assert 78.0 <= right_angle <= 102.0

# Trajectory returns toward the normal path.
left_late = planner.steering_angle(
    left,
    progress_m=18.0,
    camera_angle=90.0,
)

right_late = planner.steering_angle(
    right,
    progress_m=18.0,
    camera_angle=90.0,
)

assert left_late > 90.0
assert right_late < 90.0

assert planner.is_complete(left, 23.9) is False
assert planner.is_complete(left, 24.0) is True

print("LEFT PLAN :", left)
print("RIGHT PLAN:", right)
print(
    f"LEFT steering: start={left_angle:.1f} late={left_late:.1f}"
)
print(
    f"RIGHT steering: start={right_angle:.1f} late={right_late:.1f}"
)
print("[PASS] Smooth trajectory planner tests passed")
