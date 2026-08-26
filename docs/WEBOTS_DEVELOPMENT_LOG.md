# ROVE Webots Development Log

## Multisensor / Trajectory Development

### Baseline 1 - Multisensor perception
- Front SICK LMS 291 LiDAR
- LEFT_FRONT / CENTER_FRONT / RIGHT_FRONT zones
- Left-side, right-side and rear proximity sensors
- Sensor fusion and fail-safe supervisor

### Baseline 2 - Sensor-confirmed obstacle avoidance
- LEFT / RIGHT maneuver selection
- Maneuver direction locking
- Obstacle pass tracker
- Side-sensor pass confirmation
- Hard obstacle-clearance safety guard

### Baseline 3 - Webots trajectory integration
Date: 2026-08-26

Added:
- TrajectoryPlanner
- Smooth LEFT and RIGHT avoidance trajectories
- GPS-based physical trajectory progress
- Gyro yaw-rate supervision
- WebotsPoseReader
- Sensor-supervised trajectory execution
- Progressive acceleration/deceleration
- Side sensors increased to 31 rays and 60-degree FOV

Important:
- This is the saved trajectory integration baseline before visible-path and dynamic rerouting development.
- Real-time trajectory execution still requires continued Webots validation.
- Legacy TURN_OUT / PASS_GUIDED code remains temporarily as fallback and should be removed only after trajectory validation.

### Next Development
- Camera-derived visible nominal path
- Visible LEFT / NORMAL / RIGHT trajectory candidates
- Obstacle-to-path intersection checking
- Dynamic rerouting
- Smooth merge back to original camera path
