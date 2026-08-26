import math
import time

from .webots_sensors import WebotsLidarReader, WebotsProximityReader, WebotsPoseReader
from .sensor_zones import FrontLidarZones
from .sensor_fusion import SensorFusion
from .safety_supervisor import SafetySupervisor
from .speed_planner import SpeedPlanner
from .maneuver_planner import ManeuverPlanner
from .obstacle_pass_tracker import ObstaclePassTracker
from .trajectory_planner import TrajectoryPlanner


class MultisensorWebotsController:

    LEFT_ANGLE = 80.0
    STRAIGHT_ANGLE = 90.0
    RIGHT_ANGLE = 100.0

    def __init__(self, robot, motion_controller):
        self.robot = robot
        self.motion = motion_controller

        # Preserve RobotCommander compatibility.
        self.command_secret = motion_controller.command_secret
        self.client = motion_controller.client

        self.lidar = WebotsLidarReader(
            robot=robot,
            device_name="Sick LMS 291",
        )
        self.zones = FrontLidarZones()
        self.proximity = WebotsProximityReader(robot=robot)
        self.pose = WebotsPoseReader(robot=robot)

        self.trajectory_planner = TrajectoryPlanner(
            wheelbase_m=2.8,
            lateral_offset_m=2.5,
            min_length_m=16.0,
            max_length_m=35.0,
            min_lookahead_m=4.0,
            max_lookahead_m=10.0,
            max_steering_offset_deg=12.0,
        )

        thresholds = {
            "LEFT_FRONT": {"danger_m": 12.0, "caution_m": 25.0},
            "CENTER_FRONT": {"danger_m": 12.0, "caution_m": 25.0},
            "RIGHT_FRONT": {"danger_m": 12.0, "caution_m": 25.0},
            "LEFT_SIDE": {"danger_m": 1.5, "caution_m": 4.0},
            "RIGHT_SIDE": {"danger_m": 1.5, "caution_m": 4.0},
            "REAR": {"danger_m": 1.0, "caution_m": 2.0},
        }

        self.fusion = SensorFusion(thresholds)
        self.supervisor = SafetySupervisor()
        self.planner = SpeedPlanner(
            drive_speed_mps=8.33,
            slow_speed_mps=4.17,
            maneuver_speed_mps=2.22,
            reverse_speed_mps=0.5,
        )
        self.maneuver_planner = ManeuverPlanner()
        self.pass_tracker = ObstaclePassTracker(clear_confirm_frames=3)

        self.active_trajectory = None
        self.trajectory_progress_m = 0.0
        self.trajectory_last_position = None
        self.trajectory_pass_confirmed = False
        self.max_yaw_rate_rad_s = 1.8

        # Progressive vehicle dynamics.
        self.accel_mps2 = 1.0
        self.decel_mps2 = 3.0
        self.command_speed_mps = 0.0
        self.last_update_time = None

        # Avoidance state machine.
        self.locked_maneuver = None
        self.avoidance_phase = None
        self.avoidance_start_time = None

        self.min_turn_out_s = 0.5
        self.max_avoidance_s = 25.0
        self.hard_obstacle_clearance_m = 0.8

        # Camera takes control again after obstacle pass.
        self.reacquire_count = 0
        self.reacquire_required = 5
        self.reacquire_angle_tolerance_deg = 12.0
        self.reacquire_speed_mps = 1.5

    @property
    def connected(self):
        return self.motion.connected


    def _read_snapshot(self):
        scan = self.lidar.read_scan()

        if scan is None:
            return None

        front = self.zones.analyze(
            scan=scan,
            fov_rad=self.lidar.fov,
        )

        side = self.proximity.read()

        readings = {
            "LEFT_FRONT": front["LEFT_FRONT"],
            "CENTER_FRONT": front["CENTER_FRONT"],
            "RIGHT_FRONT": front["RIGHT_FRONT"],
            "LEFT_SIDE": side["LEFT_SIDE"],
            "RIGHT_SIDE": side["RIGHT_SIDE"],
            "REAR": side["REAR"],
        }

        return self.fusion.fuse(readings)


    def _ramp_speed(self, target_speed):
        now = float(self.robot.getTime())

        if self.last_update_time is None:
            dt = 0.1
        else:
            dt = max(
                0.02,
                min(0.25, now - self.last_update_time)
            )

        self.last_update_time = now

        if target_speed > self.command_speed_mps:
            self.command_speed_mps = min(
                target_speed,
                self.command_speed_mps + self.accel_mps2 * dt,
            )
        else:
            self.command_speed_mps = max(
                target_speed,
                self.command_speed_mps - self.decel_mps2 * dt,
            )

        return self.command_speed_mps


    def _front_danger(self, snapshot):
        return any(
            snapshot[zone]["status"] == "DANGER"
            for zone in (
                "LEFT_FRONT",
                "CENTER_FRONT",
                "RIGHT_FRONT",
            )
        )


    def _escape_corridor_clear(self, snapshot):
        if self.locked_maneuver == "RIGHT":
            zones = ("CENTER_FRONT", "RIGHT_FRONT")
        elif self.locked_maneuver == "LEFT":
            zones = ("CENTER_FRONT", "LEFT_FRONT")
        else:
            return False

        return all(
            snapshot[zone]["status"] not in {"DANGER", "INVALID"}
            for zone in zones
        )


    def _obstacle_side_clearance_safe(self, snapshot):
        if self.locked_maneuver == "LEFT":
            zones = ("RIGHT_FRONT", "RIGHT_SIDE")
        elif self.locked_maneuver == "RIGHT":
            zones = ("LEFT_FRONT", "LEFT_SIDE")
        else:
            return False

        for zone in zones:
            item = snapshot.get(zone, {})
            distance = item.get("distance_m")

            try:
                distance = float(distance)
            except (TypeError, ValueError):
                return False

            if math.isnan(distance) or distance < 0.0:
                return False

            if (
                math.isfinite(distance)
                and distance <= self.hard_obstacle_clearance_m
            ):
                return False

        return True


    def _pass_corridor_decision(self, snapshot):
        if self.locked_maneuver == "RIGHT":
            zones = (
                "CENTER_FRONT",
                "RIGHT_FRONT",
                "RIGHT_SIDE",
            )
        elif self.locked_maneuver == "LEFT":
            zones = (
                "CENTER_FRONT",
                "LEFT_FRONT",
                "LEFT_SIDE",
            )
        else:
            return "FAIL_SAFE_STOP"

        statuses = []

        for zone in zones:
            status = snapshot.get(zone, {}).get("status")

            if status not in {
                "CLEAR",
                "CAUTION",
                "DANGER",
            }:
                return "FAIL_SAFE_STOP"

            statuses.append(status)

        if "DANGER" in statuses:
            return "STOP"

        if "CAUTION" in statuses:
            return "SLOW"

        return "DRIVE_ALLOWED"


    def _front_obstacle_distance(self, snapshot):
        distances = []

        for zone in (
            "LEFT_FRONT",
            "CENTER_FRONT",
            "RIGHT_FRONT",
        ):
            distance = snapshot.get(zone, {}).get("distance_m")

            try:
                distance = float(distance)
            except (TypeError, ValueError):
                continue

            if math.isfinite(distance) and distance > 0.0:
                distances.append(distance)

        return min(distances) if distances else math.inf


    def _obstacle_side_distance(self, snapshot):
        if self.locked_maneuver == "LEFT":
            zones = ("RIGHT_FRONT", "RIGHT_SIDE")
        elif self.locked_maneuver == "RIGHT":
            zones = ("LEFT_FRONT", "LEFT_SIDE")
        else:
            return math.inf

        distances = []

        for zone in zones:
            distance = snapshot.get(zone, {}).get("distance_m")

            try:
                distance = float(distance)
            except (TypeError, ValueError):
                continue

            if math.isfinite(distance) and distance >= 0.0:
                distances.append(distance)

        return min(distances) if distances else math.inf


    def _start_trajectory(self, snapshot):
        obstacle_distance = self._front_obstacle_distance(snapshot)

        if not math.isfinite(obstacle_distance):
            return False

        pose = self.pose.read()

        if pose is None:
            return False

        try:
            self.active_trajectory = self.trajectory_planner.create_plan(
                self.locked_maneuver,
                speed_mps=max(self.command_speed_mps, 0.1),
                obstacle_distance_m=obstacle_distance,
            )
        except (TypeError, ValueError):
            return False

        self.trajectory_progress_m = 0.0
        self.trajectory_last_position = pose["position"]
        self.trajectory_pass_confirmed = False

        print(
            f"[TRAJECTORY CREATED] maneuver={self.locked_maneuver} | "
            f"obstacle={obstacle_distance:.2f} m | "
            f"length={self.active_trajectory['length_m']:.2f} m | "
            f"offset={self.active_trajectory['offset_m']:.2f} m | "
            f"lookahead={self.active_trajectory['lookahead_m']:.2f} m"
        )

        return True


    def _update_trajectory_progress(self):
        pose = self.pose.read()

        if pose is None:
            return None

        position = pose["position"]

        if self.trajectory_last_position is not None:
            dx = position[0] - self.trajectory_last_position[0]
            dy = position[1] - self.trajectory_last_position[1]
            dz = position[2] - self.trajectory_last_position[2]

            step_distance = math.sqrt(
                dx * dx + dy * dy + dz * dz
            )

            if not math.isfinite(step_distance) or step_distance > 2.5:
                return None

            self.trajectory_progress_m += step_distance

        self.trajectory_last_position = position

        return (
            self.trajectory_progress_m,
            pose["yaw_rate_rad_s"],
        )


    def _sensor_pov(self, snapshot):
        zones = (
            "LEFT_FRONT",
            "CENTER_FRONT",
            "RIGHT_FRONT",
            "LEFT_SIDE",
            "RIGHT_SIDE",
            "REAR",
        )

        parts = []

        for zone in zones:
            item = snapshot.get(zone, {})
            status = item.get("status", "INVALID")
            distance = item.get("distance_m")

            if isinstance(distance, (int, float)) and math.isfinite(distance):
                value = f"{distance:.2f}m"
            else:
                value = "CLEAR/INF"

            parts.append(
                f"{zone}={status}:{value}"
            )

        return " | ".join(parts)


    def _all_sensors_clear(self, snapshot):
        zones = (
            "LEFT_FRONT",
            "CENTER_FRONT",
            "RIGHT_FRONT",
            "LEFT_SIDE",
            "RIGHT_SIDE",
            "REAR",
        )

        return all(
            snapshot.get(zone, {}).get("status") == "CLEAR"
            for zone in zones
        )


    def _reset_avoidance(self):
        self.locked_maneuver = None
        self.avoidance_phase = None
        self.avoidance_start_time = None
        self.reacquire_count = 0
        self.pass_tracker.reset()


    def _stop_now(self, reason):
        self.command_speed_mps = 0.0
        self.last_update_time = None
        self.motion.send_motion_command(
            self.STRAIGHT_ANGLE,
            0.0,
        )

        print(f"[MULTISENSOR STOP] {reason}")


    def send_command(self, camera_angle, engine_state):
        try:
            camera_angle = float(camera_angle)
        except (TypeError, ValueError):
            self._stop_now("invalid camera steering")
            return

        if not math.isfinite(camera_angle):
            self._stop_now("non-finite camera steering")
            return

        state = str(engine_state).upper()

        # Preserve the existing RobotCommander fail-safe behaviour.
        if state == "STOP":
            self._reset_avoidance()
            self._stop_now("vision requested STOP")
            return

        snapshot = self._read_snapshot()

        if snapshot is None:
            self._stop_now("LiDAR snapshot unavailable")
            return

        now = float(self.robot.getTime())
        planned_maneuver = self.maneuver_planner.choose(snapshot)

        # ---------------------------------------------------------
        # START OBSTACLE AVOIDANCE
        # ---------------------------------------------------------

        if (
            self.locked_maneuver is None
            and state == "DRIVE"
            and planned_maneuver in {"LEFT", "RIGHT"}
        ):
            self.locked_maneuver = planned_maneuver
            self.avoidance_phase = "TRAJECTORY"
            self.avoidance_start_time = now
            self.pass_tracker.start(self.locked_maneuver)
            self.trajectory_pass_confirmed = False

            if not self._start_trajectory(snapshot):
                self._stop_now(
                    "trajectory initialization failed"
                )
                self.locked_maneuver = None
                return

            print(
                f"[MULTISENSOR] avoidance locked: "
                f"{self.locked_maneuver}"
            )

        # ---------------------------------------------------------
        # ACTIVE OBSTACLE AVOIDANCE
        # ---------------------------------------------------------

        if self.locked_maneuver is not None:

            elapsed = now - self.avoidance_start_time

            if elapsed >= self.max_avoidance_s:
                self._stop_now("avoidance timeout")
                self._reset_avoidance()
                return

            outer_zone = (
                "RIGHT_SIDE"
                if self.locked_maneuver == "RIGHT"
                else "LEFT_SIDE"
            )

            outer_status = snapshot[outer_zone]["status"]

            if outer_status in {"DANGER", "INVALID"}:
                self._stop_now(
                    f"outer boundary unsafe: {outer_zone}"
                )
                return

            print(
                f"[SENSOR POV] {self._sensor_pov(snapshot)}"
            )

            if not self._obstacle_side_clearance_safe(snapshot):
                self._stop_now(
                    "critical obstacle-side clearance"
                )
                return

            passed = self.pass_tracker.update(snapshot)

            if (
                self.avoidance_phase == "TURN_OUT"
                and elapsed >= self.min_turn_out_s
                and self._escape_corridor_clear(snapshot)
            ):
                self.avoidance_phase = "PASS_STRAIGHT"
                print("[MULTISENSOR] phase: PASS_STRAIGHT")

            if passed and not self.trajectory_pass_confirmed:
                self.trajectory_pass_confirmed = True
                print(
                    "[MULTISENSOR] obstacle PASS sensor-confirmed"
                )

            # -----------------------------------------------------
            # GPS + GYRO SUPERVISED SMOOTH TRAJECTORY
            # -----------------------------------------------------

            if self.avoidance_phase == "TRAJECTORY":

                trajectory_update = (
                    self._update_trajectory_progress()
                )

                if trajectory_update is None:
                    self._stop_now(
                        "GPS trajectory progress invalid"
                    )
                    return

                progress_m, yaw_rate = trajectory_update

                if abs(yaw_rate) > self.max_yaw_rate_rad_s:
                    self._stop_now(
                        "trajectory yaw rate unsafe"
                    )
                    return

                maneuver = self.locked_maneuver

                decision = self.supervisor.decide(
                    snapshot,
                    maneuver,
                )

                target_speed = self.planner.target_speed(
                    decision,
                    maneuver,
                )

                if target_speed <= 0.0:
                    self._stop_now(
                        "trajectory corridor unsafe"
                    )
                    return

                trajectory_complete = (
                    self.trajectory_planner.is_complete(
                        self.active_trajectory,
                        progress_m,
                    )
                )

                if trajectory_complete:

                    if not self.trajectory_pass_confirmed:
                        self._stop_now(
                            "trajectory complete without "
                            "obstacle-pass confirmation"
                        )
                        return

                    self.avoidance_phase = "REACQUIRE"
                    self.reacquire_count = 0

                    print(
                        "[TRAJECTORY COMPLETE] geometric path complete"
                    )
                    print(
                        "[MULTISENSOR] phase: REACQUIRE"
                    )

                else:

                    steering_angle = (
                        self.trajectory_planner.steering_angle(
                            self.active_trajectory,
                            progress_m=progress_m,
                            camera_angle=camera_angle,
                        )
                    )

                    obstacle_distance = (
                        self._obstacle_side_distance(snapshot)
                    )

                    # Do not counter-steer back toward an obstacle
                    # while it is still close to the vehicle.
                    if obstacle_distance <= 4.0:

                        if (
                            maneuver == "LEFT"
                            and steering_angle > 90.0
                        ):
                            steering_angle = 90.0

                        elif (
                            maneuver == "RIGHT"
                            and steering_angle < 90.0
                        ):
                            steering_angle = 90.0

                    command_speed = self._ramp_speed(
                        min(target_speed, 2.22)
                    )

                    self.motion.send_motion_command(
                        steering_angle,
                        command_speed,
                    )

                    print(
                        f"[TRAJECTORY] {maneuver} | "
                        f"progress={progress_m:.2f}/"
                        f"{self.active_trajectory['length_m']:.2f} m | "
                        f"steering={steering_angle:.1f} | "
                        f"speed={command_speed:.2f} m/s | "
                        f"yaw_rate={yaw_rate:.3f} rad/s | "
                        f"pass_confirmed="
                        f"{self.trajectory_pass_confirmed}"
                    )

                    return


            # -----------------------------------------------------
            # TURN OUT
            # -----------------------------------------------------

            if self.avoidance_phase == "TURN_OUT":

                maneuver = self.locked_maneuver

                decision = self.supervisor.decide(
                    snapshot,
                    maneuver,
                )

                target_speed = self.planner.target_speed(
                    decision,
                    maneuver,
                )

                if target_speed <= 0.0:
                    self._stop_now("avoidance path unsafe")
                    return

                steering_angle = (
                    self.LEFT_ANGLE
                    if maneuver == "LEFT"
                    else self.RIGHT_ANGLE
                )

                command_speed = self._ramp_speed(target_speed)

                self.motion.send_motion_command(
                    steering_angle,
                    command_speed,
                )

                print(
                    f"[MULTISENSOR] TURN_OUT {maneuver} | "
                    f"target={target_speed:.2f} m/s | "
                    f"ramped={command_speed:.2f} m/s"
                )
                return

            # -----------------------------------------------------
            # PASS ALONGSIDE OBSTACLE
            # -----------------------------------------------------

            if self.avoidance_phase == "PASS_STRAIGHT":

                decision = self._pass_corridor_decision(
                    snapshot
                )

                target_speed = min(
                    self.planner.target_speed(
                        decision,
                        "FORWARD",
                    ),
                    2.0,
                )

                if target_speed <= 0.0:
                    self._stop_now("forward pass path unsafe")
                    return

                command_speed = self._ramp_speed(target_speed)

                # Camera guides the vehicle back parallel to the road,
                # but correction is limited while beside the obstacle.
                if self.locked_maneuver == "LEFT":
                    obstacle_zones = (
                        "RIGHT_FRONT",
                        "RIGHT_SIDE",
                    )
                elif self.locked_maneuver == "RIGHT":
                    obstacle_zones = (
                        "LEFT_FRONT",
                        "LEFT_SIDE",
                    )
                else:
                    self._stop_now(
                        "invalid avoidance direction"
                    )
                    return

                obstacle_distances = []

                for zone in obstacle_zones:
                    distance = snapshot.get(
                        zone, {}
                    ).get("distance_m")

                    try:
                        distance = float(distance)
                    except (TypeError, ValueError):
                        continue

                    if math.isfinite(distance) and distance >= 0.0:
                        obstacle_distances.append(distance)

                obstacle_distance = (
                    min(obstacle_distances)
                    if obstacle_distances
                    else math.inf
                )

                if obstacle_distance <= 4.0:
                    # Actively maintain lateral clearance.
                    if self.locked_maneuver == "LEFT":
                        guided_angle = 84.0
                    else:
                        guided_angle = 96.0

                    print(
                        f"[CLEARANCE CORRECTION] "
                        f"obstacle={obstacle_distance:.2f} m | "
                        f"command={guided_angle:.1f}"
                    )

                elif self.locked_maneuver == "LEFT":
                    guided_angle = max(
                        80.0,
                        min(94.0, camera_angle)
                    )

                else:
                    guided_angle = max(
                        86.0,
                        min(100.0, camera_angle)
                    )

                self.motion.send_motion_command(
                    guided_angle,
                    command_speed,
                )

                print(
                    f"[MULTISENSOR] PASS_GUIDED | "
                    f"camera={camera_angle:.1f} | "
                    f"command={guided_angle:.1f} | "
                    f"speed={command_speed:.2f} m/s"
                )
                return

            # -----------------------------------------------------
            # GIVE CONTROL BACK TO CAMERA
            # -----------------------------------------------------

            if self.avoidance_phase == "REACQUIRE":

                if state != "DRIVE":
                    self.reacquire_count = 0
                    self._stop_now(
                        "waiting for camera line reacquisition"
                    )
                    return

                line_stable = (
                    abs(camera_angle - 90.0)
                    <= self.reacquire_angle_tolerance_deg
                )

                sensors_clear = self._all_sensors_clear(
                    snapshot
                )

                if line_stable and sensors_clear:
                    self.reacquire_count += 1
                else:
                    self.reacquire_count = 0

                command_speed = self._ramp_speed(
                    self.reacquire_speed_mps
                )

                self.motion.send_motion_command(
                    camera_angle,
                    command_speed,
                )

                print(
                    f"[MULTISENSOR] REACQUIRE | "
                    f"angle={camera_angle:.1f} | "
                    f"stable={self.reacquire_count}/"
                    f"{self.reacquire_required} | "
                    f"sensors_clear={sensors_clear} | "
                    f"speed={command_speed:.2f} m/s"
                )

                if (
                    self.reacquire_count
                    >= self.reacquire_required
                ):
                    print("[MULTISENSOR] line reacquired")
                    print(
                        "[MANEUVER COMPLETE] obstacle passed | "
                        "front CLEAR | sides CLEAR | rear CLEAR | "
                        "camera track CONFIRMED"
                    )
                    print(
                        f"[FINAL SENSOR POV] {self._sensor_pov(snapshot)}"
                    )
                    self._reset_avoidance()

                return

        # ---------------------------------------------------------
        # EXISTING VISION REVERSE RECOVERY
        # ---------------------------------------------------------

        if state == "REVERSE":

            decision = self.supervisor.decide(
                snapshot,
                "REVERSE",
            )

            target_speed = self.planner.target_speed(
                decision,
                "REVERSE",
            )

            if target_speed <= 0.0:
                self._stop_now("reverse path unsafe")
                return

            command_speed = self._ramp_speed(target_speed)

            self.motion.send_motion_command(
                camera_angle,
                command_speed,
                reverse=True,
            )

            print(
                f"[MULTISENSOR] REVERSE | "
                f"speed={command_speed:.2f} m/s"
            )
            return

        # Unknown vision state remains fail-safe.
        if state != "DRIVE":
            self._stop_now(
                f"unsupported vision state: {state}"
            )
            return

        # If an obstacle blocks all available paths, STOP.
        if planned_maneuver == "STOP":
            self._stop_now("no safe obstacle escape path")
            return

        # ---------------------------------------------------------
        # NORMAL CAMERA / LINE FOLLOWING
        # ---------------------------------------------------------

        decision = self.supervisor.decide(
            snapshot,
            "FORWARD",
        )

        target_speed = self.planner.target_speed(
            decision,
            "FORWARD",
        )

        if target_speed <= 0.0:
            self._stop_now("forward safety decision STOP")
            return

        command_speed = self._ramp_speed(target_speed)

        self.motion.send_motion_command(
            camera_angle,
            command_speed,
        )

        print(
            f"[MULTISENSOR] CAMERA DRIVE | "
            f"angle={camera_angle:.1f} | "
            f"target={target_speed:.2f} m/s | "
            f"ramped={command_speed:.2f} m/s"
        )
