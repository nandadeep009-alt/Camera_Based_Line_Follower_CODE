import math


class TrajectoryPlanner:
    NORMAL = "NORMAL"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    STOP = "STOP"

    def __init__(
        self,
        wheelbase_m=2.8,
        lateral_offset_m=2.5,
        min_length_m=16.0,
        max_length_m=35.0,
        min_lookahead_m=4.0,
        max_lookahead_m=10.0,
        max_steering_offset_deg=12.0,
    ):
        self.wheelbase_m = float(wheelbase_m)
        self.lateral_offset_m = float(lateral_offset_m)
        self.min_length_m = float(min_length_m)
        self.max_length_m = float(max_length_m)
        self.min_lookahead_m = float(min_lookahead_m)
        self.max_lookahead_m = float(max_lookahead_m)
        self.max_steering_offset_deg = float(
            max_steering_offset_deg
        )

        values = (
            self.wheelbase_m,
            self.lateral_offset_m,
            self.min_length_m,
            self.max_length_m,
            self.min_lookahead_m,
            self.max_lookahead_m,
            self.max_steering_offset_deg,
        )

        if not all(math.isfinite(v) and v > 0.0 for v in values):
            raise ValueError("Trajectory parameters must be finite and > 0")

        if self.max_length_m < self.min_length_m:
            raise ValueError("max_length_m must be >= min_length_m")

        if self.max_lookahead_m < self.min_lookahead_m:
            raise ValueError("max_lookahead_m must be >= min_lookahead_m")


    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))


    @staticmethod
    def _smoothstep(value):
        value = max(0.0, min(1.0, value))
        return value * value * (3.0 - 2.0 * value)


    def create_plan(
        self,
        maneuver,
        speed_mps,
        obstacle_distance_m,
    ):
        maneuver = str(maneuver).upper()

        if maneuver not in {
            self.LEFT,
            self.RIGHT,
            self.STOP,
        }:
            raise ValueError(
                f"Unsupported trajectory maneuver: {maneuver}"
            )

        speed_mps = float(speed_mps)
        obstacle_distance_m = float(obstacle_distance_m)

        if (
            not math.isfinite(speed_mps)
            or speed_mps < 0.0
            or not math.isfinite(obstacle_distance_m)
            or obstacle_distance_m <= 0.0
        ):
            raise ValueError("Invalid trajectory inputs")

        if maneuver == self.STOP:
            return {
                "maneuver": self.STOP,
                "length_m": 0.0,
                "offset_m": 0.0,
                "lookahead_m": self.min_lookahead_m,
            }

        # Put maximum lateral displacement approximately at
        # the obstacle longitudinal position.
        length_m = self._clamp(
            2.0 * obstacle_distance_m,
            self.min_length_m,
            self.max_length_m,
        )

        # Faster vehicle = look farther ahead.
        lookahead_m = self._clamp(
            4.0 + 0.75 * speed_mps,
            self.min_lookahead_m,
            self.max_lookahead_m,
        )

        offset_m = (
            self.lateral_offset_m
            if maneuver == self.LEFT
            else -self.lateral_offset_m
        )

        return {
            "maneuver": maneuver,
            "length_m": length_m,
            "offset_m": offset_m,
            "lookahead_m": lookahead_m,
        }


    def lateral_offset(self, plan, distance_m):
        length_m = float(plan["length_m"])

        if length_m <= 0.0:
            return 0.0

        s = self._clamp(
            float(distance_m) / length_m,
            0.0,
            1.0,
        )

        # Smooth bump:
        # 0 m offset at beginning
        # maximum offset halfway
        # 0 m offset at end
        shape = 16.0 * s * s * (1.0 - s) * (1.0 - s)

        return float(plan["offset_m"]) * shape


    def steering_angle(
        self,
        plan,
        progress_m,
        camera_angle=90.0,
    ):
        if plan["maneuver"] == self.STOP:
            return 90.0

        length_m = float(plan["length_m"])
        progress_m = self._clamp(
            float(progress_m),
            0.0,
            length_m,
        )

        if progress_m >= length_m:
            return self._clamp(
                float(camera_angle),
                70.0,
                110.0,
            )

        lookahead = float(plan["lookahead_m"])

        target_distance = min(
            length_m,
            progress_m + lookahead,
        )

        x = max(
            0.1,
            target_distance - progress_m,
        )

        current_y = self.lateral_offset(
            plan,
            progress_m,
        )

        target_y = self.lateral_offset(
            plan,
            target_distance,
        )

        relative_y = target_y - current_y

        # Pure-pursuit style steering.
        steering_rad = math.atan2(
            2.0 * self.wheelbase_m * relative_y,
            x * x + relative_y * relative_y,
        )

        steering_offset_deg = math.degrees(
            steering_rad
        )

        steering_offset_deg = self._clamp(
            steering_offset_deg,
            -self.max_steering_offset_deg,
            self.max_steering_offset_deg,
        )

        # Existing Webots convention:
        # <90 = LEFT
        # >90 = RIGHT
        trajectory_angle = (
            90.0 - steering_offset_deg
        )

        # Blend camera back in only near the end of the trajectory.
        progress_ratio = (
            progress_m / length_m
            if length_m > 0.0
            else 1.0
        )

        if progress_ratio <= 0.65:
            camera_blend = 0.0
        else:
            camera_blend = self._smoothstep(
                (progress_ratio - 0.65) / 0.35
            )

        safe_camera_angle = self._clamp(
            float(camera_angle),
            70.0,
            110.0,
        )

        final_angle = (
            (1.0 - camera_blend) * trajectory_angle
            + camera_blend * safe_camera_angle
        )

        return self._clamp(
            final_angle,
            90.0 - self.max_steering_offset_deg,
            90.0 + self.max_steering_offset_deg,
        )


    @staticmethod
    def is_complete(plan, progress_m):
        return (
            plan["maneuver"] == "STOP"
            or float(progress_m) >= float(plan["length_m"])
        )
