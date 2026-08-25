import math


class SpeedPlanner:
    def __init__(
        self,
        drive_speed_mps=3.0,
        slow_speed_mps=1.0,
        maneuver_speed_mps=0.8,
        reverse_speed_mps=0.5,
    ):
        self.drive_speed_mps = float(drive_speed_mps)
        self.slow_speed_mps = float(slow_speed_mps)
        self.maneuver_speed_mps = float(maneuver_speed_mps)
        self.reverse_speed_mps = float(reverse_speed_mps)

        speeds = (
            self.drive_speed_mps,
            self.slow_speed_mps,
            self.maneuver_speed_mps,
            self.reverse_speed_mps,
        )

        if not all(math.isfinite(value) and value >= 0.0 for value in speeds):
            raise ValueError("Invalid speed configuration")

        if self.slow_speed_mps > self.drive_speed_mps:
            raise ValueError("Slow speed cannot exceed drive speed")

        if self.maneuver_speed_mps > self.drive_speed_mps:
            raise ValueError("Maneuver speed cannot exceed drive speed")

        if self.reverse_speed_mps > self.drive_speed_mps:
            raise ValueError("Reverse speed cannot exceed drive speed")

    def target_speed(self, safety_decision, maneuver="FORWARD"):
        if safety_decision in ("STOP", "FAIL_SAFE_STOP"):
            return 0.0

        if maneuver == "STOP":
            return 0.0

        if maneuver not in ("FORWARD", "LEFT", "RIGHT", "REVERSE"):
            return 0.0

        if safety_decision == "SLOW":
            if maneuver == "FORWARD":
                return self.slow_speed_mps
            if maneuver in ("LEFT", "RIGHT"):
                return min(self.slow_speed_mps, self.maneuver_speed_mps)
            return min(self.slow_speed_mps, self.reverse_speed_mps)

        if safety_decision != "DRIVE_ALLOWED":
            return 0.0

        if maneuver == "FORWARD":
            return self.drive_speed_mps

        if maneuver in ("LEFT", "RIGHT"):
            return self.maneuver_speed_mps

        return self.reverse_speed_mps
