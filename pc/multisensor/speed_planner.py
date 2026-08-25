import math


class SpeedPlanner:
    def __init__(self, drive_speed_mps=3.0, slow_speed_mps=1.0):
        self.drive_speed_mps = float(drive_speed_mps)
        self.slow_speed_mps = float(slow_speed_mps)

        if not math.isfinite(self.drive_speed_mps):
            raise ValueError("Invalid drive speed")

        if not math.isfinite(self.slow_speed_mps):
            raise ValueError("Invalid slow speed")

        if self.drive_speed_mps < 0.0:
            raise ValueError("Drive speed cannot be negative")

        if self.slow_speed_mps < 0.0:
            raise ValueError("Slow speed cannot be negative")

        if self.slow_speed_mps > self.drive_speed_mps:
            raise ValueError("Slow speed cannot exceed drive speed")

    def target_speed(self, safety_decision):
        if safety_decision == "DRIVE_ALLOWED":
            return self.drive_speed_mps

        if safety_decision == "SLOW":
            return self.slow_speed_mps

        if safety_decision in ("STOP", "FAIL_SAFE_STOP"):
            return 0.0

        return 0.0
