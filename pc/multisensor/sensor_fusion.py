import math


class SensorFusion:
    ZONES = (
        "LEFT_FRONT",
        "CENTER_FRONT",
        "RIGHT_FRONT",
        "LEFT_SIDE",
        "RIGHT_SIDE",
        "REAR",
    )

    def __init__(self, thresholds):
        self.thresholds = {}

        for zone in self.ZONES:
            if zone not in thresholds:
                raise ValueError(f"Missing thresholds for {zone}")

            danger = float(thresholds[zone]["danger_m"])
            caution = float(thresholds[zone]["caution_m"])

            if not math.isfinite(danger) or not math.isfinite(caution) or danger < 0 or caution <= danger:
                raise ValueError(f"Invalid thresholds for {zone}")

            self.thresholds[zone] = (danger, caution)

    def classify(self, zone, distance_m, healthy=True):
        if not healthy:
            return "INVALID"

        try:
            distance = float(distance_m)
        except (TypeError, ValueError):
            return "INVALID"

        if math.isnan(distance) or distance < 0:
            return "INVALID"

        if math.isinf(distance):
            return "CLEAR" if distance > 0 else "INVALID"

        danger, caution = self.thresholds[zone]

        if distance <= danger:
            return "DANGER"
        if distance <= caution:
            return "CAUTION"

        return "CLEAR"

    def fuse(self, readings, health=None):
        health = health or {}
        snapshot = {}

        for zone in self.ZONES:
            distance = readings.get(zone)
            snapshot[zone] = {
                "distance_m": distance,
                "status": self.classify(
                    zone,
                    distance,
                    health.get(zone, True),
                ),
            }

        return snapshot
