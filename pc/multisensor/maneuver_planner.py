import math


class ManeuverPlanner:
    FORWARD = "FORWARD"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    STOP = "STOP"

    def _status(self, snapshot, zone):
        item = snapshot.get(zone)
        if not isinstance(item, dict):
            return "INVALID"
        return item.get("status", "INVALID")

    def _distance(self, snapshot, zone):
        try:
            value = float(snapshot[zone]["distance_m"])
        except (KeyError, TypeError, ValueError):
            return -1.0

        if math.isnan(value) or value < 0:
            return -1.0

        return value

    def choose(self, snapshot):
        center = self._status(snapshot, "CENTER_FRONT")

        if center == "INVALID":
            return self.STOP

        if center in ("CLEAR", "CAUTION"):
            return self.FORWARD

        if center != "DANGER":
            return self.STOP

        left_clear = (
            self._status(snapshot, "LEFT_FRONT") == "CLEAR"
            and self._status(snapshot, "LEFT_SIDE") == "CLEAR"
        )

        right_clear = (
            self._status(snapshot, "RIGHT_FRONT") == "CLEAR"
            and self._status(snapshot, "RIGHT_SIDE") == "CLEAR"
        )

        if not left_clear and not right_clear:
            return self.STOP

        if left_clear and not right_clear:
            return self.LEFT

        if right_clear and not left_clear:
            return self.RIGHT

        left_space = min(
            self._distance(snapshot, "LEFT_FRONT"),
            self._distance(snapshot, "LEFT_SIDE"),
        )

        right_space = min(
            self._distance(snapshot, "RIGHT_FRONT"),
            self._distance(snapshot, "RIGHT_SIDE"),
        )

        if left_space < 0 or right_space < 0:
            return self.STOP

        if right_space > left_space:
            return self.RIGHT

        return self.LEFT
