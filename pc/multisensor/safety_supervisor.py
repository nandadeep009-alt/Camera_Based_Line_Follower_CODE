class SafetySupervisor:
    DRIVE_ALLOWED = "DRIVE_ALLOWED"
    SLOW = "SLOW"
    STOP = "STOP"
    FAIL_SAFE_STOP = "FAIL_SAFE_STOP"

    MANEUVER_ZONES = {
        "FORWARD": ("LEFT_FRONT", "CENTER_FRONT", "RIGHT_FRONT"),
        "LEFT": ("LEFT_FRONT", "CENTER_FRONT", "LEFT_SIDE"),
        "RIGHT": ("RIGHT_FRONT", "CENTER_FRONT", "RIGHT_SIDE"),
        "REVERSE": ("LEFT_SIDE", "RIGHT_SIDE", "REAR"),
    }

    VALID_STATUSES = {"CLEAR", "CAUTION", "DANGER", "INVALID"}

    def decide(self, snapshot, maneuver="FORWARD"):
        if maneuver not in self.MANEUVER_ZONES:
            return self.FAIL_SAFE_STOP

        zones = self.MANEUVER_ZONES[maneuver]
        statuses = []

        for zone in zones:
            item = snapshot.get(zone)

            if not isinstance(item, dict):
                return self.FAIL_SAFE_STOP

            status = item.get("status")

            if status not in self.VALID_STATUSES:
                return self.FAIL_SAFE_STOP

            statuses.append(status)

        if "INVALID" in statuses:
            return self.FAIL_SAFE_STOP

        if "DANGER" in statuses:
            return self.STOP

        if "CAUTION" in statuses:
            return self.SLOW

        return self.DRIVE_ALLOWED
