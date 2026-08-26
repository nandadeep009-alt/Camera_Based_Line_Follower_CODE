class ObstaclePassTracker:
    WAIT_SIDE = "WAIT_SIDE"
    TRACK_SIDE = "TRACK_SIDE"
    PASSED = "PASSED"

    def __init__(self, clear_confirm_frames=3):
        self.clear_confirm_frames = int(clear_confirm_frames)
        if self.clear_confirm_frames < 1:
            raise ValueError("clear_confirm_frames must be >= 1")
        self.reset()

    def reset(self):
        self.active = False
        self.side_zone = None
        self.state = self.WAIT_SIDE
        self.clear_count = 0

    def start(self, maneuver):
        if maneuver == "LEFT":
            self.side_zone = "RIGHT_SIDE"
        elif maneuver == "RIGHT":
            self.side_zone = "LEFT_SIDE"
        else:
            self.reset()
            return False

        self.active = True
        self.state = self.WAIT_SIDE
        self.clear_count = 0
        return True

    def update(self, snapshot):
        if not self.active:
            return False

        zone = snapshot.get(self.side_zone, {})
        status = zone.get("status")

        if status not in {"CLEAR", "CAUTION", "DANGER", "INVALID"}:
            self.clear_count = 0
            return False

        if status == "INVALID":
            self.clear_count = 0
            return False

        if self.state == self.WAIT_SIDE:
            if status in {"CAUTION", "DANGER"}:
                self.state = self.TRACK_SIDE
            return False

        if self.state == self.TRACK_SIDE:
            if status == "CLEAR":
                self.clear_count += 1
                if self.clear_count >= self.clear_confirm_frames:
                    self.state = self.PASSED
                    return True
            else:
                self.clear_count = 0

        return self.state == self.PASSED
