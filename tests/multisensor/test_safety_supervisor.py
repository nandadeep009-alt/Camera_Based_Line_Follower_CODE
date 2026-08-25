import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "pc"))

from multisensor.safety_supervisor import SafetySupervisor


supervisor = SafetySupervisor()


def snapshot(default="CLEAR"):
    zones = (
        "LEFT_FRONT",
        "CENTER_FRONT",
        "RIGHT_FRONT",
        "LEFT_SIDE",
        "RIGHT_SIDE",
        "REAR",
    )
    return {zone: {"status": default} for zone in zones}


s = snapshot()
assert supervisor.decide(s, "FORWARD") == "DRIVE_ALLOWED"

s = snapshot()
s["CENTER_FRONT"]["status"] = "CAUTION"
assert supervisor.decide(s, "FORWARD") == "SLOW"

s = snapshot()
s["CENTER_FRONT"]["status"] = "DANGER"
assert supervisor.decide(s, "FORWARD") == "STOP"

s = snapshot()
s["CENTER_FRONT"]["status"] = "INVALID"
assert supervisor.decide(s, "FORWARD") == "FAIL_SAFE_STOP"

s = snapshot()
s["REAR"]["status"] = "DANGER"
assert supervisor.decide(s, "FORWARD") == "DRIVE_ALLOWED"
assert supervisor.decide(s, "REVERSE") == "STOP"

s = snapshot()
s["LEFT_SIDE"]["status"] = "DANGER"
assert supervisor.decide(s, "LEFT") == "STOP"
assert supervisor.decide(s, "RIGHT") == "DRIVE_ALLOWED"

s = snapshot()
s["RIGHT_SIDE"]["status"] = "DANGER"
assert supervisor.decide(s, "RIGHT") == "STOP"

assert supervisor.decide({}, "FORWARD") == "FAIL_SAFE_STOP"
assert supervisor.decide(snapshot(), "UNKNOWN") == "FAIL_SAFE_STOP"

print("[PASS] SafetySupervisor offline tests passed")
