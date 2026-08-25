import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "pc"))
from multisensor.sensor_fusion import SensorFusion

THRESHOLDS = {zone: {"danger_m": 1.0, "caution_m": 2.0} for zone in SensorFusion.ZONES}
fusion = SensorFusion(THRESHOLDS)

cases = [(0.5,"DANGER"),(1.0,"DANGER"),(1.5,"CAUTION"),(2.0,"CAUTION"),(3.0,"CLEAR"),(math.inf,"CLEAR"),(math.nan,"INVALID"),(-1.0,"INVALID")]

for value, expected in cases:
    actual = fusion.classify("CENTER_FRONT", value)
    print(f"{str(value):>6} -> {actual}")
    assert actual == expected

assert fusion.classify("CENTER_FRONT", 5.0, healthy=False) == "INVALID"

snapshot = fusion.fuse({zone: 3.0 for zone in SensorFusion.ZONES})
assert all(item["status"] == "CLEAR" for item in snapshot.values())

missing = fusion.fuse({})
assert all(item["status"] == "INVALID" for item in missing.values())

try:
    bad = dict(THRESHOLDS)
    bad["CENTER_FRONT"] = {"danger_m": math.nan, "caution_m": 2.0}
    SensorFusion(bad)
    raise AssertionError("Invalid thresholds accepted")
except ValueError:
    pass

print("[PASS] SensorFusion offline safety tests passed")
