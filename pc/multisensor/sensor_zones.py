# pc/multisensor/sensor_zones.py

"""
ROVE LiDAR Sensor Zoning

Purpose:
    Convert one complete front LiDAR scan into meaningful
    obstacle-distance zones.

Confirmed Webots orientation:
    Negative angle = vehicle LEFT
    Zero angle     = straight ahead
    Positive angle = vehicle RIGHT
"""

import math


class FrontLidarZones:

    def __init__(
        self,
        left_min_deg=-60.0,
        left_max_deg=-20.0,
        center_min_deg=-20.0,
        center_max_deg=20.0,
        right_min_deg=20.0,
        right_max_deg=60.0,
    ):

        self.left_min_deg = float(left_min_deg)
        self.left_max_deg = float(left_max_deg)

        self.center_min_deg = float(center_min_deg)
        self.center_max_deg = float(center_max_deg)

        self.right_min_deg = float(right_min_deg)
        self.right_max_deg = float(right_max_deg)

    # -----------------------------------------------------------------
    # ANGLE OF ONE LIDAR BEAM
    # -----------------------------------------------------------------

    @staticmethod
    def _beam_angle(index, resolution, fov_rad):

        if resolution <= 1:
            return 0.0

        fov_deg = math.degrees(fov_rad)

        step = (
            fov_deg
            / (resolution - 1)
        )

        return (
            -fov_deg / 2.0
            + index * step
        )

    # -----------------------------------------------------------------
    # MINIMUM VALID DISTANCE
    # -----------------------------------------------------------------

    @staticmethod
    def _minimum(values):

        valid = [
            distance
            for distance in values
            if math.isfinite(distance)
        ]

        if not valid:
            return math.inf

        return min(valid)

    # -----------------------------------------------------------------
    # ANALYZE COMPLETE SCAN
    # -----------------------------------------------------------------

    def analyze(
        self,
        scan,
        fov_rad,
    ):

        if not scan:

            return {
                "LEFT_FRONT": math.inf,
                "CENTER_FRONT": math.inf,
                "RIGHT_FRONT": math.inf,
            }

        resolution = len(scan)

        left_values = []
        center_values = []
        right_values = []

        for index, distance in enumerate(scan):

            if not math.isfinite(distance):
                continue

            angle = self._beam_angle(
                index,
                resolution,
                fov_rad,
            )

            if (
                self.left_min_deg
                <= angle
                < self.left_max_deg
            ):
                left_values.append(distance)

            elif (
                self.center_min_deg
                <= angle
                <= self.center_max_deg
            ):
                center_values.append(distance)

            elif (
                self.right_min_deg
                < angle
                <= self.right_max_deg
            ):
                right_values.append(distance)

        return {
            "LEFT_FRONT": self._minimum(
                left_values
            ),

            "CENTER_FRONT": self._minimum(
                center_values
            ),

            "RIGHT_FRONT": self._minimum(
                right_values
            ),
        }