# pc/multisensor/webots_sensors.py

"""
ROVE Webots Sensor Interface

Purpose:
    Provide a clean interface between Webots sensors and the future
    multi-sensor safety/fusion system.

Current implementation:
    - Front SICK LMS 291 LiDAR only.

Important:
    This module does NOT call robot.step().
    The main controller must advance Webots exactly once per control cycle.
"""

import math


class WebotsLidarReader:
    """
    Reads and validates the front SICK LMS 291 LiDAR.
    """

    def __init__(
        self,
        robot,
        device_name="Sick LMS 291",
    ):
        self.robot = robot
        self.device_name = device_name

        self.timestep = int(
            robot.getBasicTimeStep()
        )

        # -------------------------------------------------------------
        # GET LIDAR DEVICE
        # -------------------------------------------------------------

        self.lidar = robot.getDevice(
            self.device_name
        )

        if self.lidar is None:
            raise RuntimeError(
                f"LiDAR '{self.device_name}' not found."
            )

        # -------------------------------------------------------------
        # ENABLE LIDAR
        # -------------------------------------------------------------

        self.lidar.enable(
            self.timestep
        )

        # -------------------------------------------------------------
        # READ SENSOR SPECIFICATIONS
        # -------------------------------------------------------------

        self.horizontal_resolution = int(
            self.lidar.getHorizontalResolution()
        )

        self.fov = float(
            self.lidar.getFov()
        )

        self.min_range = float(
            self.lidar.getMinRange()
        )

        self.max_range = float(
            self.lidar.getMaxRange()
        )

        print()
        print("[MULTISENSOR] Front LiDAR initialized")
        print(
            f"[MULTISENSOR] device="
            f"'{self.device_name}'"
        )
        print(
            f"[MULTISENSOR] resolution="
            f"{self.horizontal_resolution}"
        )
        print(
            f"[MULTISENSOR] FOV="
            f"{math.degrees(self.fov):.1f} deg"
        )
        print(
            f"[MULTISENSOR] range="
            f"{self.min_range:.2f} m "
            f"to {self.max_range:.2f} m"
        )

    # -----------------------------------------------------------------
    # RAW SCAN
    # -----------------------------------------------------------------

    def read_scan(self):
        """
        Read one complete LiDAR horizontal scan.

        Returns:
            list[float]

        Valid obstacle distances are returned in metres.

        Invalid measurements are converted to infinity.
        """

        raw_scan = self.lidar.getRangeImage()

        if raw_scan is None:
            return None

        if len(raw_scan) != self.horizontal_resolution:
            print(
                "[MULTISENSOR WARNING] "
                "Unexpected LiDAR scan length."
            )
            return None

        clean_scan = []

        for distance in raw_scan:

            try:
                distance = float(distance)

            except (TypeError, ValueError):
                clean_scan.append(
                    math.inf
                )
                continue

            if (
                math.isnan(distance)
                or distance <= 0.0
            ):
                clean_scan.append(
                    math.inf
                )
                continue

            clean_scan.append(
                distance
            )

        return clean_scan

    # -----------------------------------------------------------------
    # CLOSEST VALID OBJECT
    # -----------------------------------------------------------------

    def closest_distance(self, scan):
        """
        Return the closest valid LiDAR measurement.
        """

        if not scan:
            return math.inf

        valid = [
            distance
            for distance in scan
            if math.isfinite(distance)
        ]

        if not valid:
            return math.inf

        return min(valid)

    # -----------------------------------------------------------------
    # STOP SENSOR
    # -----------------------------------------------------------------

    def stop(self):
        """
        Disable the LiDAR cleanly.
        """

        try:
            self.lidar.disable()

        except Exception:
            pass

        print(
            "[MULTISENSOR] Front LiDAR stopped."
        )