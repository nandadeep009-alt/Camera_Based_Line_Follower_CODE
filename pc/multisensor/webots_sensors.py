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

class WebotsProximityReader:
    """
    Read the left-side, right-side, and rear Webots DistanceSensors.

    This class does not call robot.step(). The main control loop owns
    the Webots timestep.
    """

    DEFAULT_SENSOR_NAMES = {
        "LEFT_SIDE": "left_side_obstacle",
        "RIGHT_SIDE": "right_side_obstacle",
        "REAR": "rear_obstacle",
    }

    def __init__(
        self,
        robot,
        sensor_names=None,
        max_range_m=6.0,
    ):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        self.max_range_m = float(max_range_m)
        self.sensor_names = dict(
            sensor_names or self.DEFAULT_SENSOR_NAMES
        )
        self.sensors = {}

        for zone, device_name in self.sensor_names.items():
            sensor = robot.getDevice(device_name)

            if sensor is None:
                raise RuntimeError(
                    f"Obstacle sensor '{device_name}' not found."
                )

            sensor.enable(self.timestep)
            self.sensors[zone] = sensor

            print(
                f"[MULTISENSOR] {zone} sensor initialized "
                f"('{device_name}')"
            )

    def read(self):
        """
        Return current side/rear obstacle distances in metres.

        Invalid readings are returned as math.nan so later safety logic
        can fail safe instead of treating bad data as clear space.
        """
        readings = {}

        for zone, sensor in self.sensors.items():
            try:
                value = float(sensor.getValue())
            except (TypeError, ValueError, RuntimeError):
                value = math.nan

            if not math.isfinite(value) or value < 0.0:
                readings[zone] = math.nan
                continue

            readings[zone] = min(
                value,
                self.max_range_m,
            )

        return readings

    def is_clear(self, distance):
        """
        Return True only when the reading is valid and at max range.
        """
        return (
            math.isfinite(distance)
            and distance >= self.max_range_m
        )

    def stop(self):
        """Disable all side/rear sensors cleanly."""
        for sensor in self.sensors.values():
            try:
                sensor.disable()
            except Exception:
                pass

        print("[MULTISENSOR] Side/rear sensors stopped.")
