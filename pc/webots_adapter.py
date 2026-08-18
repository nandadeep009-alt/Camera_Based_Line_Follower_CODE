import time
import numpy as np


class WebotsCameraAdapter:
    def __init__(self, robot, camera_name="camera", stale_timeout=1.0):
        self.robot = robot
        self.camera = robot.getDevice(camera_name)

        if self.camera is None:
            raise RuntimeError(
                f"Webots camera '{camera_name}' not found."
            )

        self.timestep = int(robot.getBasicTimeStep())
        self.stale_timeout = float(stale_timeout)
        self.last_frame_time = 0.0
        self.stopped = False

        self.camera.enable(self.timestep)

        print(
            f"[WEBOTS CAMERA] '{camera_name}' initialized "
            f"({self.camera.getWidth()}x{self.camera.getHeight()})"
        )

    def read(self):
        if self.stopped:
            return False, None
        print("[WEBOTS CAMERA] waiting for simulation step...", flush=True)
        if self.robot.step() == -1:
            self.stopped = True
            return False, None
        print("[WEBOTS CAMERA] simulation step received.", flush=True)
        try:
            raw = self.camera.getImage()
        except (ValueError, RuntimeError) as exc:
            print(f"[WEBOTS CAMERA] Image not available yet: {exc}")
            return False, None

        if raw is None:
            return False, None

        width = self.camera.getWidth()
        height = self.camera.getHeight()

        expected = width * height * 4

        if len(raw) != expected:
            print(
                f"[WEBOTS CAMERA] Invalid frame size: "
                f"{len(raw)} != {expected}"
            )
            return False, None

        frame = np.frombuffer(
            raw,
            dtype=np.uint8
        ).reshape(
            (height, width, 4)
        )

        frame = frame[:, :, :3].copy()
        import cv2
        if self.last_frame_time == 0.0:
            cv2.imwrite("webots_debug_frame.png", frame)
            print("[WEBOTS CAMERA] First debug frame saved.")
        self.last_frame_time = time.time()

        return True, frame

    def is_stale(self):
        if self.stopped:
            return True

        if self.last_frame_time == 0.0:
            return True

        return (
            time.time() - self.last_frame_time
            > self.stale_timeout
        )

    def stop(self):
        if self.stopped:
            return

        self.stopped = True

        try:
            self.camera.disable()
        except Exception:
            pass

        print("[WEBOTS CAMERA] stopped")


class VirtualMQTTClient:
    def publish(self, topic, payload):
        print(
            f"[WEBOTS MQTT] {topic} -> {payload}"
        )


class VirtualMQTTController:
    def __init__(
        self,
        robot,
        command_secret="WEBOTS_SIM",
        max_speed_kmh=120.0,
        reverse_speed_kmh=3.6,
        max_steering_rad=0.45
    ):
        self.robot = robot
        self.command_secret = command_secret
        self.connected = True
        # Webots Driver API uses km/h for cruising speed.
        self.max_speed_kmh = float(max_speed_kmh)
        self.reverse_speed_kmh = float(reverse_speed_kmh)
        self.max_steering_rad = float(max_steering_rad)
        self.client = VirtualMQTTClient()

        print("[WEBOTS MQTT] initialized")

    def send_command(self, angle, engine_state):
        # -------------------------------------------------------------
        # STEERING VALIDATION
        # -------------------------------------------------------------

        try:
            angle = float(angle)
        except (TypeError, ValueError):
            print("[WEBOTS SAFETY] Invalid steering. STOP.")
            self._stop()
            return
        # -------------------------------------------------------------
        # ANGLE → RADIANS
        #
        # 90° = straight
        # <90° = left
        # >90° = right
        # -------------------------------------------------------------
        steering = (
            (angle - 90.0)
            * np.pi
            / 180.0
        )
        # -------------------------------------------------------------
        # STEERING SAFETY LIMIT
        # -------------------------------------------------------------
        steering = max(
            -self.max_steering_rad,
            min(self.max_steering_rad, steering)
        )

        state = str(engine_state).upper()
        # =============================================================
        # DRIVE
        # =============================================================
        if state == "DRIVE":
            self.robot.setSteeringAngle(steering)
            self.robot.setCruisingSpeed(self.max_speed_kmh) 

            actual_speed = self.robot.getCurrentSpeed()
            target_speed = self.robot.getTargetCruisingSpeed()
            print(
                f"[WEBOTS SPEED] "
                f"target={target_speed:.2f} km/h |"
                f"actual={actual_speed:.2f} km/h | "
                f"steering={steering:.3f} rad"
            )
        # =============================================================
        # REVERSE
        # =============================================================
        elif state == "REVERSE":
            self.robot.setSteeringAngle(steering)
            self.robot.setCruisingSpeed(-self.reverse_speed_kmh)

            print(
                f"[WEBOTS CMD] REVERSE "
                f"angle={angle:.1f}"
                f"steering={steering:.3f} rad "
                f"({reverse_speed_kmh:.2f} km/h)")
        # =============================================================
        # UNKNOWN STATE → FAIL SAFE STOP
        # =============================================================
        else:
            self._stop()

            print(
                f"[WEBOTS SAFETY] "
                f"Unknown engine state '{state}'. "
                f"STOP."
            )
            self._stop()

    def _stop(self):

        try:

            self.robot.setCruisingSpeed(
                0.0
            )

            self.robot.setSteeringAngle(
                0.0
            )

            print(
                "[WEBOTS SAFETY] STOP "
                "speed=0.0 km/h "
                "steering=0.0 rad"
            )

        except Exception as exc:

            print(
                f"[WEBOTS SAFETY] "
                f"STOP failed: {exc}"
            )

    def stop(self):

        self._stop()

        self.connected = False

        print(
            "[WEBOTS MQTT] stopped"
        )