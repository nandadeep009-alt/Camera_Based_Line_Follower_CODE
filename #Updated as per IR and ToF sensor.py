..#Updated as per IR and ToF sensor integration. Added failsafe logic to stop motors if either sensor detects an obstacle or off-path condition.
# main.py (Runs on ESP32-S3)
# ROLE: Tactical Brain: Sensor Fusion, Real-time Overrides, and PID Execution

import network
import machine
from machine import Pin, PWM, WDT, I2C, SPI
from umqtt.simple import MQTTClient
import utime as time
import _thread # For runningx / camera on second core
import json

import camera       # OV2640 CAMERA DRIVER
import mpu6050      # IMU DRIVER (Needs custom lib)
import vl53l0x      # ToF SENSOR DRIVER (Needs custom lib)
import mfrc522      # RFID DRIVER (Needs custom lib)
import tcs34725     # COLOR DRIVER (Needs custom lib)

# ---------------------------------------------------------
# PIN ASSIGNMENTS & HARDWARE CONFIG
# ---------------------------------------------------------
# GPIOs 34-39 are used exclusively by the Camera interface

# PWM Actuators
SERVO_PIN   = 16 # Steering Servo (50Hz PWM)
MOTOR_EN_A  = 4  # Rear Motor Enable/Speed (PWM)
MOTOR_IN_1  = 5  # Rear Motor Direction IN1
MOTOR_IN_2  = 6  # Rear Motor Direction IN2

# Shared I2C Bus (ToF, IMU, Color)
I2C_SDA_PIN = 14
I2C_SCL_PIN = 13

# SPI Bus (RFID Reader)
SPI_SCK_PIN = 12
SPI_MOSI_PIN= 11
SPI_MISO_PIN= 10
RFID_RST_PIN= 9
RFID_CS_PIN = 8

# Digital GPIO Inputs (IR, Encoder)
IR_SENSOR_PIN = 7 # High = Path Clear, Low = Reflection Detected
ENCODER_PIN   = 3 # Encoder Pulse Pin (High interrupt rate)

# --- DEBUG FLAG ---
# Set to False to disable prints inside the high-frequency control loop
DEBUG_LOGGING = False

# ---------------------------------------------------------
# 1. HARDWARE DRIVER CLASSES
# ---------------------------------------------------------
class SteeringController:
    """Manages front servo geometry within mechanical constraints."""
    def __init__(self, pin_num):
        self.servo = PWM(Pin(pin_num))
        self.servo.freq(50) # Standard hobby servo frequency
        self.angle_constraint = (45, 135) # Mechanical limits

    def set_angle(self, angle):
        # Apply mechanical clamping
        clamped_angle = max(self.angle_constraint[0], min(self.angle_constraint[1], angle))
        # Linear map: angle -> PWM duty cycle (linear approximation: 40-115)
        duty = int(((clamped_angle / 180.0) * (115 - 40)) + 40)
        self.servo.duty(duty)
        return clamped_angle

class MotorController:
    """Manages L293D driver logic and open-loop speed."""
    def __init__(self, en, in1, in2):
        self.en = PWM(Pin(en))
        self.en.freq(1000) # PWM freq for smoother acceleration
        self.in1 = Pin(in1, Pin.OUT)
        self.in2 = Pin(in2, Pin.OUT)
        self.stop() # Ensure motors are secured on boot

    def drive(self, speed):
        """0-100 speed, positive=forward, negative=reverse."""
        clamped_speed = max(-100, min(100, speed))
        duty = int(abs(clamped_speed) * 1023 / 100) # Map 0-100 speed to 10-bit duty cycle
        self.en.duty(duty)
        if clamped_speed > 0: self._forward()
        elif clamped_speed < 0: self._reverse()
        else: self.stop()

    def _forward():
        self.in1.value(1); self.in2.value(0)
    def _reverse():
        self.in1.value(0); self.in2.value(1)
    def stop():
        self.en.duty(0); self.in1.value(0); self.in2.value(0)

# ---------------------------------------------------------
# 2. ESP32 "TACTICAL BRAIN" - THE ORCHESTRATOR
# ---------------------------------------------------------
class RobotOrchestrator:
    """Manages all local sensors, path validation, and failsafes."""
    def __init__(self):
        # SECURE BOOT: Guarantee Safe State FIRST
        self.steering = SteeringController(SERVO_PIN)
        self.motors = MotorController(MOTOR_EN_A, MOTOR_IN_1, MOTOR_IN_2)
        print("Hardware secured (Motors STOP, Servo CENTER).")

        # Configuration & Network details
        try:
            import config as _cfg
            self.ssid = _cfg.WIFI_SSID
            self.password = _cfg.WIFI_PASS
            self.broker = _cfg.MQTT_BROKER
            self.topic = _cfg.COMMAND_TOPIC
            self.secret = _cfg.COMMAND_SECRET # Authentication token
        except ImportError:
            raise Exception("FATAL: config.py missing on board.")

        # --- Local Watchdogs & Flags ---
        # 1. Hardware Watchdog: Chip will forcibly reset if this loop stalls
        self.wdt = WDT(timeout=8000) # generous 8s timeout for wifi retries
        self.last_command_time = time.ticks_ms() # for software watchdog
        self.autonomous_active = True # False means waiting for PC
        self.motion_armed = False     # Master motion lock

        # --- Sub-System Initialization ---
        # Initialize communication interfaces
        self.i2c = I2C(0, sda=Pin(I2C_SDA_PIN), scl=Pin(I2C_SCL_PIN))
        self.spi = SPI(1, baudrate=2500000, polarity=0, phase=0, 
                       sck=Pin(SPI_SCK_PIN), mosi=Pin(SPI_MOSI_PIN), miso=Pin(SPI_MISO_PIN))

        # --- LOCAL SENSOR FUSION BUS ---
        # We wrap sensor initialization in Try/Except blocks so a loose wire doesn't crash the boot
        
        # Priority 1: Obstacle Avoidance (ToF)
        try: self.tof = vl53l0x.VL53L0X(self.i2c); print("ToF sensor: OK")
        except: self.tof = None; print("ToF sensor: ERROR/Missing")

        # Priority 2: Stability (IMU)
        try: self.imu = mpu6050.mpu6050(self.i2c); print("IMU sensor: OK")
        except: self.imu = None; print("IMU sensor: ERROR/Missing")

        # Priority 3: Path Validation (IR)
        self.ir = Pin(IR_SENSOR_PIN, Pin.IN); print("IR Sensor: OK")
        
        # Priority 4: Task Validation (Color, RFID)
        try: self.color = tcs34725.TCS34725(self.i2c); print("Color sensor: OK")
        except: self.color = None; print("Color sensor: ERROR/Missing")
        try: self.rfid = mfrc522.MFRC522(self.spi, Pin(RFID_CS_PIN), Pin(RFID_RST_PIN)); print("RFID Reader: OK")
        except: self.rfid = None; print("RFID Reader: ERROR/Missing")

        # Priority 5: Precision/Closed-loop control (Encoder)
        # Placeholder: We use hardware interrupts on Pin 3 to count encoder pulses.
        # Requires advanced MicroPython C-module for hardware PCNT if high RPM.
        self.encoder_pulses = 0
        Pin(ENCODER_PIN, Pin.IN).irq(trigger=Pin.IRQ_RISING, handler=self._encoder_isr)

    def _encoder_isr(self, pin):
        # Extremely fast interrupt service routine to count pulses
        self.encoder_pulses += 1

    def _connect_wifi(self):
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        if not wlan.isconnected():
            print(f"Connecting to Wi-Fi SSID: {self.ssid}...")
            wlan.connect(self.ssid, self.password)
            while not wlan.isconnected():
                self.wdt.feed() # Must feed WDT while waiting for network
                time.sleep_ms(100)
        print("Wi-Fi Connected. IP:", wlan.ifconfig()[0])

    def connect_mqtt(self):
        self._connect_wifi()
        # Derive unique client ID from MAC address
        client_id = f"esp32_robot_{' '.join(['%02x' % b for b in network.WLAN(network.STA_IF).config('mac')])}"
        self.client = MQTTClient(client_id, self.broker)
        # Umqtt automatically calls process_message when a packet arrives
        self.client.set_callback(self.process_message)
        print("Connecting to MQTT Broker...")
        self.client.connect()
        # The .decode() is necessary as topics are bytes literals in MicroPython umqtt
        self.client.subscribe(self.topic)
        print(f"Subscribed to topic: {self.topic.decode()}")

    def process_message(self, topic, msg):
        """Umqtt Callback: Fires automatically when packet arrives."""
        try:
            # UMqtt provides the message as raw bytes. Decode to string.
            payload = msg.decode("utf-8").strip()
            # Split the comma-separated string: "SECRET,angle,state"
            parts = payload.split(",")
            
            if len(parts) != 3: return # Failsafe: Ignore bad format

            received_secret = parts[Part 1]
            angle_str       = parts[Part 2]
            state_str       = parts[Part 3]

            # --- AUTHENTICATION CHECK ---
            # If the secret does not match, ignore the entire packet.
            if self.command_secret and received_secret != self.command_secret:
                if DEBUG_LOGGING: print("Rejected unauthenticated command.")
                return

            # --- Data Parsing ---
            target_angle = int(angle_str)
            target_speed = int(state_str) # PC Brain now sends an 'optimal speed'
            
            self.last_command_time = time.ticks_ms() # Reset PC Brain watchdog
            self.motion_armed = True                 # Master motion unlock

            # We DO NOT actuate motors here. We just save the path command.
            # The next iteration of the main loop will validate it.
            self.requested_angle = target_angle
            self.requested_speed = target_speed

            if DEBUG_LOGGING: print(f"Valid path received: Angle={target_angle}, Speed={target_speed}")
                
        except Exception as e:
            # Failsafe: Any parsing error -> Ignore
            if DEBUG_LOGGING: print("Command parsing error:", e)

    def _high_frequency_safety_sweep(self):
        """Polls local sensors, returns (safe_to_move, validation_score)."""
        all_clear = True
        failsafe_stop = False

        # --- Priority 1: Deterministic Failsafe Overrides ---
        
        # 1. ToF Laser Distance Check
        if self.tof:
            try:
                # Polling takes ~20ms, ideal for failsafes
                distance_mm = self.tof.read()
                if DEBUG_LOGGING: print(f"ToF dist: {distance_mm}mm")
                # Immediate Halt if object < 15cm
                if distance_mm < 150: failsafe_stop = True; print("SAFETY FAILSAFE: Obstacle Detected (ToF)")
            except: pass

        # 2. IR Path validation (Simple binary check)
        # Assumes IR output is LOW ifreflection detected (track clear)
        if self.ir.value() == 1: 
            failsafe_stop = True; print("SAFETY FAILSAFE: IR sensor indicates Edge/Off-path")

        # 3. IMU Stability Check (Detect if car flipped/impacted)
        if self.imu and DEBUG_LOGGING:
            accel = self.imu.get_accel_data()
            if abs(accel['x']) > 15 or abs(accel['y']) > 15: # >1.5G lateral acceleration
                 failsafe_stop = True; print("SAFETY FAILSAFE: IMU Tilt/Impact")

        if failsafe_stop: return False # local reflex triggers immediate stop

        # --- Priority 2: PC Strategic Brain Validation ---
        
        # Software Watchdog: Check if robot.py is lagging
        # umqtt umqtt provides its own non-blocking check, but we add an absolute timer
        if time.ticks_diff(time.ticks_ms(), self.last_command_time) > 500: # 500ms latency cutoff
            all_clear = False
            if DEBUG_LOGGING: print("PC Brain Lagging > 500ms. Defaulting to Halt.")
        
        if not self.motion_armed: all_clear = False # Wait for first auth command

        # --- Priority 3: Non-Blocking Task Validation (Color, RFID) ---
        # Task validation does not trigger a failsafe stop, but provides scoring feedback to robot.py
        validation_status = {}
        if self.rfid:
            (status, tag_type) = self.rfid.request(self.rfid.REQIDL)
            if status == self.rfid.OK:
                (status, uid) = self.rfid.anticoll()
                if status == self.rfid.OK:
                    uid_str = "".join([hex(b)[2:] for b in uid])
                    validation_status['rfid_uid'] = uid_str
                    if DEBUG_LOGGING: print(f"RFID Tag found: {uid_str}")

        if self.color:
            try:
                rgb = self.color.read_rgb()
                validation_status['floor_rgb'] = rgb
                if DEBUG_LOGGING: print(f"Floor RGB: {rgb}")
            except: pass
        
        # Velocity Feedback (Encoder ticks since last loop)
        current_ticks = self.encoder_pulses
        self.encoder_pulses = 0 # Reset pulse counter
        # validation_status['velocity_ticks'] = current_ticks # Send back to PC

        return all_clear, validation_status

    def start_camera_on_core1(self):
        # Launch camera server on Core 1 so the Core 0 control loop is never blocked
        try:
            camera.init(0, format=camera.JPEG, framesize=camera.FRAME_QVGA)
            print("Camera hardware initialized.")
            _thread.start_new_thread(self._serve_video_loop, ())
        except Exception as e:
            print(f"CRITICAL: Camera hardware failed: {e}")

    def _serve_video_loop(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('', 80))
        s.listen(1)
        while True:
            try:
                conn, addr = s.accept()
                conn.send(b'HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n')
                while True:
                    buf = camera.capture()
                    conn.send(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf + b'\r\n')
            except Exception:
                if conn: conn.close()

    def run_control_loop(self):
        self.connect_mqtt()
        print("Robot Orchestrator Tactical loop is online. Master motion lock: ARMED")
        
        self.requested_angle = 90
        self.requested_speed = 0

        # Deterministic Time-based Loop (100 Hz tactical loop)
        target_loop_ms = 10 
        
        try:
            while True:
                loop_start = time.ticks_ms()
                self.wdt.feed() # Feed HW Watchdog Timer every iteration

                # 1. Non-Blocking Network Check (Check if any MQTT message is waiting)
                # umqtt fires process_message synchronously inside this call if message waiting
                self.client.check_msg() 

                # 2. Tactical Sensor Fusion & Local Failsafe Sweep (Copper-wire intelligence)
                is_safe, task_telem = self._high_frequency_safety_sweep()

                # --- Decision Matrix & Actuation ---
                if not is_safe:
                    # LOCAL SENSORS TRIGGER FAILSAFE
                    self.motors.stop()
                    self.steering.set_angle(90) # center steering during halt
                
                else:
                    # PC STRATEGIC PATH IS VALIDATED BY LOCAL REALITY
                    # Execute Path Command
                    actual_angle = self.steering.set_angle(self.requested_angle)
                    
                    # --- Closed-Loop Velocity PID (Simplified) ---
                    # The MotorController currently just takes requested_speed (0-100)
                    # For production, we must implement a local PID controller that
                    # adjusts EN duty cycle to match requested_speed using Encoder ticks.
                    
                    # Current Implementation: Open-loop drive
                    self.motors.drive(self.requested_speed)

                # 3. Publish Local Sensor Telemetry back to robot.py for monitoring/learning
                # Rate limited to 10Hz to prevent saturating MQTT bandwidth
                if task_telem and time.ticks_diff(time.ticks_ms(), self.last_command_time) % 100 < 10:
                     # Add current actuation state to telemetry JSON
                     task_telem['actuation'] = {"angle": actual_angle, "speed": self.requested_speed}
                     try: self.client.publish("primerail/robot/telemetry", json.dumps(task_telem))
                     except: pass

                # 4. Deterministic Loop Timing
                time.sleep_ms(max(0, target_loop_ms - time.ticks_diff(time.ticks_ms(), loop_start)))

        except KeyboardInterrupt:
            print("Control loop stopped manually by user via terminal.")
        except OSError as network_err:
            print(f"Network failure mid-drive. Initiating Hardware Halt. Error: {network_err}")
            time.sleep(1) # give print time
            machine.reset() # Full chip reset is the only safe recovery path
        finally:
            # SAFETY SHUTDOWN: If the loop breaks for ANY reason, kill the wheels
            self.motors.stop()
            print("Hardware successfully secured.")

# ---------------------------------------------------------
# 5.MAIN EXECUTION (Object Instantiation and Assembly)
# ---------------------------------------------------------
if __name__ == "__main__":
    # FAILSAFE HARDWARE BRING-UP: wrap construction in try/except so crash on boot
    # triggers a full chip reset rather than leaving board in unknown state.
    try:
        app = RobotOrchestrator()
        
        # Start camera server on the second core
        app.start_camera_on_core1()
        time.sleep(1.0) # give MJPEG stream time to fill buffer before starting control loop
        
        # Start the main high-frequency tactical loop on core 0
        app.run_control_loop()
    except Exception as boot_err:
        print(f"CRITICAL: System failed during boot assembly: {boot_err}")
        print("Resetting in 3 seconds to attempt recovery...")
        time.sleep(3)
        machine.reset() # Full hardware reset