# esp32/esp_orch.py
import machine               # type: ignore # Import low-level hardware module for system resets
from machine import WDT      # type: ignore # Import hardware Watchdog Timer module
import utime as time         # type: ignore # Import microsecond and millisecond timer utilities

DEBUG = False                 # Toggle flag for high-frequency debug serial logging

class RobotOrchestrator:
    def __init__(self, steering, motors, network, vision=None, ultrasonic=None, autonomous_mode=True, command_secret=None, manual_override_window_ms=1000, wdt_timeout_ms=8000, motion_armed=False):
        self.steering = steering                 # Store injected SteeringController instance
        self.motors = motors                     # Store injected MotorController instance
        self.network = network                   # Store injected NetworkController instance
        self.vision = vision                     # Store injected OnboardVisionController instance
        self.ultrasonic = ultrasonic             # Store injected UltrasonicSensor instance
        self.autonomous_mode = autonomous_mode   # Enable/disable autonomous vision control loop
        self.motion_armed = motion_armed         # Master safety arming flag for motor hardware activation
        self.command_secret = command_secret     # Secret token required to authenticate remote MQTT packets
        
        if not self.command_secret:              # Enforce configuration secret validation at startup
            raise RuntimeError("COMMAND_SECRET is missing or empty. The robot will not start without a valid secret.") # Halt boot if secret is unconfigured
            
        self.manual_override_window_ms = manual_override_window_ms # Manual command override duration limit in ms
        self.last_command_time = time.ticks_ms() # Timestamp tracking last general command received
        self.last_manual_command_time = 0        # Timestamp tracking last authenticated manual override command
        self._next_reconnect_attempt_ms = 0      # Deadline timestamp for non-blocking MQTT reconnect attempts
        self.emergency_stop = False              # Latch flag tracking manual emergency stop status

        try:
            self.wdt = WDT(timeout=wdt_timeout_ms) # Arm physical hardware watchdog timer
            print(f"Hardware watchdog armed ({wdt_timeout_ms}ms).") # Confirm watchdog arming in log
        except Exception as e:
            self.wdt = None                      # Nullify reference if board doesn't support WDT
            print(f"WARNING: Hardware watchdog unavailable ({e}). Running with software watchdog only.") # Output warning log

        print(f"RobotOrchestrator initialized. Autonomous mode: {self.autonomous_mode}.") # Log initialization summary

    def _feed_wdt(self):
        if self.wdt is not None:                 # Verify hardware watchdog existence
            self.wdt.feed()                      # Reset hardware watchdog timer countdown

    def _actuate(self, motor_cmd, angle=None):
        if not self.motion_armed:                # Gate physical movement based on safety arming flag
            print(f"MOTION LOCKED (MOTION_ARMED=False): would execute '{motor_cmd}'" + (f" at angle {angle}" if angle is not None else "") + ". Set MOTION_ARMED=True in config.py to enable.") # Log blocked movement
            return                               # Exit without triggering hardware pins

                # -------------------------------------------------------------
        # ULTRASONIC CLOSE-RANGE SAFETY VETO
        # -------------------------------------------------------------
        #
        # Ultrasonic does NOT steer.
        #
        # It only prevents DRIVE/REVERSE when something is
        # dangerously close.
        # -------------------------------------------------------------

        if (
            self.ultrasonic is not None
            and
            motor_cmd in (
                "DRIVE",
                "REVERSE"
            )
        ):

            distance_cm = (
                self.ultrasonic.read_cm()
            )

            if distance_cm is None:

                # Sensor failed / timed out.
                # Fail safe to STOP.

                self.motors.stop()

                print(
                    "ULTRASONIC INVALID -> "
                    "MOTORS STOPPED"
                )

                return

            if (
                distance_cm
                <=
                self.ultrasonic.STOP_DISTANCE_CM
            ):

                self.motors.stop()

                print(
                    "ULTRASONIC SAFETY STOP | "
                    f"distance={distance_cm:.1f} cm"
                )

                return

        if angle is not None:                    # Verify if a steering angle was supplied
            self.steering.set_angle(angle)       # Command physical servo position
        if motor_cmd == "DRIVE":
            self.motors.drive_forward()          # Actuate motors forward
        elif motor_cmd == "REVERSE":
            self.motors.drive_reverse()          # Actuate motors in reverse
        elif motor_cmd == "STOP":
            self.motors.stop()                   # De-energize motors immediately

    def process_message(self, topic, msg):
        try:
            payload = msg.decode("utf-8").strip() # Decode UTF-8 MQTT byte payload into string
            parts = payload.split(",")            # Parse CSV formatted string into list
            
            if len(parts) != 3:                   # Enforce strict payload structure: secret, angle, state
                raise ValueError("Invalid format") # Throw error on structural mismatch
            
            received_secret = parts[0]            # Extract secret token from payload
            if received_secret != self.command_secret: # Verify matching security token
                print("Rejected MQTT command: invalid shared secret.") # Log security rejection
                return                            # Reject unauthenticated command packet
            
            angle = int(parts[1])                 # Extract target steering angle integer
            state = parts[2].strip().upper()      # Extract target vehicle state command string
            
            self.last_command_time = time.ticks_ms() # Reset standard watchdog timestamp
            self.last_manual_command_time = time.ticks_ms() # Update manual override timestamp
            
            print(f"Executing (authenticated) -> Angle: {angle}, State: {state}") # Log valid command
            
            self.steering.set_angle(angle)        # Directly actuate steering servo
            print(f"Steering physically set to {angle} degrees.") # Log steering change
            
            if state == "DRIVE":
                if self.emergency_stop:           # Block DRIVE if emergency stop is currently latched
                    print("DRIVE command received but EMERGENCY STOP is latched. Send RESUME to re-enable motion.") # Log blocked command
                    return                        # Discard command
                self._actuate("DRIVE", angle)     # Execute DRIVE via safely gated actuate method
            elif state == "REVERSE":
                if self.emergency_stop:           # Block REVERSE if emergency stop is currently latched
                    print("REVERSE command received but EMERGENCY STOP is latched. Send RESUME to re-enable motion.") # Log blocked command
                    return                        # Discard command
                self._actuate("REVERSE", angle)   # Execute REVERSE via safely gated actuate method
            elif state == "STOP":
                self.emergency_stop = True        # Set latch for Emergency Stop condition
                self._actuate("STOP", angle)      # Force motor shutdown
                print("EMERGENCY STOP latched. Vehicle will not move until an authenticated RESUME command is received.") # Log active latch
            elif state == "RESUME":
                self.emergency_stop = False       # Clear Emergency Stop latch
                print("EMERGENCY STOP released by authenticated RESUME command. Autonomous driving will resume.") # Log latch release
            else:
                print("Unknown state received. Halting for safety.") # Log unexpected state
                self._actuate("STOP")             # Safely halt on unexpected state string
                
        except Exception as e:
            print("Parsing error:", e)            # Log parsing crash
            print("Malformed command ignored; current driving mode continues unaffected.") # Explain safe fallback

    def run(self):
        print("Starting Boot Sequence...")        # Log boot process start
        boot_attempts = 0                         # Initialize connection failure counter
        MAX_BOOT_ATTEMPTS_BEFORE_RESET = 10       # Maximum network boot retries in teleop mode
        MAX_BOOT_ATTEMPTS_AUTONOMOUS = 3          # Maximum network boot retries in autonomous mode
        network_online = False                    # Track network state during boot

        while True:                               # Boot network connection loop
            self._feed_wdt()                      # Keep hardware watchdog fed during startup
            try:
                print("Attempting to establish network connections...") # Log attempt
                self.network.connect_wifi(watchdog=self.wdt) # Connect to Wi-Fi access point
                self.network.connect_mqtt(self.process_message) # Connect to MQTT broker and bind callback
                print("SUCCESS: Robot Orchestrator is connected, online, and waiting for commands.") # Log success
                self.last_command_time = time.ticks_ms() # Initialize command timer
                network_online = True             # Mark network status active
                break                             # Exit boot connection loop
            except Exception as boot_err:
                boot_attempts += 1                 # Increment failure counter
                print(f"Boot connection failed (attempt {boot_attempts}): {boot_err}") # Log failure details
                if self.autonomous_mode and boot_attempts >= MAX_BOOT_ATTEMPTS_AUTONOMOUS: # Check threshold for autonomous mode
                    print("Network unavailable after a few tries. Proceeding WITHOUT network...") # Log offline state transition
                    network_online = False         # Record offline state
                    break                          # Exit boot connection loop to run offline
                if (not self.autonomous_mode) and boot_attempts >= MAX_BOOT_ATTEMPTS_BEFORE_RESET: # Check threshold for teleop mode
                    print("Too many failed boot attempts. Performing full hardware reset...") # Log reset action
                    self.motors.stop()            # Force stop motors prior to chip reset
                    time.sleep(1)                 # Pause briefly for serial buffer flush
                    machine.reset()               # Perform full hardware microcontroller reset
                print("Retrying boot sequence in 2 seconds...") # Log retry pause
                time.sleep(2)                     # Pause 2 seconds before retrying network connection
        
        print(f"Entering driving loop. Network online at boot: {network_online}. Motion armed: {self.motion_armed}. Autonomous: {self.autonomous_mode}.") # Log operational parameters
        try:
            while True:                           # Main operational control loop
                self._feed_wdt()                  # Feed hardware watchdog on every loop iteration
                now = time.ticks_ms()             # Grab millisecond tick counter for iteration timing

                if self.autonomous_mode:          # Execute autonomous execution branch
                    try:
                        self.network.check_for_commands() # Non-blocking check for pending MQTT commands
                    except Exception as e:
                        if time.ticks_diff(now, self._next_reconnect_attempt_ms) >= 0: # Check if reconnect cooldown has elapsed
                            print(f"Optional network link down ({e}); preparing to attempt background reconnect...") # Log reconnect trigger
                            self.motors.stop()    # Safely stop motors before executing blocking network reconnect
                            self.steering.set_angle(90) # Center steering for physical safety during reconnect
                            print("Motors and steering neutralised before reconnect attempt.") # Log safety action
                            try:
                                self.network.connect_wifi(timeout_s=5, watchdog=self.wdt) # Attempt quick 5-second Wi-Fi reconnect
                                self.network.connect_mqtt(self.process_message) # Re-establish MQTT session
                                print("Optional network link restored. Resuming autonomous driving.") # Log success
                            except Exception as reconn_err:
                                print(f"Background reconnect attempt failed: {reconn_err}. Driving on vision alone.") # Log failure
                            self._next_reconnect_attempt_ms = time.ticks_add(time.ticks_ms(), 5000) # Reset next attempt cooldown to +5s

                    manual_active = time.ticks_diff(now, self.last_manual_command_time) < self.manual_override_window_ms # Evaluate active manual override status
                    if self.emergency_stop:       # Evaluate active Emergency Stop latch
                        self._actuate("STOP")     # Maintain STOP state while latched
                        if DEBUG:
                            print("Autonomous loop: EMERGENCY STOP latched — all motion suppressed until RESUME.") # Output debug log
                    elif not manual_active:       # Run autonomous vision if no manual override is active
                        if self.vision is None:   # Safety check for missing vision controller instance
                            self._actuate("STOP") # Halt system on configuration error
                            print("CRITICAL CONFIG ERROR: autonomous_mode is True but no vision controller was provided. Halted.") # Log missing component
                        else:                     # Compute and execute onboard vision command
                            angle, state = self.vision.compute_command() # Compute steering angle and motion state from camera
                            if state == "DRIVE":
                                self._actuate("DRIVE", angle) # Actuate forward drive with calculated angle
                            elif state == "REVERSE":
                                self._actuate("REVERSE", angle) # Actuate reverse drive with calculated recovery angle
                            else:
                                self._actuate("STOP", angle) # Halt motors while keeping target steering angle

                else:                             # Execute legacy teleoperated execution branch
                    try:
                        self.network.check_for_commands() # Poll MQTT broker for teleop movement packets
                    except OSError as e:
                        print(f"Network dropped mid-drive: {e}") # Log unexpected drop
                        self.motors.stop()        # Immediately halt vehicle motion
                        print("Failsafe: Vehicle halted in place to prevent blind collision.") # Log failsafe action
                        
                        reconnect_attempts = 0    # Reset reconnect attempt counter
                        MAX_RECONNECT_ATTEMPTS_BEFORE_RESET = 15 # Set max reconnection retries limit
                        while True:               # Dedicated network recovery loop
                            self._feed_wdt()      # Feed hardware watchdog during recovery
                            try:
                                print("Attempting to reconnect to network...") # Log retry
                                self.network.connect_wifi(watchdog=self.wdt) # Reconnect Wi-Fi
                                self.network.connect_mqtt(self.process_message) # Reconnect MQTT
                                print("Reconnection successful! Resuming operations.") # Log recovery
                                self.last_command_time = time.ticks_ms() # Reset command timer
                                break             # Resume main loop execution
                            except Exception as reconn_err:
                                reconnect_attempts += 1 # Increment retry count
                                print(f"Connection failed, please connect. Error: {reconn_err} (attempt {reconnect_attempts})") # Log error
                                if reconnect_attempts >= MAX_RECONNECT_ATTEMPTS_BEFORE_RESET: # Evaluate reset condition
                                    print("Too many failed reconnection attempts. Performing full hardware reset...") # Log reset
                                    time.sleep(1) # Pause briefly
                                    machine.reset() # Perform chip reset
                                time.sleep(2)     # Pause 2s between reconnection attempts
                                
                    if time.ticks_diff(time.ticks_ms(), self.last_command_time) > 500: # Check for network command timeout (>500ms)
                        self.motors.stop()        # Halt vehicle motion due to stale command/link drop
                        print("Watchdog triggered: PC lagged or disconnected. Motors halted.") # Log watchdog trigger
                    
                time.sleep(0.01)                  # Sleep 10ms to constrain main loop execution to ~100Hz and prevent CPU thermal throttling
                
        except KeyboardInterrupt:                 # Intercept Ctrl+C terminal signal cleanly
            print("Robot program stopped manually by user via terminal.") # Log user exit
        finally:
            print("Initiating final hardware shutdown sequence...") # Log start of shutdown sequence
            self.motors.stop()                    # Cut power to motor drivers
            print("Hardware safely secured and completely stopped.") # Log clean shutdown completion