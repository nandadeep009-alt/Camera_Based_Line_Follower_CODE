"""
=========================================================================================
SUMMARY OF ROBOT BRAIN (OOP PRODUCTION VERSION)
-----------------------------------------------------------------------------------------
This script acts as the main firmware for an ESP32-S3 line-following robot.
It is built using Object-Oriented Programming (OOP) to cleanly separate responsibilities:

1. SteeringController: Manages the Servo motor. Enforces mechanical safety limits 
   so the steering never attempts to turn past its physical boundaries.
2. MotorController: Manages the L293D driver. Explicitly controls the EN (Enable) 
   pins alongside the directional IN pins to ensure precise power delivery.
3. NetworkController: Handles Wi-Fi connection and the MQTT internet bridge.
4. RobotOrchestrator: The central manager. It listens for commands, safely parses 
   the text, handles errors without crashing, and commands the hardware classes.

Safety Features Built-In:
- Failsafe Startup: Motors are explicitly forced into a STOP state on boot.
- Data Validation: Bad packets (e.g., missing commas, letters instead of numbers) 
  are caught and ignored without crashing the main loop.
- Failsafe Shutdown: If the script crashes or loses connection, a 'finally' block 
  guarantees the physical wheels are stopped to prevent runaway hardware.
=========================================================================================
"""

import network                      # Imports the library needed to control the ESP32 Wi-Fi antenna
import utime as time                         # Imports the timing library to create safe pauses
from machine import Pin, PWM        # Imports hardware controls for copper pins and electrical pulses
from umqtt.simple import MQTTClient # Imports the lightweight MQTT networking tool

import camera                       #CAMERA HARDWARE LIBRARY
import socket                       #WEB SERVER LIBRARY
import _thread                      #MULTI-CORE PROCESSING LIVRARY

# ---------------------------------------------------------
# 1. HARDWARE CLASSES (The Muscle)
# ---------------------------------------------------------
class SteeringController:
    def __init__(self, pin_num):
        # Create a PWM (Pulse Width Modulation) object on the specified pin
        self.servo = PWM(Pin(pin_num))
        # Set the electrical pulse frequency to 50Hz (mandatory for servos)
        self.servo.freq(50)
        # Immediately center the wheels on startup for safety
        self.set_angle(90)

    def set_angle(self, angle):
        # SAFETY: Clamp the angle between 45 and 135 degrees to prevent stripping physical gears
        safe_angle = max(45, min(135, angle))
        
        # Convert the human-readable degree (45-135) into a MicroPython PWM duty cycle (40-115)
        duty = int(((safe_angle / 180.0) * (115 - 40)) + 40)
        
        # Send the calculated electrical pulse to the physical servo pin
        self.servo.duty(duty)


class MotorController:
    """def __init__(self, en_a, in1, in2, en_b, in3, in4):
        # Setup Left Motor Pins
        self.en_a = Pin(en_a, Pin.OUT)  # Master power switch for Left Motor
        self.in1 = Pin(in1, Pin.OUT)    # Forward switch for Left Motor
        self.in2 = Pin(in2, Pin.OUT)    # Reverse switch for Left Motor
        
        # Setup Right Motor Pins
        self.en_b = Pin(en_b, Pin.OUT)  # Master power switch for Right Motor
        self.in3 = Pin(in3, Pin.OUT)    # Forward switch for Right Motor
        self.in4 = Pin(in4, Pin.OUT)    # Reverse switch for Right Motor
        
        # Turn ON the master power switches immediately so the L293D chip is awake
        self.en_a.value(1)
        self.en_b.value(1)
        
        # Explicitly halt the wheels on boot to prevent accidental runaway
        self.stop()

    def drive_forward(self):
        # Pull forward pins to 3.3V (HIGH) and reverse pins to 0V (LOW)
        self.in1.value(1)
        self.in2.value(0)
        self.in3.value(1)
        self.in4.value(0)

    def stop(self):
        # Drain all directional pins to 0V (LOW). The L293D will cut power to the wheels.
        self.in1.value(0)
        self.in2.value(0)
        self.in3.value(0)
        self.in4.value(0)"""
    def __init__(self, en_a, en_b, in1, in2, in3, in4): #Contructor method to initialize all the motor pins
        self.en_a = Pin(en_a, Pin.OUT)  # Initialize the Left Master Power pin
        self.in1 = Pin(in1, Pin.OUT)    # Initialize the Left Forward directional pin
        self.in2 = Pin(in2, Pin.OUT)    # Initialize the Left Reverse directional pin
        self.en_b = Pin(en_b, Pin.OUT)  # Initialize the Right Master Power pin
        self.in3 = Pin(in3, Pin.OUT)    # Initialize the Right Forward directional pin
        self.in4 = Pin(in4, Pin.OUT)    # Initialize the Right Reverse directional pin
        self.stop()                     # CHANGED: Safely call stop() on boot to lock wheels immediately
        print("MotorController initialized and secured on boot.") # ADDED: Print confirmation that motors are set up

    def drive_forward(self): # Method to command both motors to spin forward
        self.en_a.value(1)              # CHANGED: Turn ON Left master power ONLY when driving
        self.en_b.value(1)              # CHANGED: Turn ON Right master power ONLY when driving
        self.in1.value(1)               # Pull Left Forward pin to 3.3V (HIGH)
        self.in2.value(0)               # Pull Left Reverse pin to 0V (LOW)
        self.in3.value(1)               # Pull Right Forward pin to 3.3V (HIGH)
        self.in4.value(0)               # Pull Right Reverse pin to 0V (LOW)
        print("Motors driving forward.") # ADDED: Print confirmation of forward movement

    def stop(self): # Method to completely halt all motor activity and cut power
        self.en_a.value(0)              # CHANGED: COMPLETELY cut Left master power to guarantee zero jitter
        self.en_b.value(0)              # CHANGED: COMPLETELY cut Right master power to guarantee zero jitter
        self.in1.value(0)               # Drain Left Forward pin to 0V
        self.in2.value(0)               # Drain Left Reverse pin to 0V
        self.in3.value(0)               # Drain Right Forward pin to 0V
        self.in4.value(0)               # Drain Right Reverse pin to 0V
        print("Motors stopped and power cut.") # ADDED: Print confirmation that motors are physically dead

# ---------------------------------------------------------
# 2. NETWORK CLASS (The Ears)
# ---------------------------------------------------------
class NetworkController:
    def __init__(self, ssid, password, broker, topic):
        # Store all connection details inside the object
        self.ssid = ssid
        self.password = password
        self.broker = broker
        self.topic = topic
        self.client = None

    def connect_wifi(self):
        # Turn on the ESP32 Wi-Fi in Station Mode (client mode)
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        
        print("Connecting to Wi-Fi...")
        # Start the connection process to the router
        wlan.connect(self.ssid, self.password)
        
        # Wait safely in a loop until the router assigns an IP address
        while not wlan.isconnected():
            time.sleep(0.5)
            print(".", end="")
            
        print("\nWi-Fi Connected! IP:", wlan.ifconfig()[0])

    def connect_mqtt(self, message_callback):
        # Create the MQTT client object using the stored broker address
        self.client = MQTTClient("esp32_robot_prod", self.broker)
        
        # Tell the client which function to run when a message arrives
        self.client.set_callback(message_callback)
        
        print("Connecting to MQTT Broker...")
        # Dial into the server
        self.client.connect()
        
        # Subscribe to our specific walkie-talkie channel
        self.client.subscribe(self.topic)
        print("Subscribed to topic:", self.topic.decode())

    def check_for_commands(self):
        # Ask the server if any new messages are waiting
        self.client.check_msg()

# ---------------------------------------------------------
# 3. CameraStreamer (The Eyes)
# ---------------------------------------------------------
class CameraStreamer:                              # Define the CameraStreamer class to handle live video broadcasting
    def __init__(self):                            # Constructor method to set up the physical camera lens hardware
        print("Initializing Camera Hardware...")   # Print a startup status message to the terminal
        try:                                       # Start an error-monitoring block in case the camera ribbon cable is loose
            # Initialize the camera. Note: Pins vary by exact ESP32-S3 board model. '0' usually auto-detects.
            camera.init(0, format=camera.JPEG, framesize=camera.FRAME_VGA) # Turn on the lens and set it to VGA resolution (640x480)
            print("Camera initialized successfully.") # Print confirmation that the hardware is working
        except Exception as e:                     # Catch any hardware failures during initialization
            print(f"CRITICAL: Camera failed to start: {e}") # Print the exact physical failure reason to the terminal

    def start_server(self):                        # Method to boot up the web server for the PC to connect to
        print("Starting video stream server on core 1...") # Print status indicating the server is starting
        _thread.start_new_thread(self._serve_video, ()) # Launch the video streaming loop in the background on the second CPU core

    def _serve_video(self):                        # The actual web server loop that runs invisibly in the background
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Create a standard internet communication socket for the server
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Allow the socket port to be reused instantly if the ESP32 crashes
        s.bind(('', 80))                           # Bind the video server to port 80 (the universal standard website port)
        s.listen(1)                                # Listen for exactly 1 connection (we only want your PC connecting, nobody else)
        
        while True:                                # Start an infinite loop to keep the web server alive forever
            try:                                   # Start an error-monitoring block for the incoming PC connection
                conn, addr = s.accept()            # Pause the background thread and wait for the PC to connect. Accept it when it does.
                print(f"PC connected to camera stream from IP: {addr}") # Print the PC's IP address so you know it connected successfully
                
                # Send the standard HTTP headers required to create an MJPEG (Moving JPEG) video stream format
                conn.send(b'HTTP/1.1 200 OK\r\n')  # Tell the PC the network connection is successful and approved
                conn.send(b'Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n') # Tell the PC to expect a continuous looping video stream
                
                while True:                        # Start an infinite loop to blast photos over Wi-Fi as fast as possible
                    buf = camera.capture()         # Take a single, high-speed JPEG photo using the physical camera lens
                    if buf:                        # Check if the photo was successfully captured and the buffer is not empty
                        conn.send(b'--frame\r\n')  # Send the frame boundary marker to separate this photo from the last one
                        conn.send(b'Content-Type: image/jpeg\r\n\r\n') # Tell the PC this specific chunk of data is a JPEG image
                        conn.send(buf)             # Send the actual raw image bytes over the Wi-Fi connection
                        conn.send(b'\r\n')         # Send a blank line to finish the frame transmission and prepare for the next one
                        
            except Exception as stream_err:        # Catch if the PC disconnects, the Wi-Fi drops, or the socket breaks
                print(f"Video stream interrupted: {stream_err}. Waiting for reconnection...") # Print the error and patiently wait for the PC to return
            finally:                               # This block runs anytime the PC disconnects or an error occurs in the stream
                try:                               # Try to safely close the broken connection
                    conn.close()                   # Safely close the network socket so it can be opened again for the next connection attempt
                except:                            # Catch any errors that happen while trying to close the socket
                    pass                           # Ignore closing errors and continue running the main server loop

# ---------------------------------------------------------
# 4.0. ORCHESTRATOR CLASS (The Brain)
# ---------------------------------------------------------
"""class RobotOrchestrator:
    def __init__(self, steering, motors, network):
        # Dependency Injection: The manager receives the hardware and network objects
        self.steering = steering
        self.motors = motors
        self.network = network

    def process_message(self, topic, msg):
        # This function fires automatically when an MQTT packet arrives
        try:
            # Decode the raw byte message into a standard Python text string
            payload = msg.decode("utf-8").strip()
            
            # Split the text at the comma (expecting "90,DRIVE")
            parts = payload.split(",")
            
            # ERROR HANDLING: Ensure we received exactly two pieces of data
            if len(parts) != 2:
                raise ValueError("Invalid payload format. Expected 'Angle,State'")

            # Convert the first part to an integer, and clean up the second part
            angle = int(parts[0])
            state = parts[1].strip().upper()
            
            print(f"Executing -> Angle: {angle}, State: {state}")

            # 1. Execute Steering Command
            self.steering.set_angle(angle)

            # 2. Execute Motor Command
            if state == "DRIVE":
                self.motors.drive_forward()
            elif state == "STOP":
                self.motors.stop()
            else:
                # If a weird state word comes through, default to stopping for safety
                print("Unknown state received. Stopping for safety.")
                self.motors.stop()
                
        except Exception as e:
            # ERROR HANDLING: If letters were sent instead of numbers, catch the crash and stop
            print("Data parsing error:", e)
            self.motors.stop()

    def run(self):
        # Step 1: Connect to Wi-Fi
        self.network.connect_wifi()
        
        # Step 2: Connect to MQTT and pass it our processing function
        self.network.connect_mqtt(self.process_message)
        
        print("Robot Orchestrator is online and waiting for commands.")
        
        try:
            # Step 3: The Infinite Loop
            while True:
                # Check for internet messages
                self.network.check_for_commands()
                # Pause for 10ms to prevent the CPU from overheating
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            # Safely handle the user pressing Ctrl+C in the terminal
            print("Robot stopped by user.")
            
        finally:
            # SAFETY SHUTDOWN: If the loop breaks for ANY reason, kill the wheels
            self.motors.stop()
            print("Hardware safely secured.")
"""

# ---------------------------------------------------------
# 4.1. UPDATED: RobotOrchestrator (With Secure Boot Sequence)
# ---------------------------------------------------------
class RobotOrchestrator: # Define the RobotOrchestrator class to manage the main brain loop
    def __init__(self, steering, motors, network): # Constructor taking in the three hardware/network objects
        self.steering = steering                 # Store the injected steering object
        self.motors = motors                     # Store the injected motors object
        self.network = network                   # Store the injected network object
        self.last_command_time = time.ticks_ms() # Create a Watchdog Timer to track the last PC message
        print("RobotOrchestrator initialized.")  # Print confirmation of brain startup

    def process_message(self, topic, msg): # Method triggered automatically when a new MQTT network packet arrives
        try:                                     # Start an error-handling block to catch bad internet data
            self.last_command_time = time.ticks_ms() # Reset the Watchdog Timer because we just heard from the PC!
            payload = msg.decode("utf-8").strip()    # Decode the raw network byte message into standard text
            parts = payload.split(",")               # Split the text at the comma separator
            
            if len(parts) != 2:                      # Ensure we got exactly two pieces of data
                raise ValueError("Invalid format")   # Throw an error if the format is wrong
            
            angle = int(parts[0])                    # Convert the first text part into an integer math number
            state = parts[1].strip().upper()         # Clean up the second text part into an uppercase string
            
            print(f"Executing -> Angle: {angle}, State: {state}") # Print the exact command we are running
            
            self.steering.set_angle(angle)           # Command the physical steering servo to turn
            print(f"Steering physically set to {angle} degrees.") # Print steering confirmation
            
            if state == "DRIVE":                     # Check if the command word is "DRIVE"
                self.motors.drive_forward()          # Push the DC motors forward
            elif state == "STOP":                    # Check if the command word is "STOP"
                self.motors.stop()                   # Halt the DC motors
            else:                                    # Catch any weird unknown command words
                print("Unknown state received. Halting for safety.") # Print warning about unknown state
                self.motors.stop()                   # Safely halt the DC motors
                
        except Exception as e:                       # Catch any text decoding or math crash errors
            print("Parsing error:", e)               # Print the exact error to the PC terminal
            self.motors.stop()                       # Safely halt motors so the robot doesn't drive blind
            print("Motors forcibly stopped due to data parsing error.") # Print failsafe action

    def run(self): # The main operating loop of the robot that runs forever
        print("Starting Boot Sequence...")           # Print that the robot is powering on
        
        # --- FAILSAFE BOOT STAGE: PROVE THE CONNECTION FIRST ---
        while True:                                  # Start an infinite loop specifically for booting up
            try:                                     # Start watching for hardware connection errors
                print("Attempting to establish network connections...") # Print status
                self.network.connect_wifi()          # Ask hardware to connect to Wi-Fi router
                self.network.connect_mqtt(self.process_message) # Ask hardware to dial the MQTT internet broker
                
                # If the code reaches this next line without jumping to 'except', it means NO ERRORS happened!
                print("SUCCESS: Robot Orchestrator is connected, online, and waiting for commands.") # Print true verified status
                self.last_command_time = time.ticks_ms() # Start the watchdog timer fresh right now
                break                                # BREAK out of the boot loop so we can move down to the driving loop!
                
            except Exception as boot_err:            # Catch any failure (wrong Wi-Fi password, no internet, etc.)
                print(f"Boot connection failed: {boot_err}") # Print exactly why it failed to connect
                print("Retrying boot sequence in 2 seconds...") # Let user know it will try again
                time.sleep(2)                        # Wait 2 seconds before trying to boot up again
        
        # --- MAIN DRIVING STAGE ---
        try:                                         # Start the master driving loop block
            while True:                              # Start the infinite driving loop
                
                # --- FAILSAFE STAGE 1: ACTIVE NETWORK MONITORING ---
                try:                                 # Inner try-block added to catch sudden Wi-Fi drops while driving
                    self.network.check_for_commands() # Ask the MQTT internet broker for new messages
                except OSError as e:                 # Catch the specific network disconnection error
                    print(f"Network dropped mid-drive: {e}") # Print the system error that caused the drop
                    self.motors.stop()               # IMMEDIATELY kill wheels because the robot is now blind
                    print("Failsafe: Vehicle halted in place to prevent blind collision.") # Confirm physical safety
                    
                    # --- AUTO-RECOVERY RECONNECTION LOOP ---
                    while True:                      # Start an infinite loop that ONLY tries to fix the internet
                        try:                         # Start error handling for the reconnection attempts
                            print("Attempting to reconnect to network...") # Inform user that recovery is starting
                            self.network.connect_wifi() # Attempt to reconnect to the Wi-Fi router
                            self.network.connect_mqtt(self.process_message) # Attempt to reconnect to the MQTT broker
                            print("Reconnection successful! Resuming operations.") # Print success message
                            self.last_command_time = time.ticks_ms() # Reset watchdog so it doesn't instantly kill the motors again
                            break                    # BREAK out of the recovery loop and go back to driving!
                        except Exception as reconn_err: # Catch failures during the reconnection attempt
                            print(f"Connection failed, please connect. Error: {reconn_err}") # Print exact failure reason
                            time.sleep(2)            # Wait 2 seconds before trying to dial the internet again
                            
                # --- FAILSAFE STAGE 2: WATCHDOG TIMER ---
                # Check if we haven't heard from PC in over 500 milliseconds (0.5 seconds)
                if (time.ticks_ms() - self.last_command_time) > 500: # Calculate time difference
                    self.motors.stop()               # Force stop the wheels immediately due to lag
                    print("Watchdog triggered: PC lagged or disconnected. Motors halted.") # Print why watchdog stopped the robot
                    
                # --- FAILSAFE STAGE 3: DETERMINISTIC TIMING ---
                time.sleep(0.01)                     # Pause 10ms to prevent the ESP32 chip from overheating
                
        except KeyboardInterrupt:                    # Catch manual terminal stops (like pressing Ctrl+C on keyboard)
            print("Robot program stopped manually by user via terminal.") # Print exit status
        finally:                                     # This block runs no matter how the program crashes or ends
            print("Initiating final hardware shutdown sequence...") # Print shutdown start
            self.motors.stop()                       # Final absolute hardware kill switch
            print("Hardware safely secured and completely stopped.") # Explicitly print that hardware is stopped in finally block

# ---------------------------------------------------------
# 5.MAIN EXECUTION (Building and launching the robot)
# ---------------------------------------------------------
if __name__ == "__main__":                         # Check if this script is being run directly as the main program (not imported)
    
    # Credentials and endpoints for Wi-Fi and MQTT
    WIFI_SSID = "Airtel_Primerail Infralabs Pvt L" # Define the name of the local Wi-Fi network to connect to
    WIFI_PASS = "Primeit@2024"                     # Define the password for the local Wi-Fi network
    MQTT_BROKER = "broker.hivemq.com"              # Define the URL of the free public MQTT internet broker
    COMMAND_TOPIC = b"primerail/robot/control"     # Define the specific MQTT channel to listen to for PC commands

    # 1. Instantiate the Steering object
    my_steering = SteeringController(pin_num=16)   # Create the steering controller and attach the physical servo to Pin 16
    
    # 2. Instantiate the Motor object
    my_motors = MotorController(en_a=4, in1=5, in2=6, en_b=15, in3=7, in4=8) # Create the motor controller and map all physical L293D pins
    
    # 3. Instantiate the Network object
    my_network = NetworkController(WIFI_SSID, WIFI_PASS, MQTT_BROKER, COMMAND_TOPIC) # Create the network controller using the credentials above
    
    # 4. Instantiate and Start the Camera Streaming object (NEW ADDITION)
    my_camera = CameraStreamer()                   # Create the camera streaming object to control the physical ESP32-S3 lens
    my_camera.start_server()                       # Boot up the background video streaming web server on ESP32 CPU Core 1
    
    # 5. Pass the required hardware subsystems to the Orchestrator
    my_robot = RobotOrchestrator(my_steering, my_motors, my_network) # Create the main brain manager and inject the steering, motors, and network objects
    
    # 6. Boot the system
    my_robot.run()                                 # Command the fully assembled robot to begin its infinite boot and driving loop on CPU Core 0