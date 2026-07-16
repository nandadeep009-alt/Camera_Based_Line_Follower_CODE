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
import time                         # Imports the timing library to create safe pauses
from machine import Pin, PWM        # Imports hardware controls for copper pins and electrical pulses
from umqtt.simple import MQTTClient # Imports the lightweight MQTT networking tool

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
    def __init__(self, en_a, in1, in2, en_b, in3, in4):
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
        self.in4.value(0)


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
# 3. ORCHESTRATOR CLASS (The Brain)
# ---------------------------------------------------------
class RobotOrchestrator:
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


# ---------------------------------------------------------
# MAIN EXECUTION (Building and launching the robot)
# ---------------------------------------------------------
if __name__ == "__main__":
    # Credentials and endpoints
    WIFI_SSID = "Airtel_Primerail Infralabs Pvt L"
    WIFI_PASS = "Primeit2024"
    MQTT_BROKER = "broker.hivemq.com"
    COMMAND_TOPIC = b"primerail/robot/control"

    # 1. Instantiate the Steering object (Servo on Pin 16)
    my_steering = SteeringController(pin_num=16)
    
    # 2. Instantiate the Motor object (EN_A=4, IN1=5, IN2=6, EN_B=15, IN3=7, IN4=8)
    my_motors = MotorController(en_a=4, in1=5, in2=6, en_b=15, in3=7, in4=8)
    
    # 3. Instantiate the Network object
    my_network = NetworkController(WIFI_SSID, WIFI_PASS, MQTT_BROKER, COMMAND_TOPIC)
    
    # 4. Pass all three subsystems to the Orchestrator
    my_robot = RobotOrchestrator(my_steering, my_motors, my_network)
    
    # 5. Boot the system
    my_robot.run()