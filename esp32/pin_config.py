# esp32/pin_config.py
# -----------------------------------------------------------------------------
# GLOBAL SETTINGS & HARDWARE WIRING INFRASTRUCTURE MAP (ESP32 DevKit v1)
# -----------------------------------------------------------------------------
# Single source of truth for all electrical connections, power paths, and flags.

# -----------------------------------------------------------------------------
# NETWORK & MQTT CONFIGURATION CREDENTIALS
# Added missing variables required by main.py to prevent AttributeError
# -----------------------------------------------------------------------------
WIFI_SSID = "Your_WiFi_SSID"
WIFI_PASS = "Your_WiFi_Password"
MQTT_BROKER = "127.0.0.1"   #"192.168.1.100"
MQTT_USER = None
MQTT_PASS = None
COMMAND_TOPIC = b"rove/actuators" # Topic defined as byte string for MQTT client compatibility
FLEET_ALERT_TOPIC = b"rove/alerts"  
COMMAND_SECRET = b"simulation_security_token_123"  # Authentication shared secret token
MOTION_ARMED = False              # Safety flag: Set to True to allow physical motor movement

# -----------------------------------------------------------------------------
# DEBUG CONFIGURATION FLAG — controls high-frequency serial logging latency
# -----------------------------------------------------------------------------
# True  = verbose logging (development mode). flushes UART which introduces latency.
# False = safety-critical printing only (production/deployment mode). Max loop speed.
DEBUG = False                               

# -----------------------------------------------------------------------------
# POWER & GROUND DISPATCH SCHEMATIC REFERENCE (Wiring Guide)
# -----------------------------------------------------------------------------
# Battery System: LiPo / LiFePO4 (7.4V - 14.8V Raw Power Input Rail)
#   --> Directly connects to L293D pin 8 (VCC2 / Motor Power Supply Rail)
#   --> Directly connects to DSN5000 Buck Converter Input (IN+ / IN-)
#
# DSN5000 Buck Converter System (Stepped down to 5.0V Constant Supply Rail)
#   --> Output OUT+ connects to L293D pin 16 (VCC1 / Logic Power Supply 5V)
#   --> Output OUT+ connects to MG90S Servo VCC Red wire (5V High-Current Supply)
#   --> Output OUT+ connects to ESP32 DevKit v1 Vin / 5V Input pin
#
# Common Ground System Matrix (CRITICAL FOR UNIFIED ELECTRICAL SIGNAL REFERENCE)
#   --> All GND nodes (Battery Minus, DSN5000 OUT-, L293D pins 4/5/12/13, MG90S 
#       Brown wire, and ESP32 GND pins) MUST be physically tied together.

# -----------------------------------------------------------------------------
# ACTUATOR GPIO CONTROL MAPPINGS
# -----------------------------------------------------------------------------
# MG90S Steering Servo Control Pin (Requires precise 50Hz hardware PWM)
STEERING_SERVO_PIN = 2                      # ESP32 GPIO2 connected directly to MG90S Servo Orange Signal wire

# L293D H-Bridge Dual DC Motor Driver Control Interface Pins
MOTOR_EN_A = 12                             # ESP32 GPIO12 connected to L293D Pin 1 (Master Speed Enable for Left Motor)
MOTOR_IN1  = 13                             # ESP32 GPIO13 connected to L293D Pin 2 (Direction Phase Control 1 for Left Motor)
MOTOR_IN2  = 14                             # ESP32 GPIO14 connected to L293D Pin 7 (Direction Phase Control 2 for Left Motor)

MOTOR_EN_B = 15                             # ESP32 GPIO15 connected to L293D Pin 9 (Master Speed Enable for Right Motor)
MOTOR_IN3  = 16                             # ESP32 GPIO16 connected to L293D Pin 10 (Direction Phase Control 3 for Right Motor)
MOTOR_IN4  = 17                             # ESP32 GPIO17 connected to L293D Pin 15 (Direction Phase Control 4 for Right Motor)

# -----------------------------------------------------------------------------
# UPCOMING AUTONOMOUS SENSOR EXPANSION RESERVATIONS (Do not use these for actuators)
# -----------------------------------------------------------------------------
ULTRASONIC_TRIG = 4                         # Reserved for HC-SR04 ultrasonic echo trigger output
ULTRASONIC_ECHO = 5                         # Reserved for HC-SR04 ultrasonic flight duration echo input
I2C_SDA_PIN     = 21                        # Dedicated hardware I2C data lane for VL53L0X ToF and MPU6050 IMU
I2C_SCL_PIN     = 22                        # Dedicated hardware I2C clock line for VL53L0X ToF and MPU6050 IMU