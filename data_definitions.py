"""
=========================================================================================
DATA DEFINITIONS & PROTOCOL SPECIFICATIONS
=========================================================================================
Project: Camera Line Follower System
Modules: PC Vision Master & ESP32 MicroPython Agent

This file serves as the single source of truth for shared data types, state machine
definitions, MQTT wire formats, and hardware mechanical bounds across PC and ESP32.
=========================================================================================
"""

from typing import Literal, Union, Dict, Any

# =======================================================================================
# 1. PC & SYSTEM-WIDE DATA TYPES
# =======================================================================================

# SteeringAngle
# Range: [45, 135] (Integer)
# Interpretation: Mechanical degrees for the front axle servo motor.
# - 90  : Wheels pointing straight ahead (Neutral).
# - 45  : Full lock left turn.
# - 135 : Full lock right turn.
SteeringAngle = int

SERVO_MIN_ANGLE: int = 45
SERVO_CENTER_ANGLE: int = 90
SERVO_MAX_ANGLE: int = 135

# MotorState
# Interpretation: Activation state of the rear drive propulsion motors / L298N H-Bridge logic.
# - "DRIVE"   : Rear wheels spin forward to move the vehicle.
# - "STOP"    : Rear wheels lose power completely (Safety / Neutral).
# - "REVERSE" : Rear wheels spin backward to re-track a lost line during recovery.
MotorState = Literal["DRIVE", "STOP", "REVERSE"]

# VisionState
# Interpretation: Internal recovery state machine operating inside PC VisionController.
# - "DRIVE"       : Line actively detected; normal forward line tracking.
# - "SEARCH_STOP" : Line lost; vehicle held stationary for 3.5s to check for transient loss.
# - "REVERSE"     : Line still missing; vehicle mirrors last valid angle in reverse for max 3.0s.
VisionState = Literal["DRIVE", "SEARCH_STOP", "REVERSE"]


# =======================================================================================
# 2. MQTT NETWORK PROTOCOL & PAYLOAD SCHEMA
# =======================================================================================

# Command Payload Schema (PC -> ESP32)
# Topic: COMMAND_TOPIC (e.g., "robot/control/command")
# String Format (CSV): "{COMMAND_SECRET},{SteeringAngle},{MotorState}"
# Examples:
#   "MySecretKey123,90,DRIVE"   -> Straight forward drive
#   "MySecretKey123,45,DRIVE"   -> Hard left drive
#   "MySecretKey123,90,STOP"    -> Immediate stop
#   "MySecretKey123,135,REVERSE"-> Hard right reverse

COMMAND_PAYLOAD_DELIMITER = ","

# Fleet Alert Payload Schema (PC <-> Fleet Network)
# Topic: FLEET_ALERT_TOPIC (e.g., "robot/alerts/status")
# String Format (CSV): "{COMMAND_SECRET},{ALERT_LEVEL},{MESSAGE}"
# Examples:
#   "MySecretKey123,WARNING,CAMERA_FAILURE"
#   "MySecretKey123,CRITICAL,MQTT_DISCONNECT"


# =======================================================================================
# 3. ESP32 HARDWARE & PIN CONFIGURATION DEFINITIONS
# =======================================================================================

# ESP32 Watchdog Timers
MQTT_WATCHDOG_TIMEOUT_MS: int = 500  # Halts motors if no valid command received within 500ms

# Steering Servo PWM Settings
SERVO_PWM_FREQ_HZ: int = 50          # Standard 50Hz RC Servo frequency
SERVO_MIN_PULSE_US: int = 1000       # Microseconds corresponding to 0 degrees
SERVO_MAX_PULSE_US: int = 2000       # Microseconds corresponding to 180 degrees

# Pin Mapping (esp32/pin_config.py Reference)
PIN_CONFIG: Dict[str, int] = {
    # Steering Servo
    "SERVO_PWM": 13,
    
    # L298N Dual H-Bridge Motor Driver
    "MOTOR_ENA": 14,   # PWM Speed Enable Pin
    "MOTOR_IN1": 12,   # Forward Direction Logic Pin
    "MOTOR_IN2": 27,   # Reverse Direction Logic Pin
    
    # Status Indicators
    "LED_BUILTIN": 33, # System activity / status LED
}


# =======================================================================================
# 4. VALIDATION & SANITIZATION HELPERS
# =======================================================================================

def sanitize_steering_angle(angle: int) -> SteeringAngle:
    """Clamps input angle to legal mechanical servo boundaries [45, 135]."""
    return max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, int(angle)))


def sanitize_motor_state(state: str) -> MotorState:
    """Validates motor state string, defaulting to fail-safe 'STOP' on invalid input."""
    valid_states = {"DRIVE", "STOP", "REVERSE"}
    if state in valid_states:
        return state  # type: ignore[return-value]
    return "STOP"