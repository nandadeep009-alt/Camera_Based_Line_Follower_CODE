# esp32/hw.py
# -----------------------------------------------------------------------------
# LOW-LEVEL HARDWARE CONTROLLER LAYER (The Vehicle Muscle Drivers)
# -----------------------------------------------------------------------------
# Abstracts physical micro-controller hardware registers into clean method APIs.

from machine import Pin, PWM                 # type: ignore #Import native ESP32 micro-controller hardware interface control classes (suppress VS Code warning)
import machine
import utime as time
import pin_config                            # Link our central hardware wiring configuration file maps

class SteeringController:                           # Define the SteeringController class to manage the servo motor that physically turns the front wheels
    def __init__(self, pin_num):                    # Constructor method - takes the GPIO pin number that the servo signal wire is connected to
        self.servo = PWM(Pin(pin_num))              # Create a PWM object on the specified pin - PWM (Pulse Width Modulation) is the electrical language servo motors understand
        self.servo.freq(50)                         # Set the PWM signal to 50Hz (one pulse every 20ms) - this is the mandatory frequency all hobby servo motors expect; any other value will confuse or damage the servo
        self.set_angle(90)                          # Immediately command the wheels to the straight-ahead center position on boot, so the vehicle never starts in a turned state

    def set_angle(self, angle):                     # Method to physically turn the front wheels to a specific angle (45=full left, 90=straight, 135=full right)
        safe_angle = max(45, min(135, angle))       # SAFETY CLAMP: force the angle into the allowed 45-135 degree window before touching hardware - if the caller passes 20 or 200 this saves the servo gears from being stripped
        duty = int(((safe_angle / 180.0) * (115 - 40)) + 40) # Convert the human-readable degree number into a PWM duty cycle integer that the ESP32's PWM hardware understands (40 = full left at 45°, 115 = full right at 135°, linear interpolation between)
        self.servo.duty(duty)                       # Send the calculated duty cycle to the physical servo pin, which moves the servo horn and the steering linkage to the target angle


class MotorController:                              # Define the MotorController class to manage the two DC drive motors via the L293D H-bridge driver chip
    def __init__(self, en_a, en_b, in1, in2, in3, in4): # Constructor - takes the six GPIO pin numbers that connect the ESP32 to the L293D chip (two enable pins + four direction pins)
        self.en_a = Pin(en_a, Pin.OUT)              # Initialize EN_A as a digital output - this is the master power enable pin for the LEFT motor; LOW = motor coast/off, HIGH = motor actively driven
        self.in1 = Pin(in1, Pin.OUT)                # Initialize IN1 as a digital output - this is the LEFT motor's FORWARD direction pin; HIGH here with IN2 LOW = forward spin
        self.in2 = Pin(in2, Pin.OUT)                # Initialize IN2 as a digital output - this is the LEFT motor's REVERSE direction pin; HIGH here with IN1 LOW = reverse spin
        self.en_b = Pin(en_b, Pin.OUT)              # Initialize EN_B as a digital output - this is the master power enable pin for the RIGHT motor; same logic as EN_A
        self.in3 = Pin(in3, Pin.OUT)                # Initialize IN3 as a digital output - this is the RIGHT motor's FORWARD direction pin
        self.in4 = Pin(in4, Pin.OUT)                # Initialize IN4 as a digital output - this is the RIGHT motor's REVERSE direction pin
        self.stop()                                 # SAFETY: call stop() immediately on boot so all pins start LOW and the wheels cannot spin before the rest of the system is ready
        print("MotorController initialized and secured on boot.") # Confirm over serial that the motor driver is set up and the wheels are locked

    def drive_forward(self):                        # Method to command both motors to spin forward
        self.en_a.value(1)                          # Turn ON Left master power via L293D Enable A pin to feed regulated battery voltage to left coils
        self.en_b.value(1)                          # Turn ON Right master power via L293D Enable B pin to feed regulated battery voltage to right coils
        self.in1.value(1)                           # Pull Left Forward logic gate high (3.3V bias to driver)
        self.in2.value(0)                           # Pull Left Reverse logic gate low (0V reference to driver)
        self.in3.value(1)                           # Pull Right Forward logic gate high (3.3V bias to driver)
        self.in4.value(0)                           # Pull Right Reverse logic gate low (0V reference to driver)
        if pin_config.DEBUG:                        # Gate this print behind global configuration status to save precious loop processing time allocations
            print("Motors driving forward.")         # Send active state tracking update across serial channel to connected workstation console

    def drive_reverse(self):                        # Method to command both motors to spin backward for line-recovery maneuvers
        self.en_a.value(1)                          # Turn ON Left master power via L293D Enable A pin to feed regulated battery voltage to left coils
        self.en_b.value(1)                          # Turn ON Right master power via L293D Enable B pin to feed regulated battery voltage to right coils
        self.in1.value(0)                           # Pull Left Forward logic gate low (0V reference to driver)
        self.in2.value(1)                           # Pull Left Reverse logic gate high (3.3V bias to driver)
        self.in3.value(0)                           # Pull Right Forward logic gate low (0V reference to driver)
        self.in4.value(1)                           # Pull Right Reverse logic gate high (3.3V bias to driver)
        if pin_config.DEBUG:                        # Gate this print behind global configuration status to save precious loop processing time allocations
            print("Motors driving in reverse.")       # Send active state tracking update across serial channel to connected workstation console

    def stop(self):                                 # Method to completely halt all motor activity and cut power
        self.en_a.value(0)                          # Pull Left Enable pin low to decouple current loop inside H-bridge to eliminate motor jitter
        self.en_b.value(0)                          # Pull Right Enable pin low to decouple current loop inside H-bridge to eliminate motor jitter
        self.in1.value(0)                           # Reset logic pins to ground state to ensure safe structural idle state
        self.in2.value(0)                           # Reset logic pins to ground state to ensure safe structural idle state
        self.in3.value(0)                           # Reset logic pins to ground state to ensure safe structural idle state
        self.in4.value(0)                           # Reset logic pins to ground state to ensure safe structural idle state
        if pin_config.DEBUG:                        # Gate this print behind global configuration status to save precious loop processing time allocations
            print("Motors stopped and power cut.")     # Send active state tracking update across serial channel to connected workstation console


# =============================================================================
# HC-SR04 ULTRASONIC DISTANCE SENSOR
# =============================================================================
#
# Purpose:
#     Independent close-range safety sensor.
#
# IMPORTANT:
#     This sensor does NOT decide left/right avoidance.
#
# Camera:
#     Detects obstacle
#     Decides avoidance direction
#     Reacquires path
#
# Ultrasonic:
#     Detects dangerously close object
#     Commands STOP
#
# =============================================================================


class UltrasonicSensor:

    STOP_DISTANCE_CM = 55.0

    def __init__(
        self,
        trig_pin,
        echo_pin,
        sample_interval_ms=100
    ):

        self._trig = Pin(
            trig_pin,
            Pin.OUT
        )

        self._echo = Pin(
            echo_pin,
            Pin.IN
        )

        self._trig.value(0)

        self._sample_interval_ms = (
            sample_interval_ms
        )

        self._last_sample_ms = 0

        self._last_distance_cm = None

    def _measure_once(self):

        try:

            # Send 10 microsecond trigger pulse.

            self._trig.value(0)

            time.sleep_us(2)

            self._trig.value(1)

            time.sleep_us(10)

            self._trig.value(0)

            # Measure HIGH echo pulse.

            duration_us = (
                machine.time_pulse_us(
                    self._echo,
                    1,
                    30000
                )
            )

            # Negative result means timeout/error.

            if duration_us < 0:

                return None

            # Speed of sound conversion:
            #
            # distance_cm ≈ time_us / 58

            distance_cm = (
                float(duration_us)
                / 58.0
            )

            # Reject impossible values.

            if (
                distance_cm < 2.0
                or
                distance_cm > 400.0
            ):

                return None

            return distance_cm

        except Exception as err:

            print(
                "Ultrasonic measurement error:",
                err
            )

            return None

    def read_cm(self):

        now = time.ticks_ms()

        # Do not trigger the sensor every motor-control cycle.
        # Use the most recent valid measurement.

        if (
            time.ticks_diff(
                now,
                self._last_sample_ms
            )
            <
            self._sample_interval_ms
        ):

            return (
                self._last_distance_cm
            )

        self._last_sample_ms = now

        distance_cm = (
            self._measure_once()
        )

        self._last_distance_cm = (
            distance_cm
        )

        return distance_cm

    def is_blocked(self):

        distance_cm = (
            self.read_cm()
        )

        # Invalid sensor reading = unsafe.
        if distance_cm is None:

            return True

        return (
            distance_cm
            <=
            self.STOP_DISTANCE_CM
        )

    def get_distance(self):

        return self._last_distance_cm