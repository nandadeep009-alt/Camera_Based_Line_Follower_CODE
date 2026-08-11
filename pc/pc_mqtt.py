"""
=========================================================================================
PC MQTT CONTROLLER MODULE (pc_mqtt.py)
-----------------------------------------------------------------------------------------
Handles outbound network communication and payload authentication over MQTT.
Loads secure broker credentials, maintains background heartbeat pings, manages 
auto-reconnection logic, and formats steering/drive commands into secret-authenticated strings.
=========================================================================================
"""

import time  # type: ignore # Import time for handling connection retry pauses
import paho.mqtt.client as mqtt  # type: ignore # Import Paho MQTT client library for network messaging

try:  # type: ignore # Guard attempt to load sensitive configuration parameters from local file
    import config as _cfg  # type: ignore # Import configuration module holding secrets
    MQTT_BROKER = _cfg.MQTT_BROKER  # type: ignore # Extract broker host address from config
    COMMAND_TOPIC = _cfg.COMMAND_TOPIC  # type: ignore # Extract control topic string from config
    FLEET_ALERT_TOPIC = _cfg.FLEET_ALERT_TOPIC  # type: ignore # Extract alert topic string from config
    COMMAND_SECRET = _cfg.COMMAND_SECRET  # type: ignore # Extract shared security secret token from config
    print("pc_mqtt.py: credentials loaded from config.py.")  # type: ignore # Confirm successful configuration import
except ImportError:  # type: ignore # Handle scenario where config.py file is missing
    raise SystemExit(  # type: ignore # Terminate program immediately with actionable diagnostic message
        "FATAL: config.py not found. Create it with MQTT_BROKER, COMMAND_TOPIC, "  # type: ignore
        "FLEET_ALERT_TOPIC, and COMMAND_SECRET before running. "  # type: ignore
    )  # type: ignore


class MQTTController:  # type: ignore # Encapsulates outbound MQTT publishing and connection persistence
    def __init__(self, broker, topic, command_secret, max_connect_attempts=5):  # type: ignore # Initializer method
        self.topic = topic  # type: ignore # Store target publishing MQTT command topic
        self.command_secret = command_secret  # type: ignore # Store shared authentication token
        self.client = mqtt.Client()  # type: ignore # Create Paho MQTT client instance
        self.connected = False  # type: ignore # Track connection status flag
        self.client.on_connect = self._on_connect  # type: ignore # Attach connection callback function
        self.client.on_disconnect = self._on_disconnect  # type: ignore # Attach disconnection callback function

        print(f"Connecting to MQTT Broker at {broker}...")  # type: ignore # Print target broker connection attempt
        self.client.reconnect_delay_set(min_delay=1, max_delay=30)  # type: ignore # Set automatic reconnect exponential backoff limits
        attempt = 0  # type: ignore # Initialize retry counter
        while True:  # type: ignore # Loop through connection attempts up to max threshold
            try:  # type: ignore # Try opening network connection to broker
                self.client.connect(broker, 1883, 60)  # type: ignore # Connect to broker address on port 1883 with 60s keepalive
                print(f"MQTT: initial TCP connection to {broker}:1883 established. Waiting for broker acknowledgement...")  # type: ignore
                break  # type: ignore # Exit retry loop on successful TCP handshake
            except Exception as e:  # type: ignore # Catch network failure exception during connection
                attempt += 1  # type: ignore # Increment connection attempt count
                print(f"MQTT connection attempt {attempt}/{max_connect_attempts} failed: {e}")  # type: ignore # Print attempt error log
                if attempt >= max_connect_attempts:  # type: ignore # Check if retry count exceeded maximum allowable threshold
                    raise ConnectionError(f"Could not reach MQTT broker at {broker} after {max_connect_attempts} attempts.") from e  # type: ignore
                print("Retrying in 2 seconds...")  # type: ignore # Output delay status
                time.sleep(2)  # type: ignore # Sleep before re-attempting connection

        self.client.loop_start()  # type: ignore # Start background thread for processing incoming/outgoing network packets
        print("MQTT: background network loop started. Auto-reconnect is ACTIVE.")  # type: ignore

    def _on_connect(self, client, userdata, flags, rc):  # type: ignore # Callback triggered upon connection status return
        self.connected = (rc == 0)  # type: ignore # Mark connected as True if return code equals 0
        if self.connected:  # type: ignore # If connection is accepted by broker
            print(f"MQTT: successfully connected to broker. Return code: {rc} (0 = accepted).")  # type: ignore
            self.client.subscribe(self.topic)  # type: ignore # Subscribe to topic to verify session capabilities
            print(f"MQTT: re-subscribed to topic '{self.topic}' after (re)connect.")  # type: ignore
        else:  # type: ignore # If connection is rejected by broker
            print(f"MQTT: broker rejected connection. Return code: {rc}. Check credentials/address.")  # type: ignore

    def _on_disconnect(self, client, userdata, rc):  # type: ignore # Callback triggered upon network disconnect
        self.connected = False  # type: ignore # Set connected flag to False
        if rc == 0:  # type: ignore # Handle clean programmatic disconnect
            print("MQTT: cleanly disconnected from broker. Session closed intentionally.")  # type: ignore
        else:  # type: ignore # Handle unintended network disconnect
            print(f"MQTT: unexpected disconnect from broker (rc={rc}). Auto-reconnect active.")  # type: ignore
            print("MQTT: while disconnected, commands are dropped locally; ESP32 watchdog will halt vehicle.")  # type: ignore

    def send_command(self, angle, engine_state):  # type: ignore # Formats and publishes control payload
        payload = f"{self.command_secret},{angle},{engine_state}"  # type: ignore # Format authenticated CSV payload
        if not self.connected:  # type: ignore # Verify active connection status before attempting publish
            print(f"MQTT WARNING: not connected — command dropped: {payload}")  # type: ignore
            return  # type: ignore # Abort publish attempt when offline
        self.client.publish(self.topic, payload)  # type: ignore # Publish command string to MQTT broker
        print(f"MQTT: command sent -> topic='{self.topic}' | payload='{payload}'")  # type: ignore

    def stop(self):  # type: ignore # Terminates MQTT client loop and closes network socket
        print("MQTT: initiating clean shutdown sequence...")  # type: ignore
        self.client.loop_stop()  # type: ignore # Stop background MQTT network thread
        print("MQTT: background network thread stopped.")  # type: ignore
        self.client.disconnect()  # type: ignore # Transmit MQTT disconnect packet and close TCP socket
        print("MQTT: disconnect packet sent. Connection closed.")  # type: ignore