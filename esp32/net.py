# esp32/net.py
import network                      # type: ignore # Import ESP32 Wi-Fi antenna driver module (suppress VS Code linter warning)
import utime as time                # type: ignore #Import time functions for millisecond ticks and non-blocking delays
from umqtt.simple import MQTTClient # type: ignore # Import lightweight MicroPython MQTT client library (suppress VS Code linter warning)

DEBUG = False                       # Flag to control high-frequency serial logging to reduce UART latency

class NetworkController:
    def __init__(self, ssid, password, broker, topic, mqtt_user=None, mqtt_pass=None):
        self.ssid = ssid            # Store the target Wi-Fi access point SSID
        self.password = password    # Store the Wi-Fi network password
        self.broker = broker        # Store the IP address/hostname of the MQTT broker
        self.topic = topic          # Store the MQTT topic byte string to subscribe to
        self.mqtt_user = mqtt_user  # Store optional username for authenticated MQTT brokers
        self.mqtt_pass = mqtt_pass  # Store optional password for authenticated MQTT brokers
        self.client = None          # Initialize the MQTT client reference as None until connected

    def connect_wifi(self, timeout_s=15, watchdog=None):
        wlan = network.WLAN(network.STA_IF) # Instantiates standard station interface (client mode)
        wlan.active(True)                   # Turn on the Wi-Fi radio antenna
        print("Connecting to Wi-Fi...")     # Log the Wi-Fi connection attempt to serial
        wlan.connect(self.ssid, self.password) # Send connection credentials to the router
        
        start_time = time.ticks_ms()        # Capture the starting time tick in milliseconds
        while not wlan.isconnected():       # Loop until an IP address is granted by DHCP
            if watchdog is not None:        # Check if a hardware watchdog object was provided
                watchdog.feed()             # Feed the watchdog to prevent hardware reset during connection
            if time.ticks_diff(time.ticks_ms(), start_time) > timeout_s * 1000: # Calculate elapsed time vs timeout
                wlan.active(False)          # Deactivate radio on timeout to release hardware resources
                raise OSError("Wi-Fi connection timed out after {}s".format(timeout_s)) # Raise error for caller
            time.sleep(0.5)                 # Pause execution briefly for 500ms between connectivity checks
            print(".", end="")              # Print progress dot without newlines
            
        print("\nWi-Fi Connected! IP:", wlan.ifconfig()[0]) # Output granted local IP address upon success

    def connect_mqtt(self, message_callback):
        mac_bytes = network.WLAN(network.STA_IF).config("mac") # Extract raw 6-byte MAC address from Wi-Fi chip
        mac_hex = "".join("{:02x}".format(b) for b in mac_bytes) # Convert binary MAC address into a hex string
        client_id = "esp32_robot_{}".format(mac_hex) # Generate unique client ID using MAC hex string
        print(f"MQTT: using unique client ID '{client_id}'") # Log unique client ID to serial output
        self.client = MQTTClient(client_id, self.broker, user=self.mqtt_user, password=self.mqtt_pass) # Construct MQTT client
        
        self.client.set_callback(message_callback) # Register function to execute when new messages arrive
        print("Connecting to MQTT Broker...")     # Output broker connection status
        self.client.connect()                      # Open TCP socket connection to MQTT broker
        self.client.subscribe(self.topic)         # Subscribe to the assigned control topic
        print("Subscribed to topic:", self.topic.decode()) # Log subscribed topic string

    def check_for_commands(self):
        if self.client is None:             # Check if MQTT client instance exists
            raise OSError("MQTT client is None — connect_mqtt() has not been called yet or was skipped due to a failed boot. Reconnect required.") # Raise exception if client is uninitialized
        if DEBUG:                           # Check if verbose debug mode is enabled
            print("NetworkController: polling MQTT broker for waiting commands...") # Print poll notification
        self.client.check_msg()             # Perform non-blocking check on TCP socket for new MQTT packets