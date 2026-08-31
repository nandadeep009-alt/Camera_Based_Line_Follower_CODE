# Camera_Based_Line_Follower_CODE

## Autonomous Camera Line Follower System (ROVE)

A real-time, vision-guided autonomous line-following vehicle architecture using a **PC Master Control Node** for computer vision processing and an **ESP32 MicroPython Agent** for low-level motor/servo actuation over MQTT.

---

## System Architecture

```text
                        /--- System Architecture ---/
                                     |
                     +-------------------------------+
                     |      ESP32 Camera Module       |
                     +---------------+---------------+
                                     |
                           (HTTP MJPEG Stream)
                                     v
+-----------------------------------------------------------------------------+
|                               PC MASTER NODE                                |
|  +------------------+    +-------------------+    +----------------------+  |
|  |   pc_stream.py   | -> |   pc_vision.py    | -> |    pc_command.py     |  |
|  | Threaded Capture |    | OpenCV Centroid/P |    |  RobotCommander Hub  |  |
|  +------------------+    +-------------------+    +----------+-----------+  |
+--------------------------------------------------------------|--------------+
                                                                |
                                                   (MQTT CSV Command Stream)
                                                                v
                     +-------------------------------+
                     |       MQTT Broker (1883)       |
                     +---------------+---------------+
                                     |
                                     v
+-----------------------------------------------------------------------------+
|                              ESP32 AGENT NODE                               |
|                                                                             |
|  +------------------+    +-------------------+    +----------------------+  |
|  |      net.py      | -> |      orch.py      | -> |        hw.py         |  |
|  | MQTT Subscriber  |    | Watchdog / Router |    | L298N & Servo Output  |  |
|  +------------------+    +-------------------+    +----------------------+  |
+-----------------------------------------------------------------------------+
```

---

## 📁 Repository Directory Structure

```text
Camera_Based_Line_Follower_CODE/
├── README.md                          # System overview & documentation index
├── requirements.txt                   # Python dependencies
├── .gitignore
├── data_definitions.py                # Unified data type declarations & wire specs
│
├── pc/                                 # PC Master Control Node
│   ├── robot.py                        # Main PC application entry point
│   ├── config.example.py               # Template — copy to pc/config.py locally
│   ├── pc_stream.py                    # Threaded video capture with auto-reconnect
│   ├── pc_mqtt.py                      # Outbound MQTT client with auto-reconnect backoff
│   ├── pc_vision.py                    # OpenCV adaptive vision & recovery state machine
│   ├── pc_command.py                   # Main orchestrator (RobotCommander hub)
│   ├── webots_adapter.py               # PC ↔ Webots simulation bridge
│   └── backups/                        # Local backups — not part of the active build
│
├── esp32/                              # ESP32 MicroPython Agent
│   ├── main.py                         # Boot script & main execution loop
│   ├── orch.py                         # Orchestrator & 500ms watchdog logic
│   ├── net.py                          # Wi-Fi & MQTT connection manager
│   ├── hw.py                           # L298N motor driver & PWM servo controller
│   ├── cam.py                          # ESP32-CAM stream server interface
│   ├── vision.py                       # On-device vision helpers
│   └── pin_config.py                   # Hardware GPIO mapping definitions
│
├── micropython_config/                 # Deployment & calibration templates
│   ├── board_pins.json
│   ├── deploy_config.json
│   ├── hardware_calibration.json
│   ├── network_template.json
│   ├── project_manifest.json
│   └── vision_params.json
│
├── webots/                             # Webots simulation environment
│   ├── project/
│   └── worlds/
│
├── docs/
│   ├── SETUP.md                        # Install, configure, and run guide
│   ├── ROADMAP.md                      # Development phase tracker
│   └── PROJECT_STRUCTURE.md            # Folder-by-folder explanation
│
├── Archive_Old_Versions/               # Legacy/reference code, not part of the active build
└── ESP32_Brain/                        # Pre-built ESP32 firmware image
```

---

## 📚 Project Documentation

If you are new to ROVE, use the documents below in this order:

1. **[Complete Setup Guide](docs/SETUP.md)**
   Start here if you want to install, configure, and run the project on another computer.

2. **[Development Roadmap](docs/ROADMAP.md)**
   See what has been completed, what is currently being developed, and what remains.

3. **[Project Structure](docs/PROJECT_STRUCTURE.md)**
   Explains the purpose of the PC, ESP32, Webots, configuration, and documentation folders.

---

## 🚀 Quick Start

```text
NEW USER
   |
   v
Read docs/SETUP.md
   |
   v
Install Python + dependencies
   |
   v
Configure local environment (pc/config.py)
   |
   v
Run Webots / PC simulation
   |
   v
Study project structure
   |
   v
Continue development
```

---

## 🔐 Security

Never commit real passwords, Wi-Fi credentials, MQTT credentials, API keys, tokens, or other private information.

Use local configuration files that are excluded from Git (see `pc/config.example.py` and `.gitignore`).

---

## License

No license file is currently present in this repository, which means default copyright applies and others do not have legal permission to reuse this code. If you want people to be able to freely use, modify, or contribute to this project, add a `LICENSE` file (e.g. MIT, Apache-2.0) at the repository root.
