# ROVE — Project Structure

This document explains the main folders and files in the ROVE repository, and what belongs in each one going forward.

---

## Repository Overview

```text
Camera_Based_Line_Follower_CODE/
├── README.md
├── requirements.txt
├── .gitignore
├── data_definitions.py
│
├── pc/
│   ├── robot.py
│   ├── config.example.py
│   ├── pc_stream.py
│   ├── pc_mqtt.py
│   ├── pc_vision.py
│   ├── pc_command.py
│   ├── webots_adapter.py
│   └── backups/
│
├── esp32/
│   ├── main.py
│   ├── orch.py
│   ├── net.py
│   ├── hw.py
│   ├── cam.py
│   ├── vision.py
│   └── pin_config.py
│
├── micropython_config/
│   ├── board_pins.json
│   ├── deploy_config.json
│   ├── hardware_calibration.json
│   ├── network_template.json
│   ├── project_manifest.json
│   └── vision_params.json
│
├── webots/
│   ├── project/
│   └── worlds/
│
├── docs/
│   ├── SETUP.md
│   ├── ROADMAP.md
│   └── PROJECT_STRUCTURE.md
│
├── Archive_Old_Versions/
└── ESP32_Brain/
```

---

## Folder Purpose

### `pc/` — PC Master Control Node
Runs on a computer. Captures video (from a real ESP32-CAM stream or the Webots adapter), processes it with OpenCV to find the line, computes a steering/speed command, and publishes it over MQTT.

- `robot.py` — entry point; wires the stream, vision, and command modules together.
- `config.example.py` — safe template. Copy to `config.py` locally; never commit the real file.
- `pc_stream.py` — threaded video capture with auto-reconnect.
- `pc_vision.py` — adaptive thresholding, centroid/moment detection, P-controller, recovery state machine.
- `pc_mqtt.py` — outbound MQTT client with reconnect backoff.
- `pc_command.py` — orchestrator that turns vision output into drive commands (`RobotCommander`).
- `webots_adapter.py` — swaps a real camera/vehicle for the Webots simulation so the same pipeline runs in sim or on hardware.
- `backups/` — local working backups kept during development. Not part of the active build; safe to prune before release.

### `esp32/` — ESP32 MicroPython Agent
Runs on the ESP32-S3. Subscribes to MQTT commands, applies a watchdog, and drives the motors/servo.

- `main.py` — boot script and main loop.
- `orch.py` — orchestrator and 500 ms watchdog/failsafe logic.
- `net.py` — Wi-Fi and MQTT connection management.
- `hw.py` — L298N motor driver and PWM servo output.
- `cam.py` — ESP32-CAM stream server interface.
- `vision.py` — lightweight on-device vision helpers.
- `pin_config.py` — GPIO pin mapping for the board.

### `micropython_config/`
JSON templates used at deployment time — GPIO allocation, Wi-Fi/broker defaults, PWM calibration bounds, and vision parameters. None of these should contain real Wi-Fi passwords or secrets; use them as templates and keep device-specific real values off Git where they contain anything sensitive.

### `webots/`
The Webots simulation project: world file(s) and the virtual vehicle/camera used to validate the vision and control pipeline before deploying to physical hardware.

### `docs/`
All project documentation. `SETUP.md` is the entry point for a new developer; `ROADMAP.md` tracks phase-by-phase progress; this file documents the layout.

### `Archive_Old_Versions/`
Superseded code retained intentionally for future reference and recovery. It is not imported by the active build.

### `ESP32_Brain/`
Contains the pre-built MicroPython firmware image used for the ESP32-S3. It is retained intentionally for future flashing and recovery.

---

## A Note on `.gitignore`

Files that are already tracked by Git remain tracked even if they later match an entry in `.gitignore`.

`Archive_Old_Versions/` and `ESP32_Brain/` are intentionally retained in this repository for reference, firmware recovery, and future development. Do not remove or untrack them unless the repository maintenance policy changes in the future.
