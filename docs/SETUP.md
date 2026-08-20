# ROVE — Complete Setup Guide

This document explains how to set up and run the ROVE Camera-Based Line Follower project on a new computer.

The goal is simple:

> Clone the project → install the required software → install Python packages → configure the system → run the simulation.

No previous knowledge of the project is required.

---

## 1. What is ROVE?

ROVE is an autonomous camera-based line-following vehicle software system.

It contains:

- PC software for vision, decision making and simulation
- ESP32 software for real vehicle hardware control
- Webots simulation for testing without physical hardware
- Configuration files for deployment and calibration

During development, Webots allows the vehicle and road environment to be simulated before using physical hardware.

---

## 2. Project Architecture

```text
ROVE
 |
 +-- PC SOFTWARE
 |    |
 |    +-- Camera / Simulation Input
 |    +-- Vision Processing
 |    +-- Decision Making
 |    +-- MQTT Commands
 |
 +-- ESP32 SOFTWARE
 |    |
 |    +-- Wi-Fi
 |    +-- MQTT
 |    +-- Watchdog
 |    +-- Motor Control
 |    +-- Steering Control
 |
 +-- WEBOTS
      |
      +-- Virtual Vehicle
      +-- Virtual Camera
      +-- Road Environment
      +-- Hardware-in-the-Loop Testing
camera_line_follower/
|
+-- README.md
+-- requirements.txt
+-- .gitignore
+-- config.py
+-- data_definitions
|
+-- pc/
|   +-- pc_stream.py
|   +-- pc_mqtt.py
|   +-- pc_vision.py
|   +-- pc_command.py
|
+-- esp32/
|   +-- main.py
|   +-- orch.py
|   +-- net.py
|   +-- hw.py
|   +-- cam.py
|   +-- pin_config.py
|
+-- micropython_config/
|   +-- board_pins.json
|   +-- deploy_config.json
|   +-- hardware_calibration.json
|   +-- network_template.json
|   +-- project_manifest.json
|   +-- vision_params.json
|
+-- webots/
|   +-- worlds/
|   +-- controllers/
|   +-- assets/
|
+-- Archive_Old_Versions/
|
+-- docs/
    +-- SETUP.md

---

## 3. Important Safety and Security Rule

This repository must NEVER contain real passwords, Wi-Fi credentials, MQTT passwords, API keys, tokens, or other private secrets.

The repository has already been cleaned of the previously exposed Wi-Fi password from Git history.

Do not put real credentials into:

- README.md
- SETUP.md
- Python source files
- ESP32 source files
- JSON configuration files
- Git commits
- GitHub Issues
- GitHub Discussions

Use local configuration files that are excluded by `.gitignore`.

For example:

```text
config.py
```

