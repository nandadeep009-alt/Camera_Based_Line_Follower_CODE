# ROVE — Development Roadmap

This document shows what has been completed, what is currently being developed, and what remains to be done.

## Project Stages

### Phase 1 — Project Foundation
- [x] Repository created
- [x] Git branch structure established
- [x] PC software separated from ESP32 software
- [x] Webots simulation environment added
- [x] Basic project structure established

### Phase 2 — Webots Simulation
- [x] Webots road environment created
- [x] Virtual vehicle created
- [x] Virtual camera pipeline created
- [x] PC/Webots adapter implemented
- [x] Virtual MQTT controller implemented
- [x] Hardware-in-the-Loop simulation established

### Phase 3 — Vision System
- [x] Camera input implemented
- [x] Line detection implemented
- [x] Vision processing integrated with vehicle control
- [x] Debug image generation implemented
- [ ] Improve robustness for different road conditions
- [ ] Improve detection under poor lighting
- [ ] Evaluate alternative road-marking identification methods

### Phase 4 — Vehicle Control
- [x] Steering command generation
- [x] Speed control
- [x] Search behaviour when line is lost
- [x] Reverse recovery behaviour
- [ ] Final control tuning
- [ ] Full fail-safe validation

### Phase 5 — ESP32 Hardware
- [x] ESP32 software architecture established
- [x] Network layer established
- [x] Hardware control layer established
- [x] Camera layer established
- [x] Orchestration layer established
- [ ] Hardware-in-the-loop validation with physical ESP32
- [ ] Motor controller integration
- [ ] Steering hardware integration
- [ ] Watchdog and emergency-stop validation

### Phase 6 — Safety
- [ ] Communication timeout handling
- [ ] Camera failure detection
- [ ] Vision failure detection
- [ ] Motor command timeout
- [ ] Emergency stop behaviour
- [ ] Safe-state transition
- [ ] Fault logging
- [ ] Recovery-state validation

### Phase 7 — Real Vehicle Testing
- [ ] Static hardware testing
- [ ] Low-speed testing
- [ ] Controlled-track testing
- [ ] Obstacle testing
- [ ] Line-loss testing
- [ ] Communication-loss testing
- [ ] Extended-duration testing

### Phase 8 — Release
- [ ] Final documentation
- [ ] Installation guide
- [ ] Hardware configuration guide
- [ ] Software configuration guide
- [ ] Test results
- [ ] Known limitations
- [ ] Release version/tag

---

## Current Priority

The immediate development priority is:

1. Stabilize Webots simulation.
2. Validate camera-based vision.
3. Validate steering and speed control.
4. Complete fail-safe behaviour.
5. Prepare the software for physical ESP32 integration.

---

## Important Rule

A feature is considered complete only after it has been tested and documented.

The roadmap should be updated whenever a major development milestone is completed.
