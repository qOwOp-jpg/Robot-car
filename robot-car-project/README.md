# Robot Car Project

Arduino-based autonomous robot car with PID line following, obstacle detection, EEPROM telemetry logging, and Python-based post-run visualization.

## Overview

This project is a robot car built around an Arduino Uno. It follows a black line using PID control, reacts to obstacles with an ultrasonic sensor, and records important run data for later analysis. After each run, the stored telemetry can be dumped through Serial and processed in Python to generate plots and reconstruct the robot's rough path.

### System data flow

**Sensor → Arduino → PID → Motor Driver → Motor → EEPROM Log → Python**

## Main Features

- PID-based line following
- Finite State Machine (FSM) control structure
- Line sensor calibration before each run
- Obstacle detection using an ultrasonic sensor
- Lost-line recovery behavior
- EEPROM-based telemetry logging
- Python visualization for post-run analysis

## Hardware Used

- Arduino Uno R3
- DRV8833 motor driver
- 3 line sensors
- Ultrasonic sensor
- DC motors and robot chassis
- Push button
- Battery pack

## Software Stack

- **Arduino / C++** for real-time robot control
- **Python** for telemetry parsing and visualization

## Repository Structure

```text
robot-car-project/
├── arduino/
│   └── robot_car——v3.ino
├── python/
│   └── visualize_log.py
├── docs/
│   ├── images/
│   └── presentation/
└── README.md
```
