# Medical Delivery Drone Project

This repository contains the mission-control software and technical documentation developed for my final year project:

**Medical Drone for Critical Supply Transport**

The project explores how a Raspberry Pi and Pixhawk-based drone can be used for medical payload delivery through different levels of automation, operator involvement, and platform scalability.

---

## Important reading

Before reviewing the branch files, please read:

**High Importance_Amperage Calculation.pdf**

This document contains a detailed propulsion-current analysis, including:
- measured versus theoretical current-draw comparison
- RPM-to-current cubic relationship
- scaling effects of motor KV, voltage, prop diameter, pitch, and blade count
- comparative discussion of three-blade and two-blade configurations
- technical appendix material prepared in academic report format

This file is important because it explains the electrical and aerodynamic reasoning behind the propulsion choices and benchmark calculations used in this repository.

---

## Project overview

The aim of this project is to develop a practical medical-delivery drone prototype capable of:

- GPS-based mission travel
- dead reckoning mission travel
- payload release
- mission monitoring through a GUI
- return-to-start or mission completion logic

The software is built around a **Pixhawk + Raspberry Pi architecture**:

- **Pixhawk** handles flight stabilisation and core flight-controller behaviour
- **Raspberry Pi** handles mission logic, telemetry display, GUI interaction, payload control, and software-side mission flow

---

## Repository branch structure

This repository is organised into multiple branches for different prototype stages.

### `main`
Main documentation and shared project material.

Recommended contents:
- project overview
- README
- final documentation
- technical appendices
- architecture diagrams
- important PDF references

### `Prototype_1`
### `Prototype_2`

Simplest pilot-supervised mission version.

Mission style:
- Manual takeoff
- GUI-triggered mission after stable takeoff
- Manual landing


Fully automated Raspberry Pi mission version.

Mission style:
- GUI-triggered mission
- Automatic takeoff
- Automatic mission execution
- Automatic landing

### `hexacopter_prototype_1`
Hexacopter-oriented prototype branch for larger payload and scaling study.

Main idea:
- platform-scaling study
- heavier payload direction
- propulsion and structural comparison against quadrotor implementation

---

## Quick comparison

| Branch | Platform idea | Takeoff | Mission start | Mission travel | Landing |
|--------|---------------|---------|----------------|----------------|---------|
| `Prototype_1` | Simplest operational version | Manual | Started manually in GUI after takeoff | Automated | Manual |
| `Prototype_2` | Highest automation version | Automatic | Started in GUI | Fully automatic | Automatic |
| `hexacopter_prototype_1` | Scaled heavy-payload direction | Depends on implementation stage | Depends on implementation stage | Depends on implementation stage | Depends on implementation stage |

---

## Main code features

Depending on the version, the system supports:

- live Pixhawk telemetry
- Raspberry Pi GUI
- GPS waypoint travel
- dead reckoning mission mode
- payload release
- recording / mission evidence
- return-to-start logic
- transmitter override switching
- altitude-gated mission start in the hybrid version

---

## Important setup note — change the Pixhawk serial ID first

Before running the program, you **must edit the `PORT` variable in the Python file** to match your own Pixhawk serial device.

Example:

```python
PORT = "/dev/serial/by-id/usb-Holybro_Pixhawk6C_29004B001051333235363832-if00"
