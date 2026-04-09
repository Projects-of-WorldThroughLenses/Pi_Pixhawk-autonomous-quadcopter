# Medical Delivery Drone Project

This repository contains the mission-control software developed for my final year project:

**Medical Drone for Critical Supply Transport**

The project explores how a Raspberry Pi and Pixhawk-based drone can be used for medical payload delivery through different levels of automation and operator involvement.

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

## Software versions

This repository contains three main mission-control styles.

### Version 1 — Simplest Version
This is the simplest and most pilot-supervised version.

#### Mission flow
- **Manual takeoff**
- once airborne, the mission is started using the GUI
- mission travel is then handled by the software
- **Manual landing**

#### Main idea
The pilot remains responsible for the most critical phases of flight, while the software handles mission execution after the aircraft is already safely in the air.

---

### Version 2 — Fully Automated Raspberry Pi Version
This is the most automated version.

#### Mission flow
- mission is started from the GUI
- **Automatic takeoff**
- **Automatic mission execution**
- **Automatic landing**

#### Main idea
This version explores a more fully autonomous workflow where the Raspberry Pi handles the complete mission sequence.

#### Note
This version is more experimental because it reduces pilot involvement significantly.

---

### Version 3 — Altitude-Gated Mission Start Version
This is the most versatile hybrid version.

#### Mission flow
- mission is prepared first in the GUI
- **Manual takeoff**
- the drone climbs under pilot control
- once the drone reaches **4 metres altitude**, the mission begins automatically
- mission execution is handled by the software
- **Manual landing**

#### Main idea
This version combines safer manual takeoff and landing with automatic mission travel after a valid launch altitude has been reached.

---

## Quick comparison

| Version | Takeoff | Mission Start | Mission Travel | Landing |
|--------|--------|----------------|----------------|---------|
| Version 1 | Manual | Started manually in GUI after takeoff | Automated | Manual |
| Version 2 | Automatic | Started in GUI | Fully automatic | Automatic |
| Version 3 | Manual | Starts automatically after reaching 4 m | Automated | Manual |

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
- altitude-gated mission start in the advanced hybrid version

---

## Important setup note — change the Pixhawk serial ID first

Before running the program, you **must edit the `PORT` variable in the Python file** to match your own Pixhawk serial device.

In the code, it currently looks like this:

```python
PORT = "/dev/serial/by-id/usb-Holybro_Pixhawk6C_29004B001051333235363832-if00"
