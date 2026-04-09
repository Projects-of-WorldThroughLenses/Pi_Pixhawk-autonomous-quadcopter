# Medical Delivery Drone Project

This repository contains the mission-control software developed for my final year project:

**Medical Drone for Critical Supply Transport**

The project explores how a Raspberry Pi and Pixhawk-based drone can be used for medical-payload delivery through different levels of mission automation.

Rather than using only one control philosophy, this repository includes **three software variants**, each representing a different level of operator involvement during takeoff, mission execution, and landing.

---

## Project overview

The purpose of this project is to develop a medical-delivery drone prototype capable of performing guided mission tasks such as:

- GPS-based mission travel
- dead reckoning mission travel
- payload release
- mission monitoring through GUI
- return-to-start or mission completion logic

The project is built around a **Pixhawk + Raspberry Pi architecture**, where Pixhawk handles flight-controller functions and Raspberry Pi handles higher-level mission logic and interface behaviour.

---

## Software versions

This repository contains three main versions of the mission software.

### Version 1 — Simplest Version
This is the most basic and easiest-to-operate version.

#### Mission style
- **Manual takeoff**
- **Manual landing**
- After takeoff, the operator enables the mission through the GUI
- The GUI then handles the selected mission logic

#### Main idea
This version keeps the pilot responsible for the most critical flight phases while still allowing mission automation after stable flight is achieved.

#### Best use
- early testing
- safer operator-supervised missions
- validation of mission logic with pilot involvement

---

### Version 2 — Fully Automated Raspberry Pi Version
This version is the most automated version in the repository.

#### Mission style
- Mission started from the GUI
- **Automatic takeoff**
- **Automatic mission execution**
- **Automatic landing**
- Raspberry Pi carries out the full mission flow

#### Main idea
This version reduces the need for continuous operator involvement and explores a more autonomous delivery workflow.

#### Best use
- research into higher automation
- comparison against manual-assisted mission modes
- full mission sequencing experiments

#### Note
This version is more advanced and should be treated as the most automation-heavy implementation.

---

### Version 3 — Altitude-Gated Mission Start Version
This is the most versatile and practical version for mixed human/autonomous operation.

#### Mission style
- Mission is prepared first in the GUI
- **Manual takeoff**
- The mission does **not** start immediately
- The drone climbs manually
- Once the drone reaches **4 metres altitude**, the planned mission begins automatically
- **Manual landing**

#### Main idea
This version combines pilot-supervised takeoff and landing with automatic mission execution only after a safe launch altitude is reached.

#### Best use
- practical field workflow
- safer mission start sequencing
- balancing human supervision with mission automation

---

## Comparison of the three versions

| Version | Takeoff | Mission Start | Mission Execution | Landing |
|--------|--------|----------------|------------------|---------|
| Version 1 | Manual | GUI toggle after takeoff | Automated by mission logic | Manual |
| Version 2 | Automatic | GUI toggle | Fully automatic | Automatic |
| Version 3 | Manual | Automatically begins after reaching 4 m altitude | Automated by mission logic | Manual |

---

## Why these three versions were developed

These three versions were created to explore different levels of automation and mission practicality.

- **Version 1** focuses on simplicity and operator control
- **Version 2** explores full automation through Raspberry Pi mission sequencing
- **Version 3** represents a hybrid approach that is safer and more practical for real mission preparation

This progression also reflects the engineering development of the project, where control authority, safety, feasibility, and usability were gradually refined.

---

## Core hardware platform

The software in this repository was developed around the following hardware architecture:

- **Pixhawk 6C**
- **Raspberry Pi 5**
- **GPS**
- **Telemetry module**
- **Payload release servo**
- **Pi Camera**
- **Onboard display**
- **Medical delivery drone frame and propulsion system**

---

## Main engineering idea

The key system idea is to combine:

- **stable low-level flight control** from the flight controller
- **higher-level mission logic** from the Raspberry Pi
- **different levels of operator involvement** depending on the software version

This makes the repository useful not only as a final implementation archive, but also as a record of design evolution and engineering trade-off decisions.

---

## Intended use

This repository is intended for:

- final year project documentation
- software version archiving
- engineering comparison between mission-control strategies
- prototype validation and demonstration

---

## Safety note

This repository is for academic and prototype development purposes.

Any real drone testing must be carried out only under:

- controlled conditions
- proper supervision
- safe bench-testing procedures
- appropriate regulatory compliance

---

## Author

**Phone Myat Thu**  
Final Year Project  
**Medical Drone for Critical Supply Transport**
