# Autonomous Line Following Robot (Arduino-Based)
**🥇 1st Place Winner - College Robotics Competition**

## Overview
This project presents the design and implementation of an autonomous line-following robot using Arduino.

The robot detects a black line using IR sensors and adjusts motor speed to maintain stable tracking along the path.

## System Architecture
- Arduino uno
- IR line sensors
- Motor driver module
- DC motors

## Control Logic
The robot reads sensor input and adjusts motor output accordingly.
Control logic was implemented to ensure stable tracking and smooth turning behavior.

## The PID Algorithm
# Proportional ($K_P$): The "immediate response"
This is the main driving force, if the robot is to far to the left ($K_P$) creates a strong turn to the right.
# Integral ($K_I$): The "error accumulator"
This is the precision, by correcting small, steady-state error that accumulate over time.
# Derivative ($K_D$): The "predictor"
This is the stability, it predicts the robot's motion and acts as brakes to slow down the turn.

## My Contribution
- Implemented control logic
- Integrated sensors and motor driver
- Performed testing and calibration
- Validated system behavior

## Tools & Technologies
- Arduino (C/C++)
- Embedded systems
- Hardware

## Results
The robot successfully followed predefined tracks with stable motion and minimal deviation.

🏆 Achievement: This project won 1st Place in the College's Autonomous Robotics Competition, demonstrating superior tracking speed and stability compared to other entries.

![IMG_6542](https://github.com/user-attachments/assets/3df8951d-d1d1-49b5-9404-2fd5b462f8b2)

### 🎥 Demonstration Videos
**Stable Line Tracking:**

https://github.com/user-attachments/assets/96a946f8-fc03-4560-b729-121cba80b216

**Test Competition:**

https://github.com/user-attachments/assets/bef828d1-f422-4e71-877c-ca6f114cd4c8

---
**Competition** - **Institution:** Ruppin Academic Center
