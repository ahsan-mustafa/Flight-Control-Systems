# Flight Control System Design & Autopilots for McDonnell Douglas F-4 Phantom II

This repository contains MATLAB/Simulink models for the **design and simulation of Stability Augmentation Systems (SAS)** and **autopilot modes** for the McDonnell Douglas **F-4 Phantom II** aircraft.

The project demonstrates how longitudinal and lateral-directional dynamics can be controlled using classical control techniques, with inner-loop SAS and outer-loop autopilots.

---

## ✈️ Project Summary

- Designed **longitudinal and lateral stability augmentation systems (SAS)** using MATLAB and Simulink  
- Developed and tested multiple **autopilot modes** for the F-4 aircraft  
- Built a modular structure where **SAS serves as the inner loop** and autopilots act as outer loops  

---

## 🧭 Autopilot Modes Implemented

### Longitudinal Autopilots
- **Pitch Hold** – maintains a commanded pitch angle  
- **Height / Altitude Hold** – maintains a commanded altitude using pitch and/or throttle loops  

### Lateral-Directional Autopilots
- **Coordinated Turn Hold** – maintains coordinated turns with limited sideslip  
- **Heading Hold** – tracks a commanded heading  
- **Bank Hold** – maintains a commanded roll angle  

---

## 🛠 Stability Augmentation Systems (SAS)

### Longitudinal SAS
- Pitch-rate feedback
- Short-period damping improvement
- Enhanced longitudinal stability and handling qualities

### Lateral-Directional SAS
- Roll-rate damping
- Yaw-rate feedback
- Dutch-roll stabilization

These SAS loops are implemented as **inner control loops**. The autopilot modes are built on top of these stabilized dynamics.
