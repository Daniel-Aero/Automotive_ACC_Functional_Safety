# Automotive ACC with Functional Safety

This repository contains a MATLAB-based simulation of an **Automotive Adaptive Cruise Control (ACC)** system with basic **functional safety** concepts inspired by ISO 26262.

The ego vehicle:
- Tracks a desired **cruise speed** in free-flow conditions
- Follows a **lead vehicle** while keeping a safe time-gap
- Reacts to **lead-vehicle emergency braking**
- Enters a **low-speed safe-state** when a radar sensor fault is detected

---

## Features

- Longitudinal point-mass vehicle model (ego + lead)
- P-based speed control and distance control
- Time-headway safety policy:  
  `d_des = d0 + Th * v_ego`
- Mode switching between:
  - Speed control (cruise mode)
  - Distance control (following mode)
- Emergency braking scenario for the lead vehicle
- Simple safe-state logic under sensor failure
- Ready to extend toward ISO 26262-style analysis

---

## Files

- `src/acc_basic.m`  
  Basic cruise control (speed tracking only)

- `src/acc_with_lead.m`  
  ACC with constant-speed lead vehicle

- `src/acc_with_lead_emergency_safety.m`  
  ACC with lead emergency braking **and** radar sensor safe-state

- `docs/FMEA_ACC.pdf`  
  Simple FMEA documenting hazards and mitigations

- `plots/*.png`  
  Example figures for speed, distance, and acceleration profiles

---

## Functional Safety Considerations

This project is **not** a full ISO 26262 implementation, but it includes several safety-oriented ideas:

- **Emergency braking test case** for the lead vehicle  
- **Safe-state fallback** when the radar sensor is assumed to fail  
  - The controller forces a low-speed mode (e.g. 5 m/s) to reduce collision risk  
- **Basic FMEA** describing:
  - Failure modes (sensor loss, brake performance, etc.)
  - Potential impact and simple mitigation strategies

These elements show an understanding of safety-aware control design for both **automotive** and **defense mobility** applications.

---

## How to Run

1. Open MATLAB or GNU Octave  
2. Add the `src` folder to your path  
3. Run:

```matlab
acc_with_lead_emergency_safety
