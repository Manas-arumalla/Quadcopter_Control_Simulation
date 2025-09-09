# Quadcopter Control & Simulation (MATLAB)

## Overview
This repository implements a MATLAB-based simulation framework for modeling, controlling, and testing a quadcopter (X-configuration).  

It provides **full end-to-end simulation** of quadcopter dynamics, actuator effects, and multiple control strategies, with special emphasis on **LQR + PID hybrid control, integrator anti-windup, and maneuver robustness**.  

The project is designed for:
- Studying quadcopter flight dynamics
- Testing controller designs (PID, LQR, hybrid)
- Experimenting with reference smoothing, integrator freeze, and actuator filtering
- Educational use and controller prototyping

---

## Features
- Full **6-DOF rigid body dynamics** using Newton–Euler equations.
- Multiple **controller architectures**:
  - LQR attitude inner loop + PID position outer loop
  - Tuned PID attitude controllers
- **Integrator handling**:
  - Anti-windup
  - Integrator freeze during aggressive maneuvers
  - Integrator damping after saturation
- **Reference shaping**:
  - Slew-rate limiting
  - Yaw reference smoothing
- **Rotor dynamics**:
  - Actuator saturation limits
  - Low-pass filters for motor commands
- Ready-to-run **simulation scripts** with plotting and error metrics.

---

---

## How to Run
1. Open MATLAB and set this repository as the working directory.
2. Run `quad_params.m` to load parameters.
3. Choose one of the main simulation scripts:
   - `sim_lqr_tuned_perfected.m` → LQR+PID tuned scenario
   - `sim_main_maneuver_freeze_best.m` → Aggressive maneuver test
   - `quadcopter_minimal_v7.m` → Simple baseline test
4. View the automatically generated plots (3D trajectory, position error, rotor commands).

---

## Outputs
- **Trajectory plots** (3D + XY ground track)
- **Error metrics** (e.g., `Final position error (norm): 0.0017 m`)
- **Time-series** of position, attitude, angular rates, and rotor forces
- **Diagnostics** showing integrator freeze activity, saturation events, and smooth reference tracking

---

## Control System Design
- **Inner loop**: LQR or PID regulating roll, pitch, yaw, and angular rates
- **Outer loop**: PID position control, producing desired roll/pitch/yaw commands
- **Reference shaping**: Prevents sharp steps using slew-limiters & smoothing
- **Actuator mapping**: Converts control outputs to rotor thrusts/torques
- **Anti-windup**: Prevents integrators from accumulating error when actuators saturate

---

## Tuning Notes
- Reduce oscillations → lower attitude loop gains or tune LQR weights
- Remove drift → adjust integrator gains/limits
- Avoid actuator saturation → strengthen slew-rate limits & anti-windup
- Start with `quadcopter_minimal_v7.m` before testing advanced scenarios

---

## Limitations
- Simplified aerodynamic model (constant thrust/moment coefficients)
- No wind/gust disturbances modeled
- Ideal sensors (no noise, no estimation filters)
- Script-based framework (not Simulink, though could be extended)

---

## Future Work
- Add wind/gust disturbance models
- Implement sensor noise + estimator (EKF/UKF)
- Port to Simulink / Simscape
- Extend with hardware-in-the-loop or ROS2 bridge

---

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

##  Contribution
Feel free to fork, open issues, or submit pull requests for:
- Controller improvements
- Additional flight scenarios
- More realistic actuator/sensor models

