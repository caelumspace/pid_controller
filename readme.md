# PID Controller for Autonomous Driving

This repository contains a simple C++ PID (Proportional-Integral-Derivative) controller implementation tailored for autonomous driving scenarios. The example in `main()` demonstrates lane-keeping behavior in a simplified simulation, adjusting a "steering" value to keep the vehicle at the lane center (or any defined setpoint).

> **Note**: This example is meant as a starting point. For a real autonomous vehicle, you must integrate additional sensors, apply noise filtering, implement safety checks, handle actuator limits, and tune the PID gains rigorously in real-world conditions.

---

## Table of Contents

1. [Features](#features)
2. [Project Structure](#project-structure)
3. [Dependencies](#dependencies)
4. [Building](#building)
5. [Usage](#usage)
6. [PID Tuning](#pid-tuning)
7. [Future Enhancements](#future-enhancements)
8. [License](#license)
9. [Contact](#contact)

---

## Features

- **Proportional-Integral-Derivative Control**: Classic, straightforward implementation.
- **Runtime Tuning**: Change PID gains (`Kp`, `Ki`, `Kd`) and time step (`dt`) dynamically via setter methods.
- **Basic Simulation**: Illustrates how the control loop might adjust a vehicle’s lateral position to reduce error over multiple time steps.

---

## Project Structure

less

CopyEdit

`.
├── CMakeLists.txt // (If using CMake, optional)
├── main.cpp // Entry point with PID demonstration
└── README.md // This file`

---

## Dependencies

- A C++ compiler supporting C++11 or higher (e.g., `g++`, `clang`).
- [CMake (optional)](https://cmake.org/) if you prefer building via CMake.

---

## Building

### Using a Traditional Compiler

Compile using `g++` or `clang++`:

bash

CopyEdit

`g++ main.cpp -o pid_controller`

### Using CMake (Optional)

1. Create a new directory for your build:
  
  bash
  
  CopyEdit
  
  `mkdir build && cd build`
  
2. Configure and build:
  
  bash
  
  CopyEdit
  
  `cmake ..
  make`
  
3. An executable named `pid_controller` (or similar) will appear in the build directory.
  

---

## Usage

1. **Run the Program**:
  
  bash
  
  CopyEdit
  
  `./pid_controller`
  
2. **Observe Output**:
  
  - Each iteration prints:
    - Current step
    - Error between setpoint and measured position
    - Computed control output
    - Updated measured position (simulated result)
3. **Tune Gains**:
  
  - Edit the initial `kp`, `ki`, `kd` values in `main.cpp` and recompile, or
  - Add logic to adjust gains at runtime (e.g., from user input or a configuration file).

### Example Output

yaml

CopyEdit

`Step: 0 | Error: -1 | Control Output: -0.51 | Measured Position: 0.949
Step: 1 | Error: -0.949 | Control Output: -0.483 | Measured Position: 0.900
...`

---

## PID Tuning

- **Proportional (Kp)**: Influences the immediate response to the error. A higher Kp reacts more aggressively to current error but may overshoot.
- **Integral (Ki)**: Accumulates and corrects steady-state error. Excessive Ki can lead to overshoot and integral wind-up.
- **Derivative (Kd)**: Predicts future error trends, stabilizing oscillations. High Kd may amplify sensor noise.

### Tips

1. Start with **Kp**: Increase until you observe oscillations, then back off slightly.
2. Add **Kd**: Smooth out oscillations and improve settling.
3. Increase **Ki**: Address any remaining steady-state error (persistent offset).

---

## Future Enhancements

1. **Adaptive Gains**: Adjust PID gains dynamically based on speed, road conditions, or sensor confidence.
2. **Anti-Windup**: Limit the integral term to prevent excessive buildup when error remains large.
3. **Sensor Integration**: Incorporate real sensor data (camera, LiDAR, radar) and a state estimator (e.g., Kalman filter).
4. **Safety Checks**: Ensure actuator commands remain within physical limits.

---

## License

This project is provided under the MIT License. You are free to use, modify, and distribute this code in both commercial and non-commercial projects.

---

