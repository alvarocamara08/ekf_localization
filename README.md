# 📍 Extended Kalman Filter — Vehicle Localization

![Python](https://img.shields.io/badge/Python-3.8%2B-blue?logo=python&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green)
![Status](https://img.shields.io/badge/Status-In%20Progress-yellow)

## 📖 Overview

A **from-scratch** implementation of the **Extended Kalman Filter (EKF)** 
for vehicle localization, applied to a kinematic bicycle model.

The filter fuses **odometry** (motion model prediction) with simulated 
**GPS measurements** (observation update) to estimate the vehicle's pose 
`(x, y, θ)` in 2D.

> **No frameworks. No ROS. Pure Python + Math.**

## 🚧 Work in Progress

This project is currently under development.

## 🗂️ Project Structure

    ekf_localization/
    ├── models/
    │   └── bicycle_model.py       # Kinematic bicycle model
    ├── ekf/
    │   └── ekf.py                 # Extended Kalman Filter
    ├── sensors/
    │   └── sensor_models.py       # Simulated GPS and odometry
    ├── trajectory/
    │   └── path_generator.py      # Reference path generation
    ├── visualization/
    │   └── plotter.py             # Results visualization
    ├── tests/                     # Unit tests
    ├── docs/
    │   └── theory.md              # Mathematical derivation
    ├── main.py                    # Entry point
    ├── config.py                  # All tunable parameters
    └── requirements.txt
    └── LICENSE

## 📄 License

This project is licensed under the MIT License — see [LICENSE](LICENSE) for details.