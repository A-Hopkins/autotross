# UAV State Estimation using EKF

This repository models and simulates a state estimation system for a fixed-wing UAV using an Extended Kalman Filter (EKF). It provides a realistic sensor simulation pipeline, multiple test scenarios (e.g., straight and level flight, climbing, circling), and tools for parameter tuning and accuracy evaluation.

---

## Overview

The simulation tracks the UAV's pose, velocity, acceleration, and orientation using a 16-element state vector. The EKF fuses synthetic measurements from simulated GPS, IMU, barometer, and airspeed sensors. All sensors include tunable noise models.

**Key Components:**

* `ekf.py`: A general-purpose Extended Kalman Filter class
* `simulate_sensors.py`: Sensor simulator for generating time-series data with truth vs noisy readings
* `uav_model.py`: The entry point for running scenarios and evaluating EKF performance
* `parameter_tuning.py`: Batch scenario runner for tuning filter parameters

---

## How to Run

```bash
python uav_model.py
```

Each defined scenario will be simulated, run through the EKF, and evaluated using error metrics. Plots for position, velocity, and orientation are shown per scenario.

---

## State Vector Design

The EKF tracks the following 16-dimensional state vector:

```text
[x, y, z, vx, vy, vz, ax, ay, az, qx, qy, qz, qw, wx, wy, wz]
```

* **Position** (x, y, z): In meters, East-North-Up (ENU) frame
* **Velocity** (vx, vy, vz): m/s in ENU
* **Acceleration** (ax, ay, az): m/s^2 in ENU
* **Orientation** (qx, qy, qz, qw): Quaternion representing body-to-ENU rotation
* **Angular velocity** (wx, wy, wz): rad/s in body frame

### Why Each Element Matters

| Element          | Reason for Inclusion                                          |
| ---------------- | ------------------------------------------------------------- |
| Position         | Primary navigation target (where is the UAV?)                 |
| Velocity         | Needed to predict next position and airspeed                  |
| Acceleration     | Allows IMU modeling; smoother transitions                     |
| Orientation      | Required to convert between body and world frames             |
| Angular velocity | Needed to integrate orientation and compare to gyroscope data |

---

## Dynamics Model and State Transition

The dynamics model assumes:

* Constant acceleration model for translational motion
* Quaternion integration using angular velocity for rotational dynamics

### State Transition Function f(x)

Given a timestep `dt`:

```text
pos_next = pos + vel * dt + 0.5 * acc * dt^2
vel_next = vel + acc * dt
acc_next = acc  (assumed constant over small dt)
quat_next = quat * delta_quat(omega * dt)
omega_next = omega  (assumed constant over small dt)
```

Quaternion propagation uses the exponential map:

```text
delta_quat = axis-angle -> quaternion from omega * dt
```

The Jacobian F(x) reflects these relationships and is used in the EKF prediction step.

---

## Sensor Models

Sensors are simulated with configurable Gaussian noise. All simulate both noisy and ground-truth data.

### Simulated Sensors

| Sensor    | Measured Fields                      | Modeled As                   |
| --------- | ------------------------------------ | ---------------------------- |
| GPS       | Position (lat, lon, alt), velocity   | Direct ENU + velocity        |
| Barometer | Altitude                             | Noisy z-coordinate           |
| Airspeed  | True airspeed                        | norm of velocity             |
| IMU       | Orientation, angular velocity, accel | Body-frame, includes gravity |

Each sensor update is modeled as a linear or nonlinear function of the state with its own measurement model and covariance.

---

## Sensor Noise Settings

These are based on realistic specs for small fixed-wing UAVs:

| Sensor          | Std Dev     | Justification                          |
| --------------- | ----------- | -------------------------------------- |
| GPS Position    | 1.5 m       | Consumer-grade GNSS                    |
| GPS Velocity    | 0.05 m/s    | Filtered from GNSS Doppler             |
| IMU Accel       | 0.2 m/s^2   | Mid-grade IMU under moderate vibration |
| IMU Gyro        | 0.005 rad/s | High-grade MEMS                        |
| IMU Orientation | 0.005       | From full 9-DoF fusion                 |
| Barometer       | 0.5 m       | Filtered barometric altimeter          |
| Airspeed (DP)   | 1.0 Pa      | Pitot-tube sensor accuracy             |

### Sensor Measurement Models

| Sensor         | Measurement Function h(x) | Jacobian H(x)            | Noise Covariance R |
| -------------- | ------------------------- | ------------------------ | ------------------ |
| GPS Position   | x\[0:3]                   | 3x16 (identity on 0–2)   | sigma^2 \* I3      |
| GPS Velocity   | x\[3:6]                   | 3x16 (identity on 3–5)   | sigma^2 \* I3      |
| IMU Gyro       | x\[13:16]                 | 3x16 (identity on 13–15) | sigma^2 \* I3      |
| IMU Quaternion | x\[9:13]                  | 4x16 (identity on 9–12)  | sigma^2 \* I4      |
| IMU Accel      | R\_b^e \* (x\[6:9] + g)   | Approximated as 0        | sigma^2 \* I3      |
| Barometer      | x\[2] (altitude)          | 1x16 (1 at index 2)      | sigma^2            |
| Airspeed       | norm(x\[3:6])             | v / norm(v) in 3x16      | sigma^2            |

---

## Defining a New Scenario

To define a custom scenario, construct a config dictionary with the following keys:

```python
my_scenario = {
    "name": "Spiral Ascent",
    "start_orientation": np.array([0, 0, 0, 1]),
    "end_orientation": R.from_euler('xyz', [0, 30, 90], degrees=True).as_quat(),
    "start_velocity": np.array([5.0, 0.0, 0.0]),
    "end_velocity": np.array([0.0, 5.0, 5.0]),
    "start_position": np.array([0.0, 0.0, 100.0]),
    "end_position": np.array([500.0, 500.0, 200.0]),
    "duration_sec": 60.0,
    "speed": 10.0,
    "waypoints": simulate_sensors.generate_circle_waypoints(radius=200, height=150, num_points=12)
}
```

Then in `uav_model.py`:

```python
run_scenario(my_scenario, name="Spiral Ascent")
```

---

## Tuning Parameters

You can adjust filter tuning per scenario by adding a nested `tuning_params` field in the config:

```python
"tuning_params": {
    "pos_uncertainty": 25.0,
    "vel_uncertainty": 2.0,
    "acc_uncertainty": 1.0,
    "quat_uncertainty": 0.01,
    "omega_uncertainty": 0.05,
    "process_noise_pos": 0.0001,
    "process_noise_vel": 0.2,
    "process_noise_acc": 0.05,
    "process_noise_quat": 0.001,
    "process_noise_omega": 0.1
}
```

---

## Evaluation Metrics

Each run computes:

* RMSE of position, velocity, and orientation
* Normalized Error Squared (NES): Should converge near 1.0 if tuned properly
* Maximum error: Sanity check for outliers

Orientation RMSE is computed via angle error between estimated and true quaternions.

---

## Plot Outputs

Plots include:

* Position vs GPS vs EKF
* Velocity vs EKF
* Orientation (roll, pitch, yaw)
* Error norms for position and velocity

---
