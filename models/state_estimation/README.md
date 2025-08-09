
# UAV State Estimation using an Extended Kalman Filter

This repository provides a simulation and modeling environment for a fixed-wing UAV's state estimation system. The core of the system is a 13-dimensional Extended Kalman Filter (EKF) that implements a robust, IMU-driven kinematic model. This architecture allows for high-frequency state prediction and is capable of dead reckoning during short GPS outages.

The project includes a realistic sensor simulation pipeline, multiple flight scenarios for testing (e.g., straight flight, circles, climbs), and tools for evaluating filter accuracy and optimizing tuning parameters.

---
## Key Components

* `uav_model.py`: The main entry point for running scenarios, defining the EKF models, and evaluating performance.
* `ekf.py`: A general-purpose Extended Kalman Filter class.
* `simulate_sensors.py`: A comprehensive sensor simulator that generates realistic flight data, including both ground truth and noisy measurements.
* `parameter_tuning_opt.py`: A script using Bayesian Optimization (`scikit-optimize`) to automatically find the best EKF tuning parameters.

---

## How to Run a Simulation

To run all predefined flight scenarios and see their corresponding plots and error metrics, execute the main model file:

```bash
python uav_model.py
````

-----

## State Vector Design

The EKF tracks a 13-dimensional state vector, chosen to describe the UAV's motion without including redundant states.

`[x, y, z, vx, vy, vz, qx, qy, qz, qw, wx, wy, wz]`

| Element | Description | Reason for Inclusion |
| :--- | :--- | :--- |
| **Position** `[0:3]` | `x, y, z` (meters) | Primary navigation state; location in the ENU frame. |
| **Velocity** `[3:6]` | `vx, vy, vz` (m/s) | Required to predict future position and model airspeed. |
| **Orientation** `[6:10]`| `qx, qy, qz, qw` (quaternion) | Critical for rotating vectors (like acceleration) between the UAV's body frame and the world (ENU) frame. |
| **Angular Velocity** `[10:13]`| `wx, wy, wz` (rad/s) | Used to propagate the orientation forward in time. This is what the gyroscope measures. |


-----

## System Architecture: An IMU-Driven Kinematic Model

This EKF uses a robust and modern architecture that leverages the high frequency of IMU data for prediction and corrects with absolute sensors. The core philosophy is **"Predict with IMU, Correct with others."**

### Prediction Step:

The filter's `predict` step is driven by the 6-DOF IMU measurements (`ax, ay, az, wx, wy, wz`) which are treated as a **control input vector `u`**.

The state transition function `f(x, u)` performs the following physics calculations:

1.  **Propagate Orientation:** The new orientation is calculated by integrating the measured angular velocity (`omega_body`) from the gyroscope.
2.  **Rotate Acceleration:** The measured linear acceleration (`accel_body`), which is in the drone's body frame, is rotated into the global ENU frame using the current orientation quaternion.
3.  **Calculate Inertial Acceleration:** Gravity is added back to the rotated vector to get the true inertial acceleration in the ENU frame.
4.  **Update Velocity & Position:** The new velocity and position are found by integrating this inertial acceleration forward in time.

### Update Step:

The `update` step uses measurements from slower, absolute sensors to correct the drift that inevitably accumulates during the prediction step.

  * **GPS (Position & Velocity):** Provides the primary, non-drifting anchor for the UAV's position and velocity.
  * **Fused IMU Quaternion:** Provides the primary anchor for the UAV's **attitude**. This corrects for the drift from integrating the gyroscope.
  * **Barometer:** Provides an additional source of altitude information.
  * **Airspeed Sensor:** Provides a measurement of the velocity magnitude, helping to constrain the velocity estimate.

This "predict-correct" cycle allows the filter to produce a smooth, high-rate state estimate that remains locked to reality while being able to "coast" through short GPS outages using the IMU-driven dead reckoning.

-----

## Sensor Fusion Logic

The filter fuses data from multiple sensors, each with a specific role:

| Sensor | Role in EKF |
| :--- | :--- |
| **IMU (Accel + Gyro)** | **Control Input.** Drives the high-frequency `predict` step. Not used for updates. |
| **GPS (Position/Velocity)** | **Primary Corrector.** Provides absolute position and velocity updates to eliminate drift. |
| **IMU Fused Quaternion** | **Attitude Corrector.** Provides an absolute orientation update to correct for gyro drift. |
| **Barometer** | **Altitude Corrector.** Provides a supplementary, independent measurement of altitude (`z` position). |
| **Airspeed Sensor** | **Velocity Corrector.** Provides a measurement of the velocity vector's magnitude. |


-----

## Tuning the Filter

Filter performance is critically dependent on two sets of parameters defined in `uav_model.py`. The recommended way to find optimal values is by running the `parameter_tuning_opt.py` script.

#### 1\. Initial State Uncertainty (`P`)

These values set the initial covariance matrix, telling the filter how certain it is about its starting state.

  * `pos_uncertainty`
  * `vel_uncertainty`
  * `quat_uncertainty`
  * `omega_uncertainty`

#### 2\. Process Noise (`Q`)

This models the uncertainty in our dynamics model, accounting for unmodeled forces like wind and turbulence. It tells the filter how much to trust its own prediction versus the incoming sensor measurements.

  * `process_noise_pos`
  * `process_noise_vel`
  * `process_noise_quat`
  * `process_noise_omega`

An example of a tuned parameter set:

```python
"tuning_params": {
    "pos_uncertainty": 25.0,
    "vel_uncertainty": 2.0,
    "quat_uncertainty": 0.01,
    "omega_uncertainty": 0.05,
    "process_noise_pos": 0.0001,
    "process_noise_vel": 0.2,
    "process_noise_quat": 0.001,
    "process_noise_omega": 0.1
}
```

-----

## Evaluation and Plotting

Each simulation run computes and prints key performance metrics, including:

  * **RMSE:** Root Mean Squared Error for position, velocity, and orientation.
  * **NES:** Normalized Error Squared, a statistical metric of filter consistency (a well-tuned filter has a NES near 1.0).
  * **Maximum Error:** The peak error observed during the simulation.

The script also generates detailed plots comparing the EKF estimate against the ground truth for position, velocity, and orientation, as well as plots of sensor residuals and error norms over time.
