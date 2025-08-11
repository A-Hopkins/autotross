"""
UAV State Estimation using an Extended Kalman Filter (EKF).

This script serves as the primary model and simulation environment for a UAV
state estimator. It defines a 10-dimensional EKF that fuses measurements from
a simulated sensor suite (GPS, IMU, Barometer, Airspeed) to produce a robust
estimate of the UAV's state.

The core of the script is the `run_scenario` function, which:
1.  Generates a flight trajectory and corresponding "true" and "noisy" sensor
    data using the `simulate_sensors.py` module.
2.  Initializes an `ExtendedKalmanFilter` instance from `ekf.py`.
3.  Defines the system dynamics (state transition) and measurement models.
4.  Runs the EKF loop, performing prediction and update steps.
5.  Calculates and prints performance metrics (RMSE, NES).
6.  Generates plots comparing the estimated state against ground truth.

The state vector is defined as:
- [0:3]   Position (x, y, z) in meters, ENU (East-North-Up) frame.
- [3:6]   Velocity (vx, vy, vz) in m/s, ENU frame.
- [6:10]  Orientation (qx, qy, qz, qw) as a quaternion from ENU to body frame.

The dynamics model is IMU-driven, meaning the high-rate accelerometer and
gyroscope measurements are used as control inputs in the EKF's prediction step
to propagate the state forward in time. Other sensors (GPS, Barometer, etc.)
are used in the update step to correct the predicted state.

To run the simulation, execute this file directly:
    $ python uav_model.py

This will run a series of predefined scenarios and display the results.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import simulate_sensors
from ekf import ExtendedKalmanFilter

# --- Configuration ---
# Gravity vector in the ENU (East-North-Up) frame.
G_ENU = np.array([0., 0., -9.80665])

# STATE VECTOR:
# - Position (x,y,z): meters in ENU frame
# - Velocity (vx,vy,vz): meters/sec in ENU frame
# - Orientation (qx,qy,qz,qw): quaternion representing body rotation relative to ENU frame
state_dim = 10  # [x y z vx vy vz qx qy qz qw]

def run_scenario(scenario_config, name="", plot=True):
  """
  Runs a complete EKF simulation for a given flight scenario.
  (Docstring remains the same)
  """
  # --- Scenario Parameters ---
  lat0, lon0, alt0 = 37.7749, -122.4194, 0.0 # lat0, lon0, alt0 = -35.363262, 149.165237, 584.0
  scenario = simulate_sensors.Scenario(
    start_orientation=scenario_config['start_orientation'],
    end_orientation=scenario_config['end_orientation'],
    start_velocity=scenario_config['start_velocity'],
    end_velocity=scenario_config['end_velocity'],
    start_position=scenario_config['start_position'],
    end_position=scenario_config['end_position'],
    duration_sec=scenario_config['duration_sec'],
    rate_hz=10.0,
    start_latlon=(lat0, lon0),
    waypoints=scenario_config['waypoints'],
    speed=scenario_config['speed'],
    name=scenario_config['name']
  )
  
  sensor_data, truth_data = scenario.run()
  dt = scenario.dt

  # --- Initialize EKF ---
  # The control input vector `u` consists of 3D acceleration and 3D angular velocity from the IMU.
  control_dim = 6  # [ax, ay, az, wx, wy, wz]
  ekf = ExtendedKalmanFilter(state_dim, control_dim=control_dim)
  ekf.state = np.zeros((state_dim, 1))
  ekf.state[6:10] = np.array([0, 0, 0, 1]).reshape(-1, 1)  # Initialize quaternion to identity

  # --- EKF Tuning Parameters ---
  tuning = scenario_config.get('tuning_params', {})
  
  # 1. Initial State Uncertainty (P_0):
  pos_uncertainty   = tuning.get('pos_uncertainty', 2.531408)
  vel_uncertainty   = tuning.get('vel_uncertainty', 0.573733)
  quat_uncertainty  = tuning.get('quat_uncertainty', 0.01)

  # 2. Process Noise (Q):
  process_noise_pos   = tuning.get('process_noise_pos', 0.000336)
  process_noise_vel   = tuning.get('process_noise_vel', 0.054512)
  process_noise_quat  = tuning.get('process_noise_quat', 0.0005500)

  # Initialize 10x10 covariance matrix `P`
  ekf.covariance = np.zeros((state_dim, state_dim))
  ekf.covariance[0:3, 0:3]   = np.eye(3) * pos_uncertainty
  ekf.covariance[3:6, 3:6]   = np.eye(3) * vel_uncertainty
  ekf.covariance[6:10, 6:10] = np.eye(4) * quat_uncertainty

  # Initialize 10x10 process noise covariance matrix 'Q'
  ekf.process_noise = np.zeros((state_dim, state_dim))
  ekf.process_noise[0:3, 0:3]    = np.eye(3) * process_noise_pos
  ekf.process_noise[3:6, 3:6]    = np.eye(3) * process_noise_vel
  ekf.process_noise[6:10, 6:10]  = np.eye(4) * process_noise_quat

  # --- DYNAMICS MODEL (STATE TRANSITION) ---
  def f(x, u):
    """
    Defines the nonlinear state transition function `f(x, u)`.
    Angular velocity is taken exclusively from the control input `u`.
    """
    # 1. Deconstruct state and control vectors for clarity.
    x_flat = x.flatten()
    u_flat = u.flatten()

    pos        = x_flat[0:3]
    vel        = x_flat[3:6]
    quat       = x_flat[6:10]
    accel_body = u_flat[0:3]  # Acceleration from IMU in body frame
    omega_body = u_flat[3:6]  # Angular velocity from IMU in body frame

    # --- 2. ORIENTATION UPDATE ---
    omega_norm = np.linalg.norm(omega_body)
    if omega_norm > 1e-10:
        axis = omega_body / omega_norm
        angle = omega_norm * dt
        dq = R.from_rotvec(axis * angle).as_quat()
        quat_next = (R.from_quat(quat) * R.from_quat(dq)).as_quat()
    else:
        quat_next = quat
    
    quat_next /= np.linalg.norm(quat_next)

    # --- 3. VELOCITY UPDATE ---
    rot_b_e = R.from_quat(quat)
    accel_enu_measured = rot_b_e.apply(accel_body)
    inertial_accel_enu = accel_enu_measured + G_ENU
    vel_next = vel + inertial_accel_enu * dt

    # --- 4. POSITION UPDATE ---
    pos_next = pos + vel_next * dt + 0.5 * inertial_accel_enu * dt**2

    # --- 5. RECONSTRUCT NEXT STATE ---
    state_next_flat = np.concatenate([pos_next, vel_next, quat_next])
    return state_next_flat.reshape(-1, 1)

  def F_jacobian(x, u):
    """
    Computes the Jacobian of the state transition function, F.
    """
    F = np.eye(state_dim)
    F[0:3, 3:6] = np.eye(3) * dt
    return F

  # --- MEASUREMENT MODELS ---
  # Each sensor provides a measurement that is a function of the state vector.
  # For the EKF, we need two functions for each sensor type:
  # 1. Measurement Function `h(x)`: Predicts the sensor measurement given a
  #    state vector `x`.
  # 2. Measurement Jacobian `H(x)`: The partial derivative of `h(x)` with
  #    respect to the state `x`. This linearizes the measurement function
  #    around the current state estimate and is used to update the covariance.

  def h_gps(x):
    """Predicts a GPS position measurement from the state vector."""
    return x[0:3]
  def H_gps(x):
    """Jacobian of the GPS position measurement function."""
    H = np.zeros((3, state_dim))
    H[:, 0:3] = np.eye(3)
    return H

  def h_gps_vel(x):
    """Predicts a GPS velocity measurement from the state vector."""
    return x[3:6]
  def H_gps_vel(x):
    """Jacobian of the GPS velocity measurement function."""
    H = np.zeros((3, state_dim))
    H[:, 3:6] = np.eye(3)
    return H

  def h_baro(x):
    """Predicts a combined [altitude, vertical_velocity] measurementfrom the state vector."""
    return np.array([
        [x[2][0]], # z position
        [x[5][0]]  # vz velocity
    ])
  def H_baro(x):
    """Jacobian of the barometer measurement function."""
    H = np.zeros((2, state_dim))
    H[0, 2] = 1.0  # Partial derivative of z_meas w.r.t. z_state
    H[1, 5] = 1.0  # Partial derivative of vz_meas w.r.t. vz_state
    return H

  def h_airspeed(x):
    """Predicts a true airspeed measurement from the state vector.
    Airspeed is the magnitude of the velocity vector. This is a nonlinear
    function of the state.
    """
    return np.array([[np.linalg.norm(x[3:6])]])
  def H_airspeed(x):
    """Jacobian of the true airspeed measurement function.
    The partial derivative of the vector norm ||v|| with respect to the
    vector v is v / ||v||.
    """
    v = x[3:6].flatten()
    norm_v = np.linalg.norm(v)
    if norm_v < 1e-6:  # Avoid division by zero at near-zero speeds
       return np.zeros((1, state_dim))
    H = np.zeros((1, state_dim))
    H[0, 3:6] = v / norm_v
    return H

  def h_imu_quat(x):
    """Predicts an IMU orientation measurement from the state vector."""
    return x[6:10]
  def H_imu_quat(x):
    """Jacobian of the IMU orientation measurement function."""
    H = np.zeros((4, state_dim))
    H[:, 6:10] = np.eye(4)
    return H

  # --- MEASUREMENT NOISE COVARIANCE MATRICES (R) ---

  # The `R` matrix for each sensor defines the uncertainty (variance) of its
  # measurements. It's a crucial tuning parameter that tells the EKF how much
  # to trust a given sensor reading during the update step.
  #
  # A smaller `R` value means the sensor is considered more accurate, and the
  # EKF will adjust the state more aggressively to match its measurements.
  # A larger `R` value means the sensor is noisy, and the EKF will rely more
  # on its own prediction.
  #
  # These matrices are typically diagonal, assuming that the noise in each
  # axis of a sensor is independent. The diagonal elements are the variances
  # (i.e., standard_deviation^2) of the sensor noise.

  # R_gps: Covariance for the 3D GPS position measurement (East, North, Up).
  # The variance is the same for all three axes.
  R_gps   = np.eye(3) * scenario.gps_pos_noise**2

  # R_vel: Covariance for the 3D GPS velocity measurement (vx, vy, vz).
  R_vel   = np.eye(3) * scenario.gps_vel_noise**2

  # R_baro: Covariance for the 2D barometer altitude measurement (z).
  # This is a 2x2 matrix.
  R_baro  = np.diag([scenario.baro_noise**2, scenario.baro_noise**2])

  # R_dp: Covariance for the 1D airspeed measurement.
  # Note: The noise is defined on the differential pressure sensor, but here
  # we are using it for the true airspeed measurement directly. In a more
  # complex model, this would be handled differently.
  R_dp    = np.array([[scenario.dp_noise**2]])

  # R_quat: Covariance for the 4D IMU orientation (quaternion) measurement.
  # This assumes the IMU provides a direct, fused orientation estimate.
  R_quat  = np.eye(4) * scenario.orientation_noise**2 

  # --- Run Filter ---
  history = {k: [] for k in [
      'time', 'truth_pos', 'truth_vel', 'truth_quat', 'state',
      'est_pos', 'est_vel', 'est_quat', 'gps_pos', 'gps_vel'
  ]}

  imu_rate = 1000
  gps_rate = 16
  baro_rate = 10
  airspeed_rate = 25
  imu_next, gps_next, baro_next, airspeed_next = 0.0, 0.0, 0.0, 0.0

  for entry, truth in zip(sensor_data, truth_data):
    time = entry['time']

    # --- 1. PREDICT STEP ---
    accel = np.array(entry['imu']['linear_acceleration'])
    gyro = np.array(entry['imu']['angular_velocity'])
    latest_imu_measurement = np.vstack([accel.reshape(-1, 1), gyro.reshape(-1, 1)])
    ekf.predict(f, F_jacobian, control_input=latest_imu_measurement)

    # --- 2. UPDATE STEP ---
    gps_enu = simulate_sensors.geodetic_to_enu(*entry['gps']['position'], lat0, lon0, alt0)

    if time >= imu_next:
        ekf.update(np.array(entry['imu']['orientation']).reshape(4, 1), R_quat, h_imu_quat, H_imu_quat, label="IMU Quat")
        imu_next += 1.0 / imu_rate
        
    if time >= baro_next:
        z_meas = np.array([[entry['altimeter']['altitude']], [entry['altimeter']['vertical_velocity']]])
        ekf.update(z_meas, R_baro, h_baro, H_baro, label="Baro") 
        baro_next += 1.0 / baro_rate
        
    if time >= gps_next:
        ekf.update(gps_enu.reshape(3, 1), R_gps, h_gps, H_gps, label="GPS Pos")
        ekf.update(np.array(entry['gps']['velocity']).reshape(3, 1), R_vel, h_gps_vel, H_gps_vel, label="gps_vel")
        gps_next += 1.0 / gps_rate
        
    if time >= airspeed_next:
        ekf.update(np.array([[entry['airspeed']['true_airspeed']]]), R_dp, h_airspeed, H_airspeed, label="Airspeed")
        airspeed_next += 1.0 / airspeed_rate

    x = ekf.state.flatten()
    x[6:10] /= np.linalg.norm(x[6:10])

    history['time'].append(entry['time'])
    truth_enu = simulate_sensors.geodetic_to_enu(*truth['gps']['position'], lat0, lon0, alt0)
    history['truth_pos'].append(truth_enu)
    history['truth_vel'].append(truth['gps']['velocity'])
    history['truth_quat'].append(truth['imu']['orientation'])
    history['state'].append(ekf.state.copy())
    history['est_pos'].append(x[0:3])
    history['est_vel'].append(x[3:6])
    history['est_quat'].append(x[6:10])
    history['gps_pos'].append(gps_enu)
    history['gps_vel'].append(entry['gps']['velocity'])

  # --- Calculations, Metrics, and Plotting ---
  history = {k: np.array(v) for k, v in history.items()}
  pos_err = history['est_pos'] - history['truth_pos']
  vel_err = history['est_vel'] - history['truth_vel']

  def quat_error(q1, q2):
    q_diff = R.from_quat(q1).inv() * R.from_quat(q2)
    return np.abs(q_diff.magnitude() * 180/np.pi)
  
  quat_errors = np.array([quat_error(est, truth) 
                            for est, truth in zip(history['est_quat'], history['truth_quat'])])

  rmse_pos = np.sqrt(np.mean(pos_err**2, axis=0))
  rmse_vel = np.sqrt(np.mean(vel_err**2, axis=0))
  rmse_orientation = np.sqrt(np.mean(quat_errors**2))
  
  pos_nes = np.mean(np.sum(pos_err**2, axis=1) / 3)
  vel_nes = np.mean(np.sum(vel_err**2, axis=1) / 3)
  
  max_pos_err = np.max(np.linalg.norm(pos_err, axis=1))
  max_vel_err = np.max(np.linalg.norm(vel_err, axis=1))
  max_orient_err = np.max(quat_errors)

  print("\nError Metrics:")
  print(f"Position RMSE (x, y, z): {rmse_pos}")
  print(f"Velocity RMSE (vx, vy, vz): {rmse_vel}")
  print(f"Orientation RMSE: {rmse_orientation:.2f}°")
  print(f"\nNormalized Error Squared (should be ~1.0):")
  print(f"Position NES: {pos_nes:.3f}")
  print(f"Velocity NES: {vel_nes:.3f}")
  print(f"\nMaximum Errors:")
  print(f"Max Position Error: {max_pos_err:.2f} m")
  print(f"Max Velocity Error: {max_vel_err:.2f} m/s")
  print(f"Max Orientation Error: {max_orient_err:.2f}°")

  if plot:
    def quat_to_euler(q_array):
        return np.array([R.from_quat(q).as_euler('xyz', degrees=True) for q in q_array])

    est_rpy = quat_to_euler(history['est_quat'])
    truth_rpy = quat_to_euler(history['truth_quat'])

    gps_residuals = np.array([
        entry['gps']['velocity'] - h_gps_vel(state).flatten()
        for entry, state in zip(sensor_data, history['state'])
    ])

    orientation_residuals_deg = np.array([
        quat_error(entry['imu']['orientation'], h_imu_quat(state).flatten())
        for entry, state in zip(sensor_data, history['state'])
    ])

    fig, axs = plt.subplots(4, 3, figsize=(16, 12), sharex=True)
    time = history['time']

    for i, label in enumerate(['x', 'y', 'z']):
        axs[0, i].plot(time, history['truth_pos'][:, i], '-', label='Truth')
        axs[0, i].plot(time, history['gps_pos'][:, i], '.', label='GPS')
        if i == 2:
            axs[0, i].plot(time, [entry['altimeter']['altitude'] for entry in sensor_data], ':', label='Barometer (z)')
        axs[0, i].plot(time, history['est_pos'][:, i], '--', label='EKF')
        axs[0, i].set_ylabel(f'Pos {label} (m)')
        axs[0, i].legend()

    for i, label in enumerate(['vx', 'vy', 'vz']):
        axs[1, i].plot(time, history['truth_vel'][:, i], '-', label='Truth')
        axs[1, i].plot(time, history['est_vel'][:, i], '--', label='EKF')
        axs[1, i].plot(time, [entry['gps']['velocity'][i] for entry in sensor_data], ':', label='GPS Vel')
        axs[1, i].set_ylabel(f'Vel {label} (m/s)')
        axs[1, i].legend()

    for i, label in enumerate(['Roll', 'Pitch', 'Yaw']):
        axs[2, i].plot(time, truth_rpy[:, i], '-', label='Truth')
        axs[2, i].plot(time, est_rpy[:, i], '--', label='EKF')
        axs[2, i].plot(time, [R.from_quat(entry['imu']['orientation']).as_euler('xyz', degrees=True)[i] for entry in sensor_data], ':', label='IMU Orient')
        axs[2, i].set_ylabel(f'{label} (°)')
        axs[2, i].legend()

    pos_err = history['est_pos'] - history['truth_pos']
    vel_err = history['est_vel'] - history['truth_vel']

    axs[3, 0].plot(time, gps_residuals[:, 0], label='GPS dx')
    axs[3, 0].plot(time, gps_residuals[:, 1], label='GPS dy')
    axs[3, 0].plot(time, gps_residuals[:, 2], label='GPS dz')
    axs[3, 0].set_ylabel('GPS Residual (m/s)')
    axs[3, 0].legend()

    axs[3, 1].plot(time, orientation_residuals_deg, label='IMU Orient. Residual')
    axs[3, 1].set_ylabel('Orientation Residual (°)')
    axs[3, 1].set_ylim(bottom=0)
    axs[3, 1].legend()

    axs[3, 2].plot(time, np.linalg.norm(pos_err, axis=1), '-', label='Pos Error')
    axs[3, 2].plot(time, np.linalg.norm(vel_err, axis=1), '-', label='Vel Error')
    axs[3, 2].set_ylabel('Norm Error (m)')
    axs[3, 2].legend()

    axs[-1, 1].set_xlabel('Time (s)')
    fig.suptitle(f'EKF State Estimation (10D): {name}', fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()
  
  return {
    'rmse_pos': rmse_pos, 'rmse_vel': rmse_vel, 'rmse_orientation': rmse_orientation,
    'pos_nes': pos_nes, 'vel_nes': vel_nes, 'max_pos_err': max_pos_err,
    'max_vel_err': max_vel_err, 'max_orient_err': max_orient_err, 'pos_err': pos_err,
    'vel_err': vel_err, 'orient_err': quat_errors, 'time': history['time']
  }

# Define different scenarios
scenarios = [
    {
        "name": "Straight and Level",
        "config": simulate_sensors.make_scenario(name="straight",duration=100.0,speed=10.0).__dict__
    },
    {
        "name": "Circle",
        "config": simulate_sensors.make_scenario(name="figure8",duration=60.0,speed=10.0).__dict__
    },
    {
        "name": "Climbing",
        "config": simulate_sensors.make_scenario(name="spiral_climb",duration=50.0,speed=10.0).__dict__
    },
    {
        "name": "Figure-8",
        "config": simulate_sensors.make_scenario("figure8", duration=120.0, speed=12.0).__dict__
    },
    {
        "name": "Spiral Descent",
        "config": simulate_sensors.make_scenario("spiral_descent", duration=90.0, speed=10.0).__dict__
    },
    {
        "name": "Spiral Climb",
        "config": simulate_sensors.make_scenario("spiral_climb", duration=90.0, speed=10.0).__dict__
    },
    {
        "name": "S-Turn",
        "config": simulate_sensors.make_scenario("sinusoid", duration=80.0, speed=15.0).__dict__
    }
]

if __name__ == "__main__":
  # Run all scenarios
  for scenario in scenarios:
    print(f"\nRunning scenario: {scenario['name']}")
    run_scenario(scenario["config"], scenario["name"])
