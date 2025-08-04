import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import simulate_sensors
from ekf import ExtendedKalmanFilter

# --- Configuration ---
G_ENU = np.array([0., 0., -9.80665])

# State vector consists of:
# - Position (x,y,z): meters in ENU (East-North-Up) frame
# - Velocity (vx,vy,vz): meters/sec in ENU frame
# - Acceleration (ax,ay,az): meters/sec^2 in ENU frame
# - Orientation (qx,qy,qz,qw): quaternion representing body rotation relative to ENU frame
# - Angular velocity (wx,wy,wz): rad/sec in body frame
state_dim = 16  # [x y z vx vy vz ax ay az qx qy qz qw wx wy wz]


def run_scenario(scenario_config, name="", plot=True):
  """Run a single scenario and generate plots
  Args:
      scenario_config: dict containing scenario parameters
      name: string to identify the scenario in outputs
      plot: boolean to enable/disable plotting
  """
  # --- Scenario Parameters ---
  lat0, lon0, alt0 = 37.7749, -122.4194, 0.0  # San Francisco coordinates
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
  ekf = ExtendedKalmanFilter(state_dim)
  ekf.state = np.zeros((state_dim, 1))
  ekf.state[9:13] = np.array([0, 0, 0, 1]).reshape(-1, 1)  # Initialize quaternion to identity

  # Get tuning parameters from config if provided
  tuning = scenario_config.get('tuning_params', {})
  
  # Initial state uncertainties - median values across scenarios
  pos_uncertainty = tuning.get('pos_uncertainty', 2.531408)
  vel_uncertainty = tuning.get('vel_uncertainty', 0.573733)
  acc_uncertainty = tuning.get('acc_uncertainty', 0.750851)
  quat_uncertainty = tuning.get('quat_uncertainty', 0.403167)
  omega_uncertainty = tuning.get('omega_uncertainty', 0.107382)

  # Process noise parameters - consistent across scenarios
  # Chosing somewhat high process values because our system
  # can be varyingly dynamical and the sensor noise is low, easier to trust
  process_noise_pos = tuning.get('process_noise_pos', 0.000336)
  process_noise_vel = tuning.get('process_noise_vel', 0.054512)
  process_noise_acc = tuning.get('process_noise_acc', 0.096683)
  process_noise_quat = tuning.get('process_noise_quat', 0.005500)
  process_noise_omega = tuning.get('process_noise_omega', 0.025738)

  # Initialize covariance with tuned uncertainties
  ekf.covariance = np.zeros((state_dim, state_dim))
  ekf.covariance[0:3, 0:3] = np.eye(3) * pos_uncertainty
  ekf.covariance[3:6, 3:6] = np.eye(3) * vel_uncertainty
  ekf.covariance[6:9, 6:9] = np.eye(3) * acc_uncertainty
  ekf.covariance[9:13, 9:13] = np.eye(4) * quat_uncertainty
  ekf.covariance[13:16, 13:16] = np.eye(3) * omega_uncertainty

  # Process noise matrix - lower values for smoother estimates
  ekf.process_noise = np.zeros((state_dim, state_dim))
  ekf.process_noise[0:3, 0:3] = np.eye(3) * process_noise_pos
  ekf.process_noise[3:6, 3:6] = np.eye(3) * process_noise_vel
  ekf.process_noise[6:9, 6:9] = np.eye(3) * process_noise_acc
  ekf.process_noise[9:13, 9:13] = np.eye(4) * process_noise_quat
  ekf.process_noise[13:16, 13:16] = np.eye(3) * process_noise_omega

  # --- Dynamics ---
  def f(x, _):
    """State transition with constant acceleration and angular velocity model"""
    pos = x[0:3]
    vel = x[3:6]
    acc = x[6:9]
    quat = x[9:13].astype(np.float64)
    omega = x[13:16].flatten()  # Ensure omega is 1D array
    
    # Update position and velocity using acceleration
    pos_next = pos + vel * dt + 0.5 * acc * dt**2
    vel_next = vel + acc * dt
    
    # Update quaternion using angular velocity
    omega_norm = np.linalg.norm(omega)
    if omega_norm > 1e-10:
        axis = omega / omega_norm
        angle = omega_norm * dt
        dq = R.from_rotvec(axis * angle).as_quat()
        quat_next = R.from_quat(quat.flatten()).as_matrix() @ R.from_quat(dq).as_matrix()
        quat_next = R.from_matrix(quat_next).as_quat()
    else:
        quat_next = quat.flatten()
    
    # Normalize quaternion
    quat_next /= np.linalg.norm(quat_next)
    
    # Ensure all arrays are properly shaped as (n,1) before stacking
    return np.vstack([
        pos_next.reshape(-1, 1),
        vel_next.reshape(-1, 1),
        acc.reshape(-1, 1),
        quat_next.reshape(-1, 1),
        omega.reshape(-1, 1)
    ])

  def F_jacobian(x, _):
    """Jacobian of state transition function"""
    F = np.eye(state_dim)
    # Position affected by velocity and acceleration
    F[0:3, 3:6] = np.eye(3) * dt
    F[0:3, 6:9] = np.eye(3) * 0.5 * dt**2
    # Velocity affected by acceleration
    F[3:6, 6:9] = np.eye(3) * dt
    return F

  # --- Measurement Models ---
  # GPS provides direct measurements of position in ENU frame
  def h_gps(x):
     return x[0:3]
  def H_gps(x):
    H = np.zeros((3, state_dim))
    H[:, 0:3] = np.eye(3)
    return H

  # GPS velocity is also direct measurement in ENU frame
  def h_gps_vel(x):
     return x[3:6]
  def H_gps_vel(x):
    H = np.zeros((3, state_dim))
    H[:, 3:6] = np.eye(3)
    return H

  # Barometer provides direct measurement of altitude (up component)
  def h_baro(x):
     return x[2:3]
  def H_baro(x):
    H = np.zeros((1,state_dim))
    H[0,2] = 1.0
    return H

  # True airspeed sensor provides measurement of airspeed magnitude
  def h_airspeed(x):
     return np.array([[np.linalg.norm(x[3:6])]])
  def H_airspeed(x):
    v = x[3:6].flatten()
    norm_v = np.linalg.norm(v)
    if norm_v == 0:
       return np.zeros((1, state_dim))
    H = np.zeros((1, state_dim))
    H[0, 3:6] = v / norm_v
    return H

  # IMU orientation measurement (direct quaternion measurement)
  def h_imu_quat(x):
     return x[9:13]
  def H_imu_quat(x):
    H = np.zeros((4, state_dim))
    H[:, 9:13] = np.eye(4)
    return H

  # IMU acceleration measurement
  def h_accel(x):
    """
    Predict what the IMU should read (body frame, includes gravity).
    State layout:
      a_enu = x[6:9]
      q     = x[9:13]  (qx,qy,qz,qw)
    """
    a_enu = x[6:9].flatten()
    q     = x[9:13].flatten()
    R_b_e = R.from_quat(q).inv().as_matrix()   # ENU -> body
    a_body_pred = R_b_e @ (a_enu - G_ENU)      # subtract gravity, rotate
    return a_body_pred.reshape(3, 1)
  
  def H_accel(x):
    # Zero Jacobian is acceptable for now; residual still corrects bias.
    return np.zeros((3, state_dim))

  # IMU angular velocity measurement
  def h_gyro(x):
     return x[13:16]
  def H_gyro(x):
    H = np.zeros((3, state_dim))
    H[:, 13:16] = np.eye(3)
    return H

  # --- Measurement Noise ---
  # Use sensor noise values
  R_gps = np.eye(3) * scenario.gps_pos_noise**2
  R_vel = np.eye(3) * scenario.gps_vel_noise**2
  R_baro = np.array([[scenario.baro_noise**2]])
  R_dp = np.array([[scenario.dp_noise**2]])
  R_quat = np.eye(4) * scenario.orientation_noise**2
  R_accel = np.eye(3) * scenario.imu_accel_noise**2
  R_gyro = np.eye(3) * scenario.imu_gyro_noise**2

  # --- Run Filter ---
  history = {k: [] for k in [
      'time', 'truth_pos', 'truth_vel', 'truth_quat', 'state',
      'est_pos', 'est_vel', 'est_quat', 'gps_pos', 'gps_vel'
  ]}

  imu_rate = 1000  # Hz
  gps_rate = 16
  baro_rate = 10
  airspeed_rate = 25
  imu_next, gps_next, baro_next, airspeed_next = 0.0, 0.0, 0.0, 0.0

  for entry, truth in zip(sensor_data, truth_data):
    time = entry['time']

    ekf.predict(f, F_jacobian)

    gps_enu = simulate_sensors.geodetic_to_enu(*entry['gps']['position'], lat0, lon0, alt0)
    # Measurement updates - reorder for better fusion

    if time >= imu_next:
      ekf.update(np.array(entry['imu']['linear_acceleration']).reshape(3, 1), R_accel, h_accel, H_accel, label="IMU Accel")
      ekf.update(np.array(entry['imu']['angular_velocity']).reshape(3, 1), R_gyro, h_gyro, H_gyro, label="IMU Gyro")
      ekf.update(np.array(entry['imu']['orientation']).reshape(4, 1), R_quat, h_imu_quat, H_imu_quat, label="IMU Quat")
      imu_next += 1.0 / imu_rate
    if time >= baro_next:
      ekf.update(np.array([[entry['altimeter']['altitude']]]), R_baro, h_baro, H_baro, label="Baro")  # Baro before GPS
      baro_next += 1.0 / baro_rate
    if time >= gps_next:
      ekf.update(gps_enu.reshape(3, 1), R_gps, h_gps, H_gps, label="GPS Pos")
      ekf.update(np.array(entry['gps']['velocity']).reshape(3, 1), R_vel, h_gps_vel, H_gps_vel, label="gps_vel")
      gps_next += 1.0 / gps_rate
    if time >= airspeed_next:
      ekf.update(np.array([[entry['airspeed']['true_airspeed']]]), R_dp, h_airspeed, H_airspeed, label="Airspeed")
      airspeed_next += 1.0 / airspeed_rate

    x = ekf.state.flatten()
    x[9:13] /= np.linalg.norm(x[9:13])  # normalize quaternion

    history['time'].append(entry['time'])
    truth_enu = simulate_sensors.geodetic_to_enu(*truth['gps']['position'], lat0, lon0, alt0)
    history['truth_pos'].append(truth_enu)
    history['truth_vel'].append(truth['gps']['velocity'])
    history['truth_quat'].append(truth['imu']['orientation'])
    history['state'].append(ekf.state.copy())
    history['est_pos'].append(x[0:3])
    history['est_vel'].append(x[3:6])
    history['est_quat'].append(x[9:13])
    gps_enu = simulate_sensors.geodetic_to_enu(*entry['gps']['position'], lat0, lon0, alt0)
    history['gps_pos'].append(gps_enu)
    history['gps_vel'].append(entry['gps']['velocity'])

  # --- Convert to Arrays ---
  history = {k: np.array(v) for k, v in history.items()}

  # --- Calculate Errors ---
  pos_err = history['est_pos'] - history['truth_pos']
  vel_err = history['est_vel'] - history['truth_vel']

  # Quaternion error (angle between quaternions in degrees)
  def quat_error(q1, q2):
    # Compute the relative rotation between quaternions
    q_diff = R.from_quat(q1).inv() * R.from_quat(q2)
    return np.abs(q_diff.magnitude() * 180/np.pi)  # Convert to degrees
  
  quat_errors = np.array([quat_error(est, truth) 
                          for est, truth in zip(history['est_quat'], history['truth_quat'])])

  # --- Error Metrics ---
  # Position and Velocity RMSE
  rmse_pos = np.sqrt(np.mean(pos_err**2, axis=0))
  rmse_vel = np.sqrt(np.mean(vel_err**2, axis=0))
  rmse_orientation = np.sqrt(np.mean(quat_errors**2))
  
  # Normalized Error Squared (NES)
  pos_nes = np.mean(np.sum(pos_err**2, axis=1) / 3)
  vel_nes = np.mean(np.sum(vel_err**2, axis=1) / 3)
  
  # Maximum errors
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

  # --- Plots ---
  if plot:
    # Convert quaternions to roll-pitch-yaw (ZYX)
    def quat_to_euler(q_array):
      return np.array([R.from_quat(q).as_euler('xyz', degrees=True) for q in q_array])

    # Compute RPY from estimated and true quaternions
    est_rpy = quat_to_euler(history['est_quat'])
    truth_rpy = quat_to_euler(history['truth_quat'])

    # Compute residuals
    gps_residuals = np.array([
    entry['gps']['velocity'] - h_gps_vel(state).flatten()
    for entry, state in zip(sensor_data, history['state'])
    ])

    imu_accel_residuals = np.array([
        entry['imu']['linear_acceleration'] - h_accel(state).flatten()
        for entry, state in zip(sensor_data, history['state'])
    ])

    gps_gains = [log["kalman_gain_norm"] for log in ekf.debug_logs if log["label"] == "gps_vel"]
    plt.plot(gps_gains)
    plt.title("Kalman Gain Norm per State Dim for GPS Velocity")
    plt.xlabel("Time Step")
    plt.ylabel("Gain Norm")
    plt.show()
    quat_gains = [log["kalman_gain_norm"] for log in ekf.debug_logs if log["label"] == "IMU Quat"]
    plt.plot(quat_gains)
    plt.title("Kalman Gain Norm per State Dim for quatern")
    plt.xlabel("Time Step")
    plt.ylabel("Gain Norm")
    plt.show()
    # Create multi-panel plot
    fig, axs = plt.subplots(4, 3, figsize=(16, 12), sharex=True)
    time = history['time']


    # Position
    for i, label in enumerate(['x', 'y', 'z']):
      axs[0, i].plot(time, history['truth_pos'][:, i], '-', label='Truth')
      axs[0, i].plot(time, history['gps_pos'][:, i], '.', label='GPS')
      if i == 2:
        axs[0, i].plot(time, [entry['altimeter']['altitude'] for entry in sensor_data], ':', label='Barometer (z)')
      axs[0, i].plot(time, history['est_pos'][:, i], '--', label='EKF')
      axs[0, i].set_ylabel(f'Pos {label} (m)')
      axs[0, i].legend()

    # Velocity
    for i, label in enumerate(['vx', 'vy', 'vz']):
      axs[1, i].plot(time, history['truth_vel'][:, i], '-', label='Truth')
      axs[1, i].plot(time, history['est_vel'][:, i], '--', label='EKF')
      axs[1, i].plot(time, [entry['gps']['velocity'][i] for entry in sensor_data], ':', label='GPS Vel')
      axs[1, i].set_ylabel(f'Vel {label} (m/s)')
      axs[1, i].legend()

    # RPY angles
    for i, label in enumerate(['Roll', 'Pitch', 'Yaw']):
      axs[2, i].plot(time, truth_rpy[:, i], '-', label='Truth')
      axs[2, i].plot(time, est_rpy[:, i], '--', label='EKF')
      axs[2, i].plot(time, [R.from_quat(entry['imu']['orientation']).as_euler('xyz', degrees=True)[i] for entry in sensor_data], ':', label='IMU Orient')
      axs[2, i].set_ylabel(f'{label} (°)')
      axs[2, i].legend()

    # Error plots (position and velocity RMSE over time)
    pos_err = history['est_pos'] - history['truth_pos']
    vel_err = history['est_vel'] - history['truth_vel']

    # Bottom row: GPS residuals, IMU accel residuals, and total error norm
    axs[3, 0].plot(time, gps_residuals[:, 0], label='GPS dx')
    axs[3, 0].plot(time, gps_residuals[:, 1], label='GPS dy')
    axs[3, 0].plot(time, gps_residuals[:, 2], label='GPS dz')
    axs[3, 0].set_ylabel('GPS Residual (m/s)')
    axs[3, 0].legend()

    axs[3, 1].plot(time, imu_accel_residuals[:, 0], label='IMU ax')
    axs[3, 1].plot(time, imu_accel_residuals[:, 1], label='IMU ay')
    axs[3, 1].plot(time, imu_accel_residuals[:, 2], label='IMU az')
    axs[3, 1].set_ylabel('IMU Accel Residual (m/s²)')
    axs[3, 1].legend()

    axs[3, 2].plot(time, np.linalg.norm(pos_err, axis=1), '-', label='Pos Error')
    axs[3, 2].plot(time, np.linalg.norm(vel_err, axis=1), '-', label='Vel Error')
    axs[3, 2].set_ylabel('Norm Error (m)')
    axs[3, 2].legend()

    axs[-1, 1].set_xlabel('Time (s)')
    fig.suptitle(f'EKF State Estimation: {name}', fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()
  
  # Return metrics
  return {
      'rmse_pos': rmse_pos,
      'rmse_vel': rmse_vel,
      'rmse_orientation': rmse_orientation,
      'pos_nes': pos_nes,
      'vel_nes': vel_nes,
      'max_pos_err': max_pos_err,
      'max_vel_err': max_vel_err,
      'max_orient_err': max_orient_err,
      'pos_err': pos_err,
      'vel_err': vel_err,
      'orient_err': quat_errors,
      'time': history['time']
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
