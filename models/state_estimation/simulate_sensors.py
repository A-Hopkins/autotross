import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

# Constants
G = 9.80665  # gravity [m/s^2]
R_AIR = 287.05  # specific gas constant for dry air [J/(kg·K)]
P0 = 101325  # standard pressure at sea level [Pa]
EARTH_RADIUS = 6378137.0  # meters

# Utility functions
def linspace_vec(start, end, n):
  return np.linspace(start, end, n).reshape(n, -1)

def quaternion_slerp(q0, q1, t_array):
  key_rots = R.from_quat([q0, q1])
  slerp = Slerp([0, 1], key_rots)
  return slerp(t_array).as_quat()

def add_noise(vec, std):
  return vec + np.random.normal(0, std, vec.shape)

def compute_diff_pressure(rho, v):
  return 0.5 * rho * v**2

def compute_air_density(pressure, temperature):
  return pressure / (R_AIR * temperature)

def enu_to_geodetic(east, north, up, lat0_deg, lon0_deg):
  dlat = north / EARTH_RADIUS
  dlon = east / (EARTH_RADIUS * np.cos(np.deg2rad(lat0_deg)))
  lat = lat0_deg + np.rad2deg(dlat)
  lon = lon0_deg + np.rad2deg(dlon)
  return lat, lon, up

def geodetic_to_enu(lat, lon, alt, lat0, lon0, alt0):
  """
  Convert (lat, lon, alt) in degrees/meters to local ENU frame in meters.
  Assumes a flat Earth local tangent plane at (lat0, lon0, alt0).
  """
  dlat = np.deg2rad(lat - lat0)
  dlon = np.deg2rad(lon - lon0)
  avg_lat = np.deg2rad((lat + lat0) / 2.0)

  de = dlon * EARTH_RADIUS * np.cos(avg_lat)
  dn = dlat * EARTH_RADIUS
  du = alt - alt0

  return np.array([de, dn, du])

def generate_figure8_waypoints(a=100.0, b=50.0, height=100.0, num_pts=32):
  """
  figure 8 in the EN plane:
    x =  a * sin(t)
    y =  b * sin(t) * cos(t)
  """
  t = np.linspace(0, 2*np.pi, num_pts, endpoint=False)
  x = a * np.sin(t)
  y = b * np.sin(t) * np.cos(t)
  z = np.full_like(t, height)
  return np.column_stack((x, y, z))

def generate_spiral_waypoints(radius=80.0, turns=3, height_start=150.0,
                              height_end=50.0, num_pts=60):
  """
  Horizontal spiral (constant radius, descending or climbing).
  """
  theta = np.linspace(0, 2*np.pi*turns, num_pts)
  x = radius * np.cos(theta)
  y = radius * np.sin(theta)
  z = np.linspace(height_start, height_end, num_pts)
  return np.column_stack((x, y, z))

def generate_sinusoid_waypoints(length=400.0, amplitude=50.0,
                                wavelength=100.0, height=100.0, num_pts=40):
  """
  S-turn slalom along +X with sinusoidal Y excursions.
  """
  x = np.linspace(0, length, num_pts)
  y = amplitude * np.sin(2*np.pi * x / wavelength)
  z = np.full_like(x, height)
  return np.column_stack((x, y, z))

def generate_circle_waypoints(radius=100.0, height=100.0, num_points=8):
  """Generate waypoints for a circular path"""
  theta = np.linspace(0, 2*np.pi, num_points+1)[:-1]  # exclude last point to avoid duplicate
  x = radius * np.cos(theta)
  y = radius * np.sin(theta)
  z = np.full_like(theta, height)
  points = np.vstack([x, y, z]).T
  return points

def compute_waypoint_timing(waypoints, speed):
  """Compute timing for each waypoint based on constant speed"""
  distances = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
  segment_times = distances / speed
  times = np.zeros(len(waypoints))
  times[1:] = np.cumsum(segment_times)
  return times

def interpolate_position(waypoints, waypoint_times, t):
  """Interpolate position between waypoints at time t"""
  if t <= waypoint_times[0]: return waypoints[0]
  if t >= waypoint_times[-1]: return waypoints[-1]
    
  idx = np.searchsorted(waypoint_times, t) - 1
  t_ratio = (t - waypoint_times[idx]) / (waypoint_times[idx+1] - waypoint_times[idx])
  return waypoints[idx] + t_ratio * (waypoints[idx+1] - waypoints[idx])

def compute_orientation_from_path(pos_current, pos_next, up=np.array([0, 0, 1])):
  """Compute orientation quaternion from current and next positions
  Uses atan2 to compute proper heading angle and maintains continuous yaw"""
  direction = pos_next - pos_current
  if np.all(np.abs(direction) < 1e-6):
    return np.array([0, 0, 0, 1])
  
  # Compute heading angle using atan2 for proper quadrant handling
  yaw = np.arctan2(direction[1], direction[0])  # atan2(y, x) for heading
  
  # Compute pitch angle
  horizontal_distance = np.sqrt(direction[0]**2 + direction[1]**2)
  pitch = np.arctan2(direction[2], horizontal_distance)
  
  # Create rotation matrix from Euler angles (assuming zero roll)
  rotation = R.from_euler('xyz', [0, pitch, yaw])
  return rotation.as_quat()

def make_scenario(name, duration, speed):
  """
  Convenience factory -> returns a ready Scenario instance.
  Valid names: 'figure8', 'spiral_descent', 'spiral_climb',
                'sinusoid', 'straight'.
  """
  if name == "figure8":
    wps = generate_figure8_waypoints()
  elif name == "spiral_descent":
    wps = generate_spiral_waypoints(height_start=150, height_end=50)
  elif name == "spiral_climb":
    wps = generate_spiral_waypoints(height_start=50, height_end=150)
  elif name == "sinusoid":
    wps = generate_sinusoid_waypoints()
  elif name == "straight":
    wps = np.array([[0., 0., 100.], [500., 0., 100.]])
  else:
    raise ValueError(f"Unknown scenario '{name}'")

  # orientation endpoints don’t matter because generate_trajectory()
  # will override them using path-based yaw/pitch.
  return Scenario(
    name="custom", waypoints=wps, duration_sec=duration,
    speed=speed, start_orientation=np.array([0,0,0,1]),
    end_orientation=np.array([0,0,0,1])
  )

# Scenario configuration
class Scenario:
  def __init__(
    self,
    duration_sec=10.0,
    rate_hz=10,
    start_orientation=np.array([0, 0, 0, 1]),
    end_orientation=np.array([0, 0.707, 0, 0.707]),
    start_velocity=np.array([0.0, 0.0, 0.0]),
    end_velocity=np.array([10.0, 0.0, 0.0]),
    start_position=np.array([0.0, 0.0, 100.0]),
    end_position=np.array([100.0, 0.0, 110.0]),
    start_latlon=(37.4275, -122.1697),
    temperature=288.15,
    pressure=101325.0,
    # Updated sensor noise values based on real-world specs
    gps_pos_noise=1.5,      # meters
    gps_vel_noise=0.5,      # m/s
    imu_accel_noise=0.2,    # m/s^2
    imu_gyro_noise=0.005,   # rad/s
    orientation_noise=0.005, # quaternion noise
    baro_noise=0.5,         # meters
    dp_noise=1.0,           # Pa
    name="custom",
    waypoints=None,
    speed=10.0
    ):
      self.duration_sec = duration_sec  # store as duration_sec
      self.N = int(duration_sec * rate_hz)
      self.dt = 1.0 / rate_hz
      self.times = np.linspace(0, duration_sec, self.N)

      self.start_orientation = start_orientation
      self.end_orientation = end_orientation
      self.start_velocity = start_velocity
      self.end_velocity = end_velocity
      self.start_position = start_position
      self.end_position = end_position
      self.temperature = temperature
      self.pressure = pressure
      self.lat0, self.lon0 = start_latlon
      self.name = name
      self.waypoints = waypoints
      self.speed = speed

      # Sensor noise standard deviations
      self.gps_pos_noise = gps_pos_noise # meters
      self.gps_vel_noise = gps_vel_noise  # m/s
      self.imu_accel_noise = imu_accel_noise  # m/s^2
      self.imu_gyro_noise = imu_gyro_noise  # rad/s
      self.orientation_noise = orientation_noise  # quaternion noise (unitless)
      self.baro_noise = baro_noise  # meters
      self.dp_noise = dp_noise  # Pa

  def generate_trajectory(self):
      """Generate full trajectory based on scenario type"""
      if self.name == "circle":
          if self.waypoints is None:
              self.waypoints = generate_circle_waypoints()
      elif self.name == "custom" and self.waypoints is not None:
          pass  # use provided waypoints
      else:
          # Default to linear interpolation between start/end
          self.waypoints = np.vstack([self.start_position, self.end_position])
      
      waypoint_times = compute_waypoint_timing(self.waypoints, self.speed)
      total_time = waypoint_times[-1]
      
      # Scale times to match desired duration
      waypoint_times *= self.duration_sec / total_time
      
      # Generate trajectory points with continuous yaw
      position_seq = []
      velocity_seq = []
      orientation_seq = []
      last_yaw = 0  # Track previous yaw to maintain continuity
      
      for t in self.times:
          pos = interpolate_position(self.waypoints, waypoint_times, t)
          pos_next = interpolate_position(self.waypoints, waypoint_times, t + self.dt)
          
          vel = (pos_next - pos) / self.dt
          quat = compute_orientation_from_path(pos, pos_next)
          
          # Ensure continuous yaw by checking for wraparound
          if len(orientation_seq) > 0:
              prev_euler = R.from_quat(orientation_seq[-1]).as_euler('xyz')
              curr_euler = R.from_quat(quat).as_euler('xyz')
              # Handle yaw wraparound
              diff = curr_euler[2] - prev_euler[2]
              if diff > np.pi:
                  curr_euler[2] -= 2*np.pi
              elif diff < -np.pi:
                  curr_euler[2] += 2*np.pi
              quat = R.from_euler('xyz', curr_euler).as_quat()
          
          position_seq.append(pos)
          velocity_seq.append(vel)
          orientation_seq.append(quat)
      
      return np.array(position_seq), np.array(velocity_seq), np.array(orientation_seq)

  def run(self):
    if hasattr(self, 'waypoints') or self.name != "custom":
      position_seq, velocity_seq, orientation_seq = self.generate_trajectory()
    else:
      position_seq = linspace_vec(self.start_position, self.end_position, self.N)
      velocity_seq = linspace_vec(self.start_velocity, self.end_velocity, self.N)
      orientation_seq = quaternion_slerp(self.start_orientation, self.end_orientation, np.linspace(0, 1, self.N))

    rho = compute_air_density(self.pressure, self.temperature)

    noisy_data = []
    true_data = []
    for i in range(self.N):
      t = self.times[i]
      v = velocity_seq[i]
      p = position_seq[i]
      q = orientation_seq[i]

      # IMU (true)
      accel_true = (velocity_seq[i] - velocity_seq[i-1])/self.dt if i > 0 else np.zeros(3)
  
      # Add gravity compensation
      # Gravity in ENU frame is [0, 0, -G]
      gravity_enu = np.array([0, 0, -G])
      # Convert to body frame using current orientation
      rot = R.from_quat(q)
      gravity_body = rot.inv().apply(gravity_enu)
      # IMU measures proper acceleration (including gravity)
      accel_true = accel_true - gravity_body
  
      if i > 0:
          q_prev = orientation_seq[i-1]
          # Calculate the change in orientation from the previous step to the current
          delta_rot = R.from_quat(q) * R.from_quat(q_prev).inv()
          # The rotation vector represents the axis-angle change.
          # Dividing by dt gives the average angular velocity.
          gyro_true = delta_rot.as_rotvec() / self.dt
      else:
          gyro_true = np.zeros(3)

      # GPS (true)
      lat_true, lon_true, alt_true = enu_to_geodetic(p[0], p[1], p[2], self.lat0, self.lon0)

      # Altimeter (true)
      alt_vel_true = v[2]

      # Airspeed (true)
      tas = np.linalg.norm(v)
      dp_true = compute_diff_pressure(rho, tas)

      # Add noise for each sensor
      accel = add_noise(accel_true, self.imu_accel_noise)
      gyro = add_noise(gyro_true, self.imu_gyro_noise)
      quat = add_noise(q, self.orientation_noise)

      gps_pos = add_noise(p, self.gps_pos_noise)
      gps_vel = add_noise(v, self.gps_vel_noise)
      lat, lon, alt = enu_to_geodetic(gps_pos[0], gps_pos[1], gps_pos[2], self.lat0, self.lon0)

      alt_noise = add_noise(np.array([p[2]]), self.baro_noise)[0]
      dp = add_noise(np.array([dp_true]), self.dp_noise)[0]

      noisy_data.append({
        'time': t,
        'imu': {
          'orientation': quat,
          'angular_velocity': gyro,
          'linear_acceleration': accel
        },
        'gps': {
          'position': (lat, lon, alt),
          'velocity': gps_vel
        },
        'altimeter': {
          'altitude': alt_noise,
          'vertical_velocity': alt_vel_true,
          'reference': 0.0
        },
        'airspeed': {
          'differential_pressure': dp,
          'temperature': self.temperature,
          'true_airspeed': tas
        }
      })

      true_data.append({
        'time': t,
        'imu': {
          'orientation': q,
          'angular_velocity': gyro_true,
          'linear_acceleration': accel_true
        },
        'gps': {
          'position': (lat_true, lon_true, alt_true),
          'velocity': v
        },
        'altimeter': {
          'altitude': p[2],
          'vertical_velocity': alt_vel_true,
          'reference': 0.0
        },
        'airspeed': {
          'differential_pressure': dp_true,
          'temperature': self.temperature,
          'true_airspeed': tas
        }
      })

    return noisy_data, true_data

if __name__ == "__main__":
  np.random.seed(42)
  scenario = Scenario()
  sensor_data, truth_data = scenario.run()

  for entry, truth in zip(sensor_data, truth_data):
    print(f"Time: {entry['time']:.2f}s")
    print("  IMU:")
    print(f"    Orientation: {entry['imu']['orientation']}, Truth: {truth['imu']['orientation']}")
    print(f"    Angular Velocity: {entry['imu']['angular_velocity']}, Truth: {truth['imu']['angular_velocity']}")
    print(f"    Linear Accel: {entry['imu']['linear_acceleration']}, Truth: {truth['imu']['linear_acceleration']}")
    print("  GPS:")
    print(f"    Lat: {entry['gps']['position'][0]:.6f}, Truth: {truth['gps']['position'][0]:.6f}")
    print(f"    Lon: {entry['gps']['position'][1]:.6f}, Truth: {truth['gps']['position'][1]:.6f}")
    print(f"    Alt: {entry['gps']['position'][2]:.2f} m, Truth: {truth['gps']['position'][2]:.2f} m")
    print(f"    Velocity: {entry['gps']['velocity']}, Truth: {truth['gps']['velocity']}")
    print("  Altimeter:")
    print(f"    Altitude: {entry['altimeter']['altitude']:.2f} m, Truth: {truth['altimeter']['altitude']:.2f} m")
    print(f"    Vertical Velocity: {entry['altimeter']['vertical_velocity']:.2f} m/s, Truth: {truth['altimeter']['vertical_velocity']:.2f} m/s")
    print("  Airspeed:")
    print(f"    DP: {entry['airspeed']['differential_pressure']:.2f} Pa, Truth: {truth['airspeed']['differential_pressure']:.2f} Pa")
    print(f"    TAS: {entry['airspeed']['true_airspeed']:.2f} m/s, Truth: {truth['airspeed']['true_airspeed']:.2f} m/s")
    print()
