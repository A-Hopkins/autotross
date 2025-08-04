import numpy as np

class KalmanFilter:
  def __init__(self, state_dim, control_dim=0):
    self.state_dim = state_dim
    self.control_dim = control_dim

    self.state = np.zeros((state_dim, 1))                    # x
    self.covariance = np.eye(state_dim)                      # P
    self.transition_matrix = np.eye(state_dim)               # F
    self.control_matrix = np.zeros((state_dim, control_dim)) # B
    self.process_noise = np.eye(state_dim)                   # Q

  def predict(self, control_input=None):
    """
    Kalman Filter Prediction Step:

      x = F * x + B * u         (predict state)
      P = F * P * F.T + Q       (predict covariance)

    Where:
      x: current state vector
      F: transition matrix
      u: control input (optional)
      B: control matrix
      P: current state covariance
      Q: process noise covariance
    """
    if self.control_dim > 0 and control_input is not None:
      assert control_input.shape == (self.control_dim, 1)
      self.state = self.transition_matrix @ self.state + self.control_matrix @ control_input
    else:
      self.state = self.transition_matrix @ self.state

    self.covariance = (
      self.transition_matrix @ self.covariance @ self.transition_matrix.T + self.process_noise
    )

  def update(self, measurement, measurement_matrix, measurement_noise):
    """
    Kalman Filter Update Step:

      y = z - H * x             (innovation)
      S = H * P * H.T + R       (innovation covariance)
      K = P * H.T * S^-1        (Kalman gain)
      x = x + K * y             (updated state)
      P = (I - K * H) * P       (updated covariance)

    Where:
      z: actual measurement
      H: measurement matrix
      R: measurement noise covariance
      x: predicted state
      P: predicted state covariance
      I: identity matrix
    """
    z = measurement
    H = measurement_matrix
    R = measurement_noise

    y = z - H @ self.state
    S = H @ self.covariance @ H.T + R
    K = self.covariance @ H.T @ np.linalg.inv(S)

    self.state = self.state + K @ y
    I = np.eye(self.state_dim)
    self.covariance = (I - K @ H) @ self.covariance


class ExtendedKalmanFilter(KalmanFilter):
  def __init__(self, state_dim, control_dim=0):
    super().__init__(state_dim, control_dim)
    self.debug_logs = []

  def predict(self, state_transition_func, jacobian_func, control_input=None):
    """
    EKF Prediction Step (nonlinear f):

      x = f(x, u)               (nonlinear state propagation)
      F = df/dx                 (Jacobian of f at x)
      P = F * P * F.T + Q       (linearized covariance propagation)

    Where:
      f(x, u): nonlinear transition function
      F: Jacobian of f
      Q: process noise covariance
    """
    if self.control_dim > 0 and control_input is not None:
      self.state = state_transition_func(self.state, control_input)
      F = jacobian_func(self.state, control_input)
    else:
      zero_input = np.zeros((self.control_dim, 1))
      self.state = state_transition_func(self.state, zero_input)
      F = jacobian_func(self.state, zero_input)

    self.covariance = F @ self.covariance @ F.T + self.process_noise

  def update(self, measurement, measurement_noise, measurement_func, jacobian_func, label=""):
    """
    EKF Update Step (nonlinear h):

      y = z - h(x)              (innovation)
      H = dh/dx                 (Jacobian of h at x)
      S = H * P * H.T + R       (innovation covariance)
      K = P * H.T * S^-1        (Kalman gain)
      x = x + K * y             (updated state)
      P = (I - K * H) * P       (updated covariance)

    Where:
      h(x): nonlinear measurement function
      H: Jacobian of h
      R: measurement noise covariance
    """
    z = measurement
    h_x = measurement_func(self.state)
    H = jacobian_func(self.state)

    y = z - h_x
    S = H @ self.covariance @ H.T + measurement_noise
    K = self.covariance @ H.T @ np.linalg.inv(S)

    self.state = self.state + K @ y
    I = np.eye(self.state_dim)
    self.covariance = (I - K @ H) @ self.covariance

    # Save debug info
    self.debug_logs.append({
      "label": label,
      "innovation": y.flatten(),
      "innovation_norm": np.linalg.norm(y),
      "kalman_gain_norm": np.linalg.norm(K, axis=0),  # Per state dim
      "state_correction_norm": np.linalg.norm((K @ y).flatten())
    })
