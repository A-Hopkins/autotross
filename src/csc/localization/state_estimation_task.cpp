/**
 * @file state_estimation_task.cpp
 * @brief Implementation of the StateEstimationTask class.
 *
 * This file contains the implementation of the StateEstimationTask class, which is responsible
 * for fusing sensor data to produce a robust estimate of the UAV's state using an Extended Kalman Filter (EKF).
 */
#include "csc/localization/state_estimation_task.h"
#include "protocore/include/logger.h"

enum StateIndex
{
  PX = 0,
  PY = 1,
  PZ = 2,
  VX = 3,
  VY = 4,
  VZ = 5,
  QX = 6,
  QY = 7,
  QZ = 8,
  QW = 9,
};

StateEstimationTask::~StateEstimationTask() {}

/**
 * @brief Processes incoming sensor messages from the broker.
 *
 *  This method is the entry point for all subscribed messages. It delegates the message
 * to the appropriate handler to perform an EKF update or prediction step based on the message
 * type.
 * @param msg The incoming message.
 */
void StateEstimationTask::process_message(const msg::Msg& msg)
{
  switch (msg.get_type())
  {
    case msg::Type::StateMsg:
    {
      transition_to_state(static_cast<task::TaskState>(msg.get_data_as<msg::StateMsg>()->state));
      break;
    }
    case msg::Type::HeartbeatMsg:
    {
      handle_heartbeat(msg.get_data_as<msg::HeartbeatMsg>());
      break;
    }
    case msg::Type::AirspeedDataMsg:
    case msg::Type::AltimeterDataMsg:
    case msg::Type::GPSDataMsg:
    case msg::Type::IMUDataMsg:
    {
      if (current_state == task::TaskState::RUNNING)
      {
        handle_sensor_data(msg);
      }
      break;
    }

    default:
    {
      Logger::instance().log(LogLevel::WARN, get_name(), " received unhandled message type: " + msg::msg_type_to_string(msg.get_type()));
      break;
    }
  }
}

/**
 * @brief Handles state transitions for the task (e.g., initializing, running, shutting down).
 * @param new_state The new state to transition to.
 */
void StateEstimationTask::transition_to_state(task::TaskState new_state)
{
  if (new_state == current_state)
    return;

  Logger::instance().log(LogLevel::INFO, get_name(), " transitioning to " + task_state_to_string(new_state));
  current_state = new_state;

  switch (new_state)
  {
    case task::TaskState::NOT_STARTED:
    case task::TaskState::IDLE:
    case task::TaskState::RUNNING:
    case task::TaskState::STOPPED:
    case task::TaskState::ERROR:
    {
      break;
    }

    default:
    {
      Logger::instance().log(LogLevel::ERROR, get_name(), "Unknown state transition requested: " + task_state_to_string(new_state));
      break;
    }
  }
  safe_publish(msg::Msg(this, msg::StateAckMsg{static_cast<uint8_t>(current_state)}));
}

/**
 * @brief Processes incoming sensor data messages (Airspeed, Altimeter, GPS, IMU).
 *
 * This method is responsible for extracting sensor data from the incoming message
 * and feeding it into the Extended Kalman Filter (EKF) for an update step.
 * It distinguishes between different sensor types and calls the appropriate
 * EKF update function with the corresponding measurement and noise covariance.
 * @param sensor_msg The incoming sensor data message.
 */
void StateEstimationTask::handle_sensor_data(const msg::Msg& sensor_msg)
{
  switch (sensor_msg.get_type())
  {
    case msg::Type::AirspeedDataMsg:
    {
      auto airspeed_msg = sensor_msg.get_data_as<msg::AirspeedDataMsg>();

      if (airspeed_msg)
      {
        linalg::Vector<Airspeed::AIRSPEED_MEASUREMENT_DIM> z = {airspeed_msg->true_airspeed};
        ekf.update<Airspeed::AIRSPEED_MEASUREMENT_DIM>(z, Airspeed::R_airspeed, h_airspeed, H_airspeed);
      }
      break;
    }
    case msg::Type::AltimeterDataMsg:
    {
      auto altimeter_msg = sensor_msg.get_data_as<msg::AltimeterDataMsg>();

      if (altimeter_msg)
      {
        linalg::Vector<Altimeter::ALTIMETER_MEASUREMENT_DIM> z = {altimeter_msg->vertical_position, altimeter_msg->vertical_velocity};
        ekf.update<Altimeter::ALTIMETER_MEASUREMENT_DIM>(z, Altimeter::R_altimeter, h_altimeter, H_altimeter);
      }
      break;
    }
    case msg::Type::GPSDataMsg:
    {
      auto gps_msg = sensor_msg.get_data_as<msg::GPSDataMsg>();

      if (gps_msg)
      {
        // --- LLA to ENU Conversion ---
        // Convert the current GPS measurement into the local ENU frame.
        linalg::Vector<3> current_ecef = geo_utils::lla_to_ecef(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);
        linalg::Vector<3> current_enu  = geo_utils::ecef_to_enu(current_ecef, ref_ecef, lat_origin, lon_origin);

        ekf.update<GPS::GPS_MEASUREMENT_DIM>(current_enu, GPS::R_gps_pos, h_gps_pos, H_gps_pos);

        linalg::Vector<GPS::GPS_MEASUREMENT_DIM> z_vel = {gps_msg->velocity(0), gps_msg->velocity(1), gps_msg->velocity(2)};
        ekf.update<GPS::GPS_MEASUREMENT_DIM>(z_vel, GPS::R_gps_vel, h_gps_vel, H_gps_vel);
      }
      break;
    }
    case msg::Type::IMUDataMsg:
    {
      auto imu_msg = sensor_msg.get_data_as<msg::IMUDataMsg>();

      if (imu_msg)
      {
        // --- Predict Step ---
        auto   now = std::chrono::steady_clock::now();
        double dt  = std::chrono::duration<double>(now - last_predict_time).count();
        if (dt <= 0.0 || dt > 0.1)
        {
          dt = 0.001; // 1kHz;
        }
        last_predict_time = now;

        linalg::Vector<CONTROL_DIM> u;
        u(0) = imu_msg->linear_acceleration(0);
        u(1) = imu_msg->linear_acceleration(1);
        u(2) = imu_msg->linear_acceleration(2);
        u(3) = imu_msg->angular_velocity(0);
        u(4) = imu_msg->angular_velocity(1);
        u(5) = imu_msg->angular_velocity(2);

        auto f = [this, dt](const auto& x, const auto& u_in) { return state_transition_model(x, u_in, dt); };
        auto F = [this, dt](const auto& x, const auto& u_in) { return state_transition_jacobian(x, u_in, dt); };
        ekf.predict(f, F, u);

        // --- Update Step with Fused IMU Orientation ---
        linalg::Vector<IMU::IMU_MEASUREMENT_DIM> z_quat = {imu_msg->orientation(0), imu_msg->orientation(1), imu_msg->orientation(2),
                                                           imu_msg->orientation(3)};
        ekf.update<IMU::IMU_MEASUREMENT_DIM>(z_quat, IMU::R_imu, h_imu_quat, H_imu_quat);
      }
      break;
    }
    default:
    {
      Logger::instance().log(LogLevel::ERROR, get_name(), "Unknown sensor type: " + msg_type_to_string(sensor_msg.get_type()));
    }
  }

  publish_estimate();
}

/**
 * @brief Publishes the current state estimate
 *
 * This method extracts the current state estimate from the EKF and publishes it
 * as a `msg::StateEstimateMsg` to the message broker. This allows other tasks
 * to consume the latest estimated state of the UAV.
 */
void StateEstimationTask::publish_estimate()
{
  // Grab filter output
  auto x = ekf.get_state();
  auto P = ekf.get_covariance();

  current_state_est.position(0)    = x(0);
  current_state_est.position(1)    = x(1);
  current_state_est.position(2)    = x(2);
  current_state_est.velocity(0)    = x(3);
  current_state_est.velocity(1)    = x(4);
  current_state_est.velocity(2)    = x(5);
  current_state_est.orientation(0) = x(6);
  current_state_est.orientation(1) = x(7);
  current_state_est.orientation(2) = x(8);
  current_state_est.orientation(3) = x(9);

  current_state_est.covariance = P;

  safe_publish(msg::Msg(this, current_state_est));
}

/**
 * @brief The non-linear state transition function f(x, u).
 *
 * Propagates the state forward by one time step `dt` using the full
 * 6-DOF IMU measurement as the control input.
 *
 * @param x The current state vector (10-dim).
 * @param u The control vector from the IMU (6-dim).
 * @param dt The time delta in seconds.
 * @return The predicted next state vector.
 */
linalg::Vector<StateEstimationTask::STATE_DIM> StateEstimationTask::state_transition_model(const linalg::Vector<STATE_DIM>&   x,
                                                                                           const linalg::Vector<CONTROL_DIM>& u, double dt)
{
  constexpr double GRAVITY_ACCELERATION = 9.80665; // m/s^2

  // Deconstruct state and control vectors
  linalg::Vector<3> pos = {x(PX), x(PY), x(PZ)};
  linalg::Vector<3> vel = {x(VX), x(VY), x(VZ)};
  linalg::Vector<4> q   = {x(QX), x(QY), x(QZ), x(QW)};

  linalg::Vector<3> accel_body = {u(0), u(1), u(2)};
  linalg::Vector<3> omega_body = {u(3), u(4), u(5)};

  // --- Attitude Update ---
  linalg::Vector<4> q_next;
  double            omega_norm = linalg::norm(omega_body);
  if (omega_norm > 1e-10)
  {
    linalg::Vector<3> axis       = omega_body * (1.0 / omega_norm);
    double            angle      = omega_norm * dt;
    double            half_angle = angle / 2.0;
    double            sin_half   = std::sin(half_angle);

    // Create delta quaternion dq
    linalg::Vector<4> dq;
    dq(0) = axis(0) * sin_half;   // qx
    dq(1) = axis(1) * sin_half;   // qy
    dq(2) = axis(2) * sin_half;   // qz
    dq(3) = std::cos(half_angle); // qw

    // Compose rotations: q_next = q * dq
    // (Manual quaternion multiplication)
    q_next(0) = q(3) * dq(0) + q(0) * dq(3) + q(1) * dq(2) - q(2) * dq(1);
    q_next(1) = q(3) * dq(1) - q(0) * dq(2) + q(1) * dq(3) + q(2) * dq(0);
    q_next(2) = q(3) * dq(2) + q(0) * dq(1) - q(1) * dq(0) + q(2) * dq(3);
    q_next(3) = q(3) * dq(3) - q(0) * dq(0) - q(1) * dq(1) - q(2) * dq(2);
  }
  else
  {
    q_next = q;
  }
  q_next = q_next * (1.0 / linalg::norm(q_next)); // Normalize

  // --- Velocity & Position Update ---
  double               qw = q(3), qx = q(0), qy = q(1), qz = q(2);
  linalg::Matrix<3, 3> R; // Rotation matrix from body to ENU
  R(0, 0) = 1 - 2 * (qy * qy + qz * qz);
  R(0, 1) = 2 * (qx * qy - qz * qw);
  R(0, 2) = 2 * (qx * qz + qy * qw);
  R(1, 0) = 2 * (qx * qy + qz * qw);
  R(1, 1) = 1 - 2 * (qx * qx + qz * qz);
  R(1, 2) = 2 * (qy * qz - qx * qw);
  R(2, 0) = 2 * (qx * qz - qy * qw);
  R(2, 1) = 2 * (qy * qz + qx * qw);
  R(2, 2) = 1 - 2 * (qx * qx + qy * qy);

  linalg::Vector<3> accel_enu          = R * accel_body;
  linalg::Vector<3> gravity_vec        = {0.0, 0.0, -GRAVITY_ACCELERATION};
  linalg::Vector<3> inertial_accel_enu = accel_enu + gravity_vec;

  linalg::Vector<3> vel_next = vel + inertial_accel_enu * dt;
  linalg::Vector<3> pos_next = pos + vel * dt + (inertial_accel_enu * 0.5) * dt * dt;

  // Reconstruct the state vector
  linalg::Vector<STATE_DIM> x_next;
  x_next(PX) = pos_next(0);
  x_next(PY) = pos_next(1);
  x_next(PZ) = pos_next(2);
  x_next(VX) = vel_next(0);
  x_next(VY) = vel_next(1);
  x_next(VZ) = vel_next(2);
  x_next(QX) = q_next(0);
  x_next(QY) = q_next(1);
  x_next(QZ) = q_next(2);
  x_next(QW) = q_next(3);

  return x_next;
}

/**
 * @brief The Jacobian of the state transition function, F = df/dx.
 *
 * @param x The current state vector
 * @param u The contrl vector
 * @param dt The time delta in seconds
 *
 * @return The jacobian
 */
linalg::Matrix<StateEstimationTask::STATE_DIM, StateEstimationTask::STATE_DIM>
StateEstimationTask::state_transition_jacobian(const linalg::Vector<STATE_DIM>& x, const linalg::Vector<CONTROL_DIM>& u, double dt)
{
  linalg::Matrix<STATE_DIM, STATE_DIM> F = linalg::Matrix<STATE_DIM, STATE_DIM>::identity();
  // d(pos_next)/d(vel) = I * dt
  for (size_t i = 0; i < 3; ++i)
  {
    F(PX + i, VX + i) = dt;
  }
  return F;
}

// --- Measurement Models and Jacobians ---
/**
 * Each sensor provides a measurement that is a function of the state vector.
 * For the EKF, we need two functions for each sensor type:
 * 1. Measurement Function h(x): Predicts the sensor measurement given a
 *    state vector x.
 * 2. Measurement Jacobian H(x): The partial derivative of h(x) with
 *    respect to the state x. This linearizes the measurement function
 *    around the current state estimate and is used to update the covariance.
 */

/**
 * @brief Measurement function h(x) for the airspeed sensor.
 *
 * Predicts the true airspeed, which is the magnitude of the velocity vector.
 * This is a non-linear function of the state.
 * h(x) = sqrt(vx^2 + vy^2 + vz^2)
 *
 * @param x The current state vector
 * @return The predicted airspeed measurement (a 1-dim vector).
 */
linalg::Vector<Airspeed::AIRSPEED_MEASUREMENT_DIM> StateEstimationTask::h_airspeed(const linalg::Vector<STATE_DIM>& x)
{
  linalg::Vector<3> v = {x(VX), x(VY), x(VZ)};
  return {linalg::norm(v)};
}

/**
 * @brief Jacobian H(x) of the airspeed measurement function.
 *
 * Computes the partial derivative of h_airspeed with respect to the state x.
 * The Jacobian is a 1x10 row vector
 *
 * @param x The current state vector.
 * @return The Jacobian matrix of the airspeed measurement.
 */
linalg::Matrix<Airspeed::AIRSPEED_MEASUREMENT_DIM, StateEstimationTask::STATE_DIM>
StateEstimationTask::H_airspeed(const linalg::Vector<STATE_DIM>& x)
{
  linalg::Matrix<Airspeed::AIRSPEED_MEASUREMENT_DIM, STATE_DIM> H;
  linalg::Vector<3>                                             v      = {x(VX), x(VY), x(VZ)};
  double                                                        norm_v = linalg::norm(v);
  if (norm_v > 1e-6)
  {
    H(0, VX) = v(0) / norm_v;
    H(0, VY) = v(1) / norm_v;
    H(0, VZ) = v(2) / norm_v;
  }
  return H;
}

/**
 * @brief Measurement function h(x) for the altimeter.
 *
 * Predicts the altitude and vertical velocity measurement, which correspond to the
 * z-position (pz) and z-velocity (vz) components of the state vector.
 * h(x) = [pz, vz]^T
 *
 * @param x The current state vector.
 * @return The predicted altitude and vertical velocity measurement (a 2-dim vector).
 */
linalg::Vector<Altimeter::ALTIMETER_MEASUREMENT_DIM> StateEstimationTask::h_altimeter(const linalg::Vector<STATE_DIM>& x)
{
  return {x(PZ), x(VZ)};
}

/**
 * @brief Jacobian H(x) of the altimeter measurement function.
 *
 * Computes the partial derivative of h_altimeter with respect to the state x.
 * The Jacobian is a 2x10 matrix
 *
 * @param x The current state vector.
 * @return The Jacobian matrix of the altimeter measurement.
 */
linalg::Matrix<Altimeter::ALTIMETER_MEASUREMENT_DIM, StateEstimationTask::STATE_DIM>
StateEstimationTask::H_altimeter(const linalg::Vector<STATE_DIM>& x)
{
  linalg::Matrix<Altimeter::ALTIMETER_MEASUREMENT_DIM, STATE_DIM> H;
  H(0, PZ) = 1.0;
  H(1, VZ) = 1.0;
  return H;
}

/**
 * @brief Measurement function h(x) for GPS position.
 *
 * Predicts the GPS position measurement, which corresponds to the position
 * components [px, py, pz] of the state vector.
 *
 * @param x The current state vector.
 * @return The predicted GPS position measurement (a 3-dim vector).
 */
linalg::Vector<GPS::GPS_MEASUREMENT_DIM> StateEstimationTask::h_gps_pos(const linalg::Vector<STATE_DIM>& x)
{
  return {x(PX), x(PY), x(PZ)};
}

/**
 * @brief Jacobian H(x) of the GPS position measurement function.
 *
 * Computes the partial derivative of h_gps_pos with respect to the state x.
 * The Jacobian is a 3x10 matrix with a 3x3 identity matrix in the position block.
 *
 * @param x The current state vector.
 * @return The Jacobian matrix of the GPS position measurement.
 */
linalg::Matrix<GPS::GPS_MEASUREMENT_DIM, StateEstimationTask::STATE_DIM> StateEstimationTask::H_gps_pos(const linalg::Vector<STATE_DIM>& x)
{
  linalg::Matrix<GPS::GPS_MEASUREMENT_DIM, STATE_DIM> H;
  H(0, PX) = 1.0;
  H(1, PY) = 1.0;
  H(2, PZ) = 1.0;
  return H;
}

/**
 * @brief Measurement function h(x) for GPS velocity.
 *
 * Predicts the GPS velocity measurement, which corresponds to the velocity
 * components [vx, vy, vz] of the state vector.
 *
 * @param x The current state vector.
 * @return The predicted GPS velocity measurement (a 3-dim vector).
 */
linalg::Vector<GPS::GPS_MEASUREMENT_DIM> StateEstimationTask::h_gps_vel(const linalg::Vector<STATE_DIM>& x)
{
  return {x(VX), x(VY), x(VZ)};
}

/**
 * @brief Jacobian H(x) of the GPS velocity measurement function.
 *
 * Computes the partial derivative of h_gps_vel with respect to the state x.
 * The Jacobian is a 3x10 matrix with a 3x3 identity matrix in the velocity block.
 *
 * @param x The current state vector.
 * @return The Jacobian matrix of the GPS velocity measurement.
 */
linalg::Matrix<GPS::GPS_MEASUREMENT_DIM, StateEstimationTask::STATE_DIM> StateEstimationTask::H_gps_vel(const linalg::Vector<STATE_DIM>& x)
{
  linalg::Matrix<GPS::GPS_MEASUREMENT_DIM, STATE_DIM> H;
  H(0, VX) = 1.0;
  H(1, VY) = 1.0;
  H(2, VZ) = 1.0;
  return H;
}

/**
 * @brief Measurement function h(x) for the IMU orientation (quaternion).
 *
 * Predicts the IMU orientation measurement, which corresponds to the quaternion
 * components [qx, qy, qz, qw] of the state vector.
 *
 * @param x The current state vector.
 * @return The predicted IMU orientation measurement (a 4-dim vector).
 */
linalg::Vector<IMU::IMU_MEASUREMENT_DIM> StateEstimationTask::h_imu_quat(const linalg::Vector<STATE_DIM>& x)
{
  return {x(QX), x(QY), x(QZ), x(QW)};
}

/**
 * @brief Jacobian H(x) of the IMU orientation measurement function.
 *
 * Computes the partial derivative of h_imu_quat with respect to the state x.
 * The Jacobian is a 4x10 matrix with a 4x4 identity matrix in the orientation block.
 *
 * @param x The current state vector.
 * @return The Jacobian matrix of the IMU orientation measurement.
 */
linalg::Matrix<IMU::IMU_MEASUREMENT_DIM, StateEstimationTask::STATE_DIM> StateEstimationTask::H_imu_quat(const linalg::Vector<STATE_DIM>& x)
{
  linalg::Matrix<IMU::IMU_MEASUREMENT_DIM, STATE_DIM> H;
  H(0, QX) = 1.0;
  H(1, QY) = 1.0;
  H(2, QZ) = 1.0;
  H(3, QW) = 1.0;
  return H;
}
