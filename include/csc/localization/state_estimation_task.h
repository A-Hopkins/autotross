/**
 * @file state_estimation_task.h
 * @brief Defines the StateEstimationTask class, which is responsible for fusing sensor data
 * to produce a robust estimate of the UAV's state using an Extended Kalman Filter (EKF).
 *
 */

#pragma once
#include "core/geo_utils.h"
#include "csc/sensors/airspeed/airspeed.h"
#include "csc/sensors/altimeter/altimeter.h"
#include "csc/sensors/gps/gps.h"
#include "csc/sensors/imu/imu.h"
#include "kfplusplus/include/kfplusplus.h"
#include "msg/airspeed_msg.h"
#include "msg/altimeter_msg.h"
#include "msg/gps_msg.h"
#include "msg/imu_msg.h"
#include "msg/state_estimate_msg.h"
#include "protocore/include/task/task.h"
#include <chrono>

/**
 * @class StateEstimationTask
 * @brief Implements the State Estimation Computer Software Component (CSC).
 *
 * This task is responsible for producing a consistent, time-aligned estimate of the UAV's
 * state (position, velocity, orientation, and angular velocity) by fusing multiple asynchronous
 * and noisy sensor sources. It operates an Extended Kalman Filter (EKF) with a 13-dimensional
 * state vector. The estimator's architecture is built on a modern, IMU-driven kinematic model
 * where high-frequency IMU data drives the prediction step, and other sensors (GPS, Altimeter,
 * Airspeed) provide absolute corrections in the update step.
 */
class StateEstimationTask : public task::Task
{
public:
  /**
   * @brief Factory method to create a shared pointer to an StateEstimationTask instance.
   *
   * Initializes the task after creation.
   * @return A std::shared_ptr<StateEstimationTask> pointing to the newly created instance.
   */
  static std::shared_ptr<StateEstimationTask> create()
  {
    auto instance = std::shared_ptr<StateEstimationTask>(new StateEstimationTask("StateEstimationTask"));
    instance->on_initialize();
    return instance;
  }

  /**
   * @brief Destructor.
   */
  ~StateEstimationTask();

protected:
  /**
   * @brief Protected constructor to enforce creation via the `create` factory method.
   * @param name The name of the task.
   */
  StateEstimationTask(const std::string& name = "StateEstimationTask") : task::Task(name) {}

  /**
   * @brief Processes incoming sensor messages from the broker.
   *
   *  This method is the entry point for all subscribed messages. It delegates the message
   * to the appropriate handler to perform an EKF update or prediction step based on the message
   * type.
   * @param msg The incoming message.
   */
  void process_message(const msg::Msg& msg) override;

  /**
   * @brief Handles state transitions for the task (e.g., initializing, running, shutting down).
   * @param new_state The new state to transition to.
   */
  void transition_to_state(task::TaskState new_state) override;

  /**
   * @brief Initializes the task by subscribing to necessary sensor data topics.
   * Subscribes to IMU, GPS, Altimeter, and Airspeed data messages, which serve as the
   * inputs for the EKF.
   */
  void on_initialize() override
  {
    safe_subscribe(msg::Type::AirspeedDataMsg);
    safe_subscribe(msg::Type::AltimeterDataMsg);
    safe_subscribe(msg::Type::GPSDataMsg);
    safe_subscribe(msg::Type::IMUDataMsg);
  }

private:
  // TODO: Currently just hard code the lat lon
  static constexpr double lat_origin = -35.363262;
  static constexpr double lon_origin = 149.165237;
  static constexpr double alt_origin = 584.0;
  linalg::Vector<3>       ref_ecef   = geo_utils::lla_to_ecef(lat_origin, lon_origin, alt_origin);

  // --- EKF Dimensions ---
  ///< Dimension of the state vector (10: pos_xyz, vel_xzy, quat_xzyw).
  static constexpr size_t STATE_DIM = 10;
  ///< Dimension of the control input vector (6: accel_xyz, gyro_xyz from IMU).
  static constexpr size_t CONTROL_DIM = 6;

  /**
   * @brief Extended Kalman Filter instance for state estimation.
   *
   * This filter is used to predict and update the UAV's state. It uses a 13-dimensional state
   vector to represent the UAV's:
   * Position (x, y, z): In [m], in an East-North-Up (ENU) local tangent plane frame.
   * Velocity (vx, vy, vz): In [m/s], also in the ENU frame.
   * Orientation (qx, qy, qz, qw): As a unit quaternion, representing the rotation from the ENU
   frame to the body frame.
   * Angular Velocity (wx, wy, wz): In [rad/s], in the body frame.

   The EKF employs a prediction-update cycle:
   *   Prediction: Driven by high-rate IMU (accelerometer and gyroscope) measurements, which serve
   as control inputs to propagate the state forward in time using a nonlinear kinematic model.
   *   Update: Corrects the predicted state using measurements from other sensors (GPS, Altimeter,
   Airspeed). Each sensor type has its own measurement function and associated noise covariance.
   */
  kfplusplus::ExtendedKalmanFilter<STATE_DIM, CONTROL_DIM> ekf;

  ///< The process noise covariance matrix (Q). Models uncertainty in the dynamics.
  linalg::Matrix<STATE_DIM, STATE_DIM> Q;

  std::chrono::steady_clock::time_point last_predict_time;
  msg::StateEstimateMsg                 current_state_est{}; ///< Current state of the localization task

  /**
   * @brief Processes incoming sensor data messages (Airspeed, Altimeter, GPS, IMU).
   *
   * This method is responsible for extracting sensor data from the incoming message
   * and feeding it into the Extended Kalman Filter (EKF) for an update step.
   * It distinguishes between different sensor types and calls the appropriate
   * EKF update function with the corresponding measurement and noise covariance.
   * @param sensor_msg The incoming sensor data message.
   */
  void handle_sensor_data(const msg::Msg& sensor_msg);

  /**
   * @brief Publishes the current state estimate
   *
   * This method extracts the current state estimate from the EKF and publishes it
   * as a `msg::StateEstimateMsg` to the message broker. This allows other tasks
   * to consume the latest estimated state of the UAV.
   */
  void publish_estimate();

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
  linalg::Vector<STATE_DIM> state_transition_model(const linalg::Vector<STATE_DIM>& x, const linalg::Vector<CONTROL_DIM>& u, double dt);

  /**
   * @brief The Jacobian of the state transition function, F = df/dx.
   *
   * @param x The current state vector
   * @param u The contrl vector
   * @param dt The time delta in seconds
   *
   * @return The jacobian
   */
  linalg::Matrix<STATE_DIM, STATE_DIM> state_transition_jacobian(const linalg::Vector<STATE_DIM>& x, const linalg::Vector<CONTROL_DIM>& u,
                                                                 double dt);

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
   * @param x The current state vector [px,py,pz, vx,vy,vz, qx,qy,qz,qw].
   * @return The predicted airspeed measurement (a 1-dim vector).
   */
  static linalg::Vector<Airspeed::AIRSPEED_MEASUREMENT_DIM> h_airspeed(const linalg::Vector<STATE_DIM>& x);

  /**
   * @brief Jacobian H(x) of the airspeed measurement function.
   *
   * Computes the partial derivative of h_airspeed with respect to the state x.
   * The Jacobian is a 1x10 row vector: [0, 0, 0, vx/||v||, vy/||v||, vz/||v||, 0, 0, 0, 0].
   *
   * @param x The current state vector.
   * @return The Jacobian matrix of the airspeed measurement.
   */
  static linalg::Matrix<Airspeed::AIRSPEED_MEASUREMENT_DIM, STATE_DIM> H_airspeed(const linalg::Vector<STATE_DIM>& x);

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
  static linalg::Vector<Altimeter::ALTIMETER_MEASUREMENT_DIM> h_altimeter(const linalg::Vector<STATE_DIM>& x);

  /**
   * @brief Jacobian H(x) of the altimeter measurement function.
   *
   * Computes the partial derivative of h_altimeter with respect to the state x.
   * The Jacobian is a 2x10 matrix
   *
   * @param x The current state vector.
   * @return The Jacobian matrix of the altimeter measurement.
   */
  static linalg::Matrix<Altimeter::ALTIMETER_MEASUREMENT_DIM, STATE_DIM> H_altimeter(const linalg::Vector<STATE_DIM>& x);

  /**
   * @brief Measurement function h(x) for GPS position.
   *
   * Predicts the GPS position measurement, which corresponds to the position
   * components [px, py, pz] of the state vector.
   *
   * @param x The current state vector.
   * @return The predicted GPS position measurement (a 3-dim vector).
   */
  static linalg::Vector<GPS::GPS_MEASUREMENT_DIM> h_gps_pos(const linalg::Vector<STATE_DIM>& x);

  /**
   * @brief Jacobian H(x) of the GPS position measurement function.
   *
   * Computes the partial derivative of h_gps_pos with respect to the state x.
   * The Jacobian is a 3x10 matrix with a 3x3 identity matrix in the position block.
   *
   * @param x The current state vector.
   * @return The Jacobian matrix of the GPS position measurement.
   */
  static linalg::Matrix<GPS::GPS_MEASUREMENT_DIM, STATE_DIM> H_gps_pos(const linalg::Vector<STATE_DIM>& x);

  /**
   * @brief Measurement function h(x) for GPS velocity.
   *
   * Predicts the GPS velocity measurement, which corresponds to the velocity
   * components [vx, vy, vz] of the state vector.
   *
   * @param x The current state vector.
   * @return The predicted GPS velocity measurement (a 3-dim vector).
   */
  static linalg::Vector<GPS::GPS_MEASUREMENT_DIM> h_gps_vel(const linalg::Vector<STATE_DIM>& x);

  /**
   * @brief Jacobian H(x) of the GPS velocity measurement function.
   *
   * Computes the partial derivative of h_gps_vel with respect to the state x.
   * The Jacobian is a 3x10 matrix with a 3x3 identity matrix in the velocity block.
   *
   * @param x The current state vector.
   * @return The Jacobian matrix of the GPS velocity measurement.
   */
  static linalg::Matrix<GPS::GPS_MEASUREMENT_DIM, STATE_DIM> H_gps_vel(const linalg::Vector<STATE_DIM>& x);

  /**
   * @brief Measurement function h(x) for the IMU orientation (quaternion).
   *
   * Predicts the IMU orientation measurement, which corresponds to the quaternion
   * components [qx, qy, qz, qw] of the state vector.
   *
   * @param x The current state vector.
   * @return The predicted IMU orientation measurement (a 4-dim vector).
   */
  static linalg::Vector<IMU::IMU_MEASUREMENT_DIM> h_imu_quat(const linalg::Vector<STATE_DIM>& x);

  /**
   * @brief Jacobian H(x) of the IMU orientation measurement function.
   *
   * Computes the partial derivative of h_imu_quat with respect to the state x.
   * The Jacobian is a 4x10 matrix with a 4x4 identity matrix in the orientation block.
   *
   * @param x The current state vector.
   * @return The Jacobian matrix of the IMU orientation measurement.
   */
  static linalg::Matrix<IMU::IMU_MEASUREMENT_DIM, STATE_DIM> H_imu_quat(const linalg::Vector<STATE_DIM>& x);
};
