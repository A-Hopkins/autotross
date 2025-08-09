/**
 * @file state_estimation_task.h
 * @brief Defines the StateEstimationTask class, which is responsible for fusing sensor data
 * to produce a robust estimate of the UAV's state using an Extended Kalman Filter (EKF).
 * 
 */

#pragma once
#include "kfplusplus/include/kfplusplus.h"
#include "msg/airspeed_msg.h"
#include "msg/altimeter_msg.h"
#include "msg/gps_msg.h"
#include "msg/imu_msg.h"
#include "msg/state_estimate_msg.h"

#include "protocore/include/task/task.h"

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
  static std::shared_ptr<StateEstimationTask> create ()
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
  // --- EKF Dimensions ---
  static constexpr size_t STATE_DIM = 13;                ///< Dimension of the state vector (13: pos_xyz, vel_xzy, quat_xzyw, omega_xzy).
  static constexpr size_t CONTROL_DIM = 6;               ///< Dimension of the control input vector (6: accel_xyz, gyro_xyz from IMU).
  static constexpr size_t AIRSPEED_MEASUREMENT_DIM = 1;  ///< Dimension of the airspeed measurement (1: true airspeed).
  static constexpr size_t ALTIMETER_MEASUREMENT_DIM = 1; ///< Dimension of the altimeter measurement (1: altitude).
  static constexpr size_t GPS_MEASUREMENT_DIM = 3;       ///< Dimension of the GPS position measurement (3: x, y, z).
  static constexpr size_t IMU_MEASUREMENT_DIM = 4;       ///< Dimension of the IMU orientation measurement (4: qx, qy, qz, qw).

  /**
   * @brief Extended Kalman Filter instance for state estimation.
   *
   * This filter is used to predict and update the UAV's state. It uses a 13-dimensional state vector to represent the UAV's:
   * Position (x, y, z): In [m], in an East-North-Up (ENU) local tangent plane frame.
   * Velocity (vx, vy, vz): In [m/s], also in the ENU frame.
   * Orientation (qx, qy, qz, qw): As a unit quaternion, representing the rotation from the ENU frame to the body frame.
   * Angular Velocity (wx, wy, wz): In [rad/s], in the body frame.

   The EKF employs a prediction-update cycle:
   *   Prediction: Driven by high-rate IMU (accelerometer and gyroscope) measurements, which serve as control inputs to propagate the state forward in time using a nonlinear kinematic model.
   *   Update: Corrects the predicted state using measurements from other sensors (GPS, Altimeter, Airspeed). Each sensor type has its own measurement function and associated noise covariance.
   */
  kfplusplus::ExtendedKalmanFilter<STATE_DIM, CONTROL_DIM> ekf;
  
  linalg::Matrix<STATE_DIM, STATE_DIM> Q; ///< The process noise covariance matrix (Q). Models uncertainty in the dynamics.

  // --- Measurement Noise Covariance Matrices (R) ---
  linalg::Matrix<AIRSPEED_MEASUREMENT_DIM, AIRSPEED_MEASUREMENT_DIM> R_airspeed;    ///< Measurement noise for the airspeed sensor.
  linalg::Matrix<ALTIMETER_MEASUREMENT_DIM, ALTIMETER_MEASUREMENT_DIM> R_altimeter; ///< Measurement noise for the altimeter.
  linalg::Matrix<GPS_MEASUREMENT_DIM, GPS_MEASUREMENT_DIM> R_gps;                   ///< Measurement noise for GPS position.
  linalg::Matrix<GPS_MEASUREMENT_DIM, GPS_MEASUREMENT_DIM> R_gps_vel;               ///< Measurement noise for GPS velocity.
  linalg::Matrix<IMU_MEASUREMENT_DIM, IMU_MEASUREMENT_DIM> R_imu;                   ///< Measurement noise for the IMU's fused orientation (quaternion).

  msg::StateEstimateMsg current_state_est{}; ///< Current state of the localization task

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

};
