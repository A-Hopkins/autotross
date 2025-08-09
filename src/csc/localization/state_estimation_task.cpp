/**
 * @file state_estimation_task.cpp
 * @brief Implementation of the StateEstimationTask class.
 *
 * This file contains the implementation of the StateEstimationTask class, which is responsible 
 * for fusing sensor data to produce a robust estimate of the UAV's state using an Extended Kalman Filter (EKF).
 */
#include "csc/localization/state_estimation_task.h"
#include "protocore/include/logger.h"

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
      Logger::instance().log(LogLevel::WARN, get_name(),
                             " received unhandled message type: " +
                                 msg::msg_type_to_string(msg.get_type()));
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

  Logger::instance().log(LogLevel::INFO, get_name(),
                         " transitioning to " + task_state_to_string(new_state));
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
      Logger::instance().log(LogLevel::ERROR, get_name(),
                             "Unknown state transition requested: " +
                                 task_state_to_string(new_state));
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
      break;
    }
    case msg::Type::AltimeterDataMsg:
    {
      break;
    }
    case msg::Type::GPSDataMsg:
    {
      break;
    }
    case msg::Type::IMUDataMsg:
    {
      break;
    }
    default:
    {
      Logger::instance().log(LogLevel::ERROR, get_name(),
                             "Unknown sensor type: " +
                                 msg_type_to_string(sensor_msg.get_type()));
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

  current_state_est.position(0) = x(0);
  current_state_est.position(1) = x(1);
  current_state_est.position(2) = x(2);
  current_state_est.velocity(0) = x(3);
  current_state_est.velocity(1) = x(4);
  current_state_est.velocity(2) = x(5);
  current_state_est.orientation(0) = x(6);
  current_state_est.orientation(1) = x(7);
  current_state_est.orientation(2) = x(8);
  current_state_est.orientation(3) = x(9);
  current_state_est.angular_velocity(0) = x(10);
  current_state_est.angular_velocity(1) = x(11);
  current_state_est.angular_velocity(2) = x(12);

  current_state_est.covariance = P;

  safe_publish(msg::Msg(this, current_state_est));
}
