/**
 * @file imu_task.cpp
 * @brief Implementation of the IMUTask class.
 *
 * This file contains the implementation of the IMUTask class, which is responsible for
 * managing the IMU sensor, processing its data, and handling state transitions based on
 * received messages.
 */
#include "csc/sensors/imu/imu_task.h"
#include "csc/sensors/imu/imu.h"
#include "protocore/include/logger.h"

/**
 * @brief Destructor for the IMUTask class.
 *
 * Ensures that the IMU sensor is stopped when the task object is destroyed.
 */
IMUTask::~IMUTask()
{
  imu_sensor.stop();
}

/**
 * @brief Processes incoming messages for the IMU task.
 *
 * Handles different message types, specifically StateMsg for state transitions
 * and HeartbeatMsg for heartbeat handling. Other message types are logged as unhandled.
 *
 * @param msg The message to process.
 */
void IMUTask::process_message(const msg::Msg& msg)
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
 * @brief Transitions the task to a new state.
 *
 * Performs actions associated with entering the new state, such as starting or stopping
 * the IMU sensor. It also updates the current state and publishes a StateAckMsg.
 *
 * @param new_state The target state to transition to.
 */
void IMUTask::transition_to_state(task::TaskState new_state)
{
  if (new_state == current_state)
    return;

  Logger::instance().log(LogLevel::INFO, get_name(),
                         " transitioning to " + task_state_to_string(new_state));
  current_state = new_state;

  switch (new_state)
  {
    case task::TaskState::NOT_STARTED:
    {
      break;
    }
    case task::TaskState::IDLE:
    {
      break;
    }
    case task::TaskState::RUNNING:
    {
      imu_sensor.start([this](const msg::IMUDataMsg& data) { process_imu_data(data); });
      break;
    }

    case task::TaskState::STOPPED:
    {
      imu_sensor.stop();
      break;
    }
    case task::TaskState::ERROR:
    {
      imu_sensor.stop();
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
 * @brief Processes IMU data received from the sensor.
 *
 * If the task is in the RUNNING state, it logs the reception of IMU data
 * and publishes the data as an IMUDataMsg.
 *
 * @param data The IMU data message received from the sensor.
 */
void IMUTask::process_imu_data(const msg::IMUDataMsg& data)
{
  if (current_state == task::TaskState::RUNNING)
  {
    safe_publish(msg::Msg(this, data));
  }
}
