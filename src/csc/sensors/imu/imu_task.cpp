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
  for (auto& imu_sensor : imu_sensors)
  {
    imu_sensor.stop(); // Stop all IMU sensors to ensure no data is being processed when the task is destroyed.
  }
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
 * @brief Periodic task processing method.
 *
 * This method is called periodically based on the set interval. It processes IMU data
 * if the task is in the RUNNING state.
 */
void IMUTask::periodic_task_process()
{
  // This method is called periodically based on the set interval.
  // It processes IMU data if the task is in the RUNNING state.
  process_imu_data();
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
      for (auto& imu_sensor : imu_sensors)
      {
        imu_sensor.start(); // Start all IMU sensors to begin data collection.
      }
      break;
    }

    case task::TaskState::STOPPED:
    {
      for (auto& imu_sensor : imu_sensors)
      {
        imu_sensor.stop(); // Stop all IMU sensors to ensure no data is being processed when the task is destroyed.
      }
      break;
    }
    case task::TaskState::ERROR:
    {
      for (auto& imu_sensor : imu_sensors)
      {
        imu_sensor.stop(); // Stop all IMU sensors to ensure no data is being processed when the task is destroyed.
      }
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
 * @brief Processes IMU data collected by IMU sensor(s).
 *
 * This method is called periodically to handle the latest IMU data. It reads
 * the current IMU data and executes the voting algorithm before publishing the data.
 */
void IMUTask::process_imu_data()
{
  if (current_state == task::TaskState::RUNNING)
  {
    // Collect data from all IMU sensors
    std::array<msg::IMUDataMsg, IMU_COUNT> local_imu_data;
    for(size_t i = 0; i < IMU_COUNT; ++i)
    {
      local_imu_data[i] = imu_sensors[i].get_current_data();
    }

    // Vote
    // TODO time this function
    std::array<bool, IMU_COUNT> valid_imus = vote_valid_imus(local_imu_data);

    // Publish Valid IMU data
    for (size_t i = 0; i < IMU_COUNT; ++i)
    {
      if (valid_imus[i])
      {
        safe_publish(msg::Msg(this, local_imu_data[i]));
      }
      else
      {
        imu_sensors[i].set_status(IMU::Status::INVALID);
        Logger::instance().log(LogLevel::ERROR, get_name(),
                               "IMU sensor " + std::to_string(imu_sensors[i].get_id()) +
                                " reported invalid data. Stopping sensor.");
        // TODO possible recovery?
        imu_sensors[i].stop();
      }
    }
  }
}

/**
 * @brief Votes on the validity of IMU data from multiple sensors.
 *
 * This method evaluates the IMU data from all sensors and determines which sensors
 * have valid data based on a voting mechanism. It returns an array indicating the validity
 * of each IMU's data.
 * @param imu_data An array containing the latest IMU data from each sensor.
 * @return An array of booleans indicating whether each IMU's data is valid.
 */
std::array<bool, IMUTask::IMU_COUNT> IMUTask::vote_valid_imus(const std::array<msg::IMUDataMsg, IMU_COUNT>& imu_data)
{
  std::array<bool, IMU_COUNT> valid_imus = {true}; // Initialize all IMUs as valid

  // TODO: Implement a voting algorithm to determine the validity of IMU data
  return valid_imus;
}
