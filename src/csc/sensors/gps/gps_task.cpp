/**
 * @file gps_task.cpp
 * @brief Implementation of the GPSTask class.
 *
 * This file contains the implementation of the GPSTask class, which is responsible for
 * managing the GPS sensor, processing its data, and handling state transitions based on
 * received messages.
 */
#include "csc/sensors/gps/gps_task.h"
#include "protocore/include/logger.h"

/**
 * @brief Destructor for the GPSTask.
 *
 * Ensures that the GPS sensor is stopped when the task object is destroyed.
 */
GPSTask::~GPSTask()
{
  gps_sensor.stop();
}

/**
 * @brief Processes incoming messages for the GPSTask.
 *
 * Handles different message types, specifically StateMsg for state transitions
 * and HeartbeatMsg for heartbeat handling. Logs unhandled message types.
 *
 * @param msg The message to process.
 */
void GPSTask::process_message(const msg::Msg& msg)
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
 * Manages the state transitions of the GPSTask. Performs actions specific
 * to entering each state, such as starting or stopping the GPS sensor.
 * Publishes a StateAckMsg upon successful transition.
 *
 * @param new_state The target state to transition to.
 */
void GPSTask::transition_to_state(task::TaskState new_state)
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
      gps_sensor.start([this](const msg::GPSDataMsg& data) { process_gps_data(data); });
      break;
    }

    case task::TaskState::STOPPED:
    case task::TaskState::ERROR:
    {
      gps_sensor.stop();
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
 * @brief Processes a new GPS data message.
 * 
 * This function is called asynchronously when new GPS data is received.
 * @param data The GPSDataMsg received from the GPS interface.
 */
void GPSTask::process_gps_data(const msg::GPSDataMsg& data)
{
  if (current_state == task::TaskState::RUNNING)
  {
    safe_publish(msg::Msg(this, data));
  }
}
