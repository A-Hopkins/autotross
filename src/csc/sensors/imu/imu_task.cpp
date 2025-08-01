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

#include "core/scoped_timer.h"

#include <algorithm>
#include <chrono>

static std::array<msg::IMUDataMsg, IMUTask::IMU_COUNT> last_imu_sample{};
static const auto start = std::chrono::steady_clock::now();
static std::array<std::chrono::steady_clock::time_point, IMUTask::IMU_COUNT> last_imu_time =
{
  start, start, start
};

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
    // Fetch current metadata from all sensors
    std::array<IMU::IMUMetaData, IMU_COUNT> local_imu_data;
    std::array<msg::IMUDataMsg, IMU_COUNT> voting_candidates;
    std::array<bool, IMU_COUNT> valid_mask;
    std::array<bool, IMU_COUNT> degraded_mask;

    local_imu_data.fill(IMU::IMUMetaData{});
    voting_candidates.fill(msg::IMUDataMsg{});
    valid_mask.fill(false);
    degraded_mask.fill(false);

    const auto now = std::chrono::steady_clock::now();

    // Snapshot IMU data and status
    for (size_t i = 0; i < IMU_COUNT; ++i)
    {
      local_imu_data[i] = imu_sensors[i].get_current_data();

      // If the IMU is not initialized, skip it
      if (local_imu_data[i].status == IMU::Status::UNINITIALIZED)
      {
        Logger::instance().log(LogLevel::WARN, get_name(),
                              "IMU " + std::to_string(imu_sensors[i].get_id()) +
                              " is UNINITIALIZED. Skipping data processing.");
        continue; // Skip uninitialized IMUs
      }

      // Only Valid sensors participate in stale detection
      if (local_imu_data[i].status == IMU::Status::VALID)
      {
        if (!msg::IMUDataMsg::approx_equal(local_imu_data[i].imu_data, last_imu_sample[i], 1e-12))
        {
          last_imu_sample[i] = local_imu_data[i].imu_data;
          last_imu_time[i] = now;
        }
      }

      // Check if the IMU data is stale
      if (now - last_imu_time[i] > STALE_TIMEOUT&& (imu_sensors[i].get_status() != IMU::Status::INVALID &&
          imu_sensors[i].get_status() != IMU::Status::UNINITIALIZED))
      {
        imu_sensors[i].set_status(IMU::Status::INVALID);
        Logger::instance().log(LogLevel::ERROR, get_name(),
                              "IMU " + std::to_string(imu_sensors[i].get_id()) +
                              " marked INVALID due to stale data.");
        continue; // Skip invalid IMUs
      }

      if (local_imu_data[i].status == IMU::Status::VALID)
      {
        valid_mask[i] = true;
        voting_candidates[i] = local_imu_data[i].imu_data;
      }
      else if (local_imu_data[i].status == IMU::Status::DEGRADED)
      {
        degraded_mask[i] = true;
        voting_candidates[i] = local_imu_data[i].imu_data; // Will be used for recovery voting
      }
    }

    // Run primary voting among VALID sensors
    std::array<bool, IMU_COUNT> valid_imus = vote_valid_imus(voting_candidates, valid_mask);

    // Handle VALID sensor publishing and demotion if rejected by vote
    for (size_t i = 0; i < IMU_COUNT; ++i)
    {
      if (valid_mask[i] && valid_imus[i])
      {
        // Publish only currently VALID and vote-confirmed IMUs
        safe_publish(msg::Msg(this, voting_candidates[i]));
      }
      else if (valid_mask[i] && !valid_imus[i])
      {
        // Demote to DEGRADED, reset recovery counters
        imu_sensors[i].set_status(IMU::Status::DEGRADED);
        imu_sensors[i].set_recovery_pass_count(0);

        // TODO: Publish a sensor status so other components can react
        Logger::instance().log(LogLevel::WARN, get_name(),
                              "IMU sensor " + std::to_string(imu_sensors[i].get_id()) +
                              " voted out. Marking as DEGRADED.");
      }
    }

    // Try to recover DEGRADED sensors (do not publish)

    std::array<bool, IMU_COUNT> combined_mask;
    for (size_t i = 0; i < IMU_COUNT; ++i)
    {
      // Combine valid and degraded masks for recovery voting (separate from degraded to track an actual degraded recovery vote)
      combined_mask[i] = (valid_mask[i] || degraded_mask[i]);
    }
    
    // We now test each DEGRADED sensor against the VALID sensors using the same voting logic
    std::array<bool, IMU_COUNT> recovery_vote = vote_valid_imus(voting_candidates, combined_mask);
    for (size_t i = 0; i < IMU_COUNT; ++i)
    {
      if (!degraded_mask[i])
      {
        continue; // Not in recovery set
      }

      if (recovery_vote[i])
      {
        uint8_t pass_count = imu_sensors[i].get_recovery_pass_count() + 1;
        imu_sensors[i].set_recovery_pass_count(pass_count);
        imu_sensors[i].set_recovery_fail_count(0); // reset fail counter

        if (pass_count >= IMU_RECOVERY_THRESHOLD)
        {
          imu_sensors[i].set_status(IMU::Status::VALID);
          imu_sensors[i].set_recovery_pass_count(0);
          Logger::instance().log(LogLevel::INFO, get_name(),
                                "IMU sensor " + std::to_string(imu_sensors[i].get_id()) +
                                " passed recovery threshold. Promoted to VALID.");
        }
      }
      else
      {
        uint8_t fail_count = imu_sensors[i].get_recovery_fail_count() + 1;
        imu_sensors[i].set_recovery_fail_count(fail_count);
        imu_sensors[i].set_recovery_pass_count(0); // reset pass counter

        if (fail_count >= IMU_RECOVERY_MAX_FAILURES)
        {
          imu_sensors[i].set_status(IMU::Status::INVALID);
          Logger::instance().log(LogLevel::ERROR, get_name(),
                                "IMU sensor " + std::to_string(imu_sensors[i].get_id()) +
                                " failed recovery repeatedly. Marked INVALID and stopped.");
        }
      }
    }
  }
}

/**
 * @brief Executes cluster-based consensus voting over IMU sensor data.
 *
 * Given a fixed-size array of IMU readings and a corresponding validity mask, this method
 * identifies the largest group (cluster) of sensors whose measurements mutually agree
 * within defined thresholds. Only sensors that belong to the largest such cluster
 * are considered valid and marked as such in the return value.
 *
 * Method:
 * - For each sensor:
 *     - Compare it to every other VALID sensor using `compare_imu()`.
 *     - Count how many others it agrees with (including itself).
 * - Identify the maximum cluster size.
 * - All sensors that are part of this max-size cluster (size >= 2) are marked VALID.
 *
 * @param imu_data     Array of IMUDataMsg, one per sensor (must be valid where mask[i] is true).
 * @param active_mask  Boolean mask indicating which sensors are currently active and eligible for voting.
 * @return std::array<bool, IMU_COUNT> where each element is true iff the corresponding IMU was included in the maximal agreement cluster.
 *
 * Notes:
 * - If no agreement cluster of size >= 2 is found, all sensors are marked invalid.
 * - This method assumes all IMUs are sampled synchronously and aligned in time.
 * - The voting mechanism is tolerant to one outlier sensor among three (N=3).
 */
std::array<bool, IMUTask::IMU_COUNT> IMUTask::vote_valid_imus(const std::array<msg::IMUDataMsg, IMU_COUNT>& imu_data, const std::array<bool, IMU_COUNT>& active_mask)
{
  ScopedTimer timer("IMUTask::vote_valid_imus");
  std::array<bool, IMU_COUNT> valid_imus;
  valid_imus.fill(false);
  
  size_t best_cluster_size = 0;
  std::array<size_t, IMU_COUNT> cluster_sizes{};
  bool allow_single_sensor_valid = (std::count(active_mask.begin(), active_mask.end(), true) == 1);

  for (size_t i = 0; i < IMU_COUNT; ++i)
  {
    if (!active_mask[i])
    {
      continue;
    }

    size_t count = 1; // Start with self as a valid member of its own cluster
    for (size_t j = 0; j < IMU_COUNT; ++j)
    {
      if (i == j || !active_mask[j])
      {
        continue;
      }
        
      if (compare_imu(imu_data[i], imu_data[j]))
      {
        ++count;
      }
    }
    cluster_sizes[i] = count;
    if (count > best_cluster_size)
    {
      best_cluster_size = count;
    }
  }

  for (size_t i = 0; i < IMU_COUNT; ++i)
  {
    if (cluster_sizes[i] == best_cluster_size && (best_cluster_size >= 2 || allow_single_sensor_valid))
    {
      valid_imus[i] = true;
    }
  }

  return valid_imus;
}

/**
 * @brief Compares two IMU data samples for agreement within configured thresholds.
 *
 * Computes the L2 norm difference between corresponding sensor quantities:
 * - Angular velocity (rad/s)
 * - Linear acceleration (m/s^2)
 * - Orientation quaternion (unitless; comparison accounts for antipodal symmetry)
 *
 * Two IMU samples are considered to agree if all three component differences fall
 * below their respective thresholds.
 *
 * Orientation difference is computed as:
 *   min(||q_1 - q_2||, ||q_1 + q_2||)
 * to account for quaternion sign ambiguity.
 *
 * @param a First IMU data sample.
 * @param b Second IMU data sample.
 * @return true if all vector differences are within threshold limits; false otherwise.
 */
bool IMUTask::compare_imu(const msg::IMUDataMsg& a, const msg::IMUDataMsg& b)
{

  float ang_vel_diff = linalg::norm(a.angular_velocity - b.angular_velocity);
  float lin_acc_diff = linalg::norm(a.linear_acceleration - b.linear_acceleration);
  float orientation_diff = std::min(linalg::norm(a.orientation - b.orientation), linalg::norm(a.orientation + b.orientation));

  return (ang_vel_diff < ANGULAR_VEL_THRESHOLD) &&
         (lin_acc_diff < LINEAR_ACCEL_THRESHOLD) &&
         (orientation_diff < ORIENTATION_THRESHOLD);
}
