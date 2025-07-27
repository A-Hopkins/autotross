/**
 * @file imu_task.h
 * @brief Defines the IMUTask class, responsible for handling IMU sensor data processing.
 *
 * The IMUTask class extends BaseTask and integrates with an IMU sensor to process and manage
 * inertial measurement data. It provides functionality to handle and process IMU readings.
 */

#pragma once
#include <array>
#include "imu.h"
#include "msg/imu_msg.h"
#include "protocore/include/task/task.h"

#ifdef UNIT_TESTING
#include <gtest/gtest_prod.h>
#endif

/**
 * @class IMUTask
 * @brief A task that manages and processes IMU sensor data.
 *
 * The IMUTask class is responsible for:
 * - Integrating with the IMU sensor.
 * - Receiving and processing IMU data.
 * - Handling state-based IMU operations.
 */
class IMUTask : public task::Task
{
public:
  static constexpr std::size_t IMU_COUNT = 3; ///< Number of IMU sensors managed by this task.

  static std::shared_ptr<IMUTask> create()
  {
    auto instance = std::shared_ptr<IMUTask>(new IMUTask("IMUTask"));
    instance->on_initialize();
    return instance;
  }

  /**
   * @brief Destructor for IMUTask, ensuring proper resource cleanup.
   */
  ~IMUTask();

protected:
  /**
   * @brief Constructs an IMUTask instance with a specified name.
   *
   * Initializes the task with a given name and sets up the underlying task infrastructure.
   * The IMU sensor itself (`imu_sensor`) is initialized by its default constructor.
   * @param name The name assigned to this task for identification and logging. Defaults to
   * "IMUTask".
   */
  IMUTask(const std::string& name = "IMUTask")
    : task::Task(name),
      imu_sensors{{IMU(1), IMU(2), IMU(3)}}  // Initialize three IMU sensors with unique IDs.
  {
    
  #ifndef UNIT_TESTING
    set_periodic_task_interval(std::chrono::milliseconds(PROCESSING_INTERVAL));
  #endif
  }

  /**
   * @brief Processes incoming messages directed to this task.
   *
   * Handles messages based on their type. Primarily listens for `StateMsg`
   * to trigger state transitions and `HeartbeatMsg` for system health checks.
   * Logs unhandled message types.
   * @param msg The message received by the task.
   */
  void process_message(const msg::Msg& msg) override;

  /**
   * @brief Periodic task processing method.
   *
   * This method is called periodically based on the set interval. It processes IMU data
   * if the task is in the RUNNING state.
   */
  void periodic_task_process() override;

  /**
   * @brief Transitions the task to a new operational state.
   *
   * Manages the internal state of the IMU task. Starts or stops the IMU sensor
   * based on the target state (e.g., starts in `RUNNING`, stops in `STOPPED` or `ERROR`).
   * Publishes a `StateAckMsg` upon completion of the transition.
   * @param new_state The target state for the task (e.g., IDLE, RUNNING, STOPPED).
   */
  void transition_to_state(task::TaskState new_state) override;

  /**
   * @brief Performs initial setup for the task upon creation.
   *
   * Subscribes the task to `StateMsg` to allow the StateManager to control its lifecycle.
   * This method is called internally by the `create()` factory function.
   */
  void on_initialize() override { safe_subscribe(msg::Type::StateMsg); }

private:
#ifdef UNIT_TESTING
  FRIEND_TEST(IMUTaskTest, TransitionsToRunningStartsAllSensors);
  FRIEND_TEST(IMUTaskTest, TransitionsToStopInvalidatesAllSensors);
  FRIEND_TEST(IMUTaskTest, VotingAllSensorsAgree);
  FRIEND_TEST(IMUTaskTest, VotingRejectsOutlier);
  FRIEND_TEST(IMUTaskTest, VotingAllDisagreeReturnsNone);
  FRIEND_TEST(IMUTaskTest, VoteValidIMUsPerformance);
  FRIEND_TEST(IMUTaskTest, SingleActiveSensorIsAlwaysValidIfClusterOfOne);
  FRIEND_TEST(IMUTaskTest, CompareIMUThresholdEdge);
  FRIEND_TEST(IMUTaskTest, DegradeThenRecover);
  FRIEND_TEST(IMUTaskTest, StartDegradedThenRecover);
  FRIEND_TEST(IMUTaskTest, StartDegradedThenInvalid);
  FRIEND_TEST(IMUTaskTest, StaleSensorDetection);
#endif
  static constexpr std::chrono::milliseconds PROCESSING_INTERVAL = std::chrono::milliseconds(1); ///< Interval for processing IMU data.
  static constexpr std::chrono::milliseconds STALE_THRESHOLD = IMU_COUNT * PROCESSING_INTERVAL; ///< Threshold to consider IMU data as stale.
  static constexpr float ANGULAR_VEL_THRESHOLD = 0.2f;  ///< Max angular velocity difference [rad/s]
  static constexpr float LINEAR_ACCEL_THRESHOLD = 0.5f; ///< Max linear acceleration difference [m/s^2]
  static constexpr float ORIENTATION_THRESHOLD = 0.1f;  ///< Max quaternion difference (unitless)
  static constexpr uint8_t IMU_RECOVERY_THRESHOLD = 3;  ///< Number of recovery passes before IMU is considered valid again.
  static constexpr uint8_t IMU_RECOVERY_MAX_FAILURES = 5;  ///< Number of recovery passes before IMU is demoted to invalid.
  static constexpr uint8_t IMU_STALE_THRESHOLD = 10; ///< Number of stale reads before IMU is considered invalid.

  std::array<IMU, IMU_COUNT> imu_sensors; ///< Array of IMU sensor instances for data retrieval and processing.

  /**
   * @brief Processes IMU data collected by IMU sensor(s).
   *
   * This method is called periodically to handle the latest IMU data. It reads
   * the current IMU data and executes the voting algorithm before publishing the data.
   */
  void process_imu_data();

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
  std::array<bool, IMU_COUNT> vote_valid_imus(const std::array<msg::IMUDataMsg, IMU_COUNT>& imu_data, const std::array<bool, IMU_COUNT>& active_mask);

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
  bool compare_imu(const msg::IMUDataMsg& a, const msg::IMUDataMsg& b);
};
