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
    set_periodic_task_interval(std::chrono::milliseconds(PROCESSING_INTERVAL));
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
  static constexpr std::size_t IMU_COUNT = 3; ///< Number of IMU sensors managed by this task.
  static constexpr std::chrono::milliseconds PROCESSING_INTERVAL = std::chrono::milliseconds(1); ///< Interval for processing IMU data.
  static constexpr std::chrono::milliseconds STALE_THRESHOLD = IMU_COUNT * PROCESSING_INTERVAL; ///< Threshold to consider IMU data as stale.
  std::array<IMU, IMU_COUNT> imu_sensors; ///< Array of IMU sensor instances for data retrieval and processing.

  /**
   * @brief Processes IMU data collected by IMU sensor(s).
   *
   * This method is called periodically to handle the latest IMU data. It reads
   * the current IMU data and executes the voting algorithm before publishing the data.
   */
  void process_imu_data();

  /**
   * @brief Votes on the validity of IMU data from multiple sensors.
   *
   * This method evaluates the IMU data from all sensors and determines which sensors
   * have valid data based on a voting mechanism. It returns an array indicating the validity
   * of each IMU's data.
   * @param imu_data An array containing the latest IMU data from each sensor.
   * @return An array of booleans indicating whether each IMU's data is valid.
   */
  std::array<bool, IMU_COUNT> vote_valid_imus(const std::array<msg::IMUDataMsg, IMU_COUNT>& imu_data);
};
