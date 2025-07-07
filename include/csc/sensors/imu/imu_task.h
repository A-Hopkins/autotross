/**
 * @file imu_task.h
 * @brief Defines the IMUTask class, responsible for handling IMU sensor data processing.
 *
 * The IMUTask class extends BaseTask and integrates with an IMU sensor to process and manage
 * inertial measurement data. It provides functionality to handle and process IMU readings.
 */

#pragma once
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
  IMUTask(const std::string& name = "IMUTask") : task::Task(name) {}

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
  IMU imu_sensor; ///< Instance of the IMU sensor interface used for data retrieval and processing.
                  ///< The specific implementation (hardware or simulation) is determined at compile
                  ///< time.

  /**
   * @brief Processes incoming IMU sensor data received from the IMU sensor instance.
   *
   * This method is typically called by the callback registered with the IMU sensor.
   * If the task is in the `RUNNING` state, it logs the reception of data and
   * publishes the received IMU data as a `IMUDataMsg` for other tasks to consume.
   * @param data An `IMUDataMsg` object containing the latest sensor readings.
   */
  void process_imu_data(const msg::IMUDataMsg& data);
};
