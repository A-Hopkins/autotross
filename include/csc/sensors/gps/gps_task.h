/**
 * @file gps_task.h
 * @brief Defines the GPSTask class, responsible for handling GPS data processing.
 */

#pragma once

#include "msg/gps_msg.h"
#include "gps.h"
#include "protocore/include/task/task.h"

/**
 * @class GPSTask
 * @brief A task that manages and processes GPS sensor data.
 *
 * The GPSTask class is responsible for:
 * - Integrating with the GPS sensor interface.
 * - Receiving and processing GPS data via a callback.
 * - Handling task lifecycle transitions (RUNNING, STOPPED, etc.).
 */
class GPSTask : public task::Task
{
public:
  /**
   * @brief Factory method to create a shared pointer to an GPSTask instance.
   *
   * Initializes the task after creation.
   * @return A std::shared_ptr<GPSTask> pointing to the newly created instance.
   */
  static std::shared_ptr<GPSTask> create()
  {
    auto instance = std::shared_ptr<GPSTask>(new GPSTask("GPSTask"));
    instance->on_initialize();
    return instance;
  }

  /**
   * @brief Destructor. Ensures GPS streaming is stopped.
   */
  ~GPSTask();

protected:
  /**
   * @brief Constructs an GPSTask instance.
   * @param name The name of the task. Defaults to "GPSTask".
   */
  GPSTask(const std::string& name = "GPSTask") : task::Task(name) {}

  /**
   * @brief Processes incoming messages.
   * @param msg The message to process.
   */
  void process_message(const msg::Msg& msg) override;

  /**
   * @brief Transitions the task to a new state.
   *
   */
  void transition_to_state(task::TaskState new_state) override;

  /**
   * @brief Performs initialization steps for the task.
   *
   * Subscribes the task to StateMsg messages.
   */
  void on_initialize() override { safe_subscribe(msg::Type::StateMsg); }

private:
  GPS gps_sensor; ///< Underlying GPS sensor abstraction.

  /**
   * @brief Processes a new GPS data message.
   * @param data The GPSDataMsg received from the GPS interface.
   */
  void process_gps_data(const msg::GPSDataMsg& data);
};
