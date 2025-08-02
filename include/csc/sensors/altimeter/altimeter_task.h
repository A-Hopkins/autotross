/**
 * @file altimeter_task.h
 * @brief Defines the AltimeterTask class, responsible for handling Altimeter data processing.
 */

#pragma once

#include "msg/altimeter_msg.h"
#include "altimeter.h"
#include "protocore/include/task/task.h"

/**
 * @class AltimeterTask
 * @brief A task that manages and processes Altimeter sensor data.
 *
 * The AltimeterTask class is responsible for:
 * - Integrating with the Altimeter sensor interface.
 * - Receiving and processing Altimeter data via a callback.
 * - Handling task lifecycle transitions (RUNNING, STOPPED, etc.).
 */
class AltimeterTask : public task::Task
{
public:
  /**
   * @brief Factory method to create a shared pointer to an AltimeterTask instance.
   *
   * Initializes the task after creation.
   * @return A std::shared_ptr<AltimeterTask> pointing to the newly created instance.
   */
  static std::shared_ptr<AltimeterTask> create()
  {
    auto instance = std::shared_ptr<AltimeterTask>(new AltimeterTask("AltimeterTask"));
    instance->on_initialize();
    return instance;
  }

  /**
   * @brief Destructor. Ensures Altimeter streaming is stopped.
   */
  ~AltimeterTask();

protected:
  /**
   * @brief Constructs an AltimeterTask instance.
   * @param name The name of the task. Defaults to "AltimeterTask".
   */
  AltimeterTask(const std::string& name = "AltimeterTask") : task::Task(name) {}

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
  Altimeter altimeter_sensor; ///< Underlying Altimeter sensor abstraction.

  /**
   * @brief Processes a new Altimeter data message.
   * @param data The AltimeterDataMsg received from the Altimeter interface.
   */
  void process_altimeter_data(const msg::AltimeterDataMsg& data);
};
