/**
 * @file airspeed_task.h
 * @brief Defines the AirspeedTask class, responsible for handling Airspeed data processing.
 */

#pragma once

#include "msg/airspeed_msg.h"
#include "airspeed.h"
#include "protocore/include/task/task.h"

/**
 * @class AirspeedTask
 * @brief A task that manages and processes Airspeed sensor data.
 *
 * The AirspeedTask class is responsible for:
 * - Integrating with the Airspeed sensor interface.
 * - Receiving and processing Airspeed data via a callback.
 * - Handling task lifecycle transitions (RUNNING, STOPPED, etc.).
 */
class AirspeedTask : public task::Task
{
public:
  /**
   * @brief Factory method to create a shared pointer to an AirspeedTask instance.
   *
   * Initializes the task after creation.
   * @return A std::shared_ptr<AirspeedTask> pointing to the newly created instance.
   */
  static std::shared_ptr<AirspeedTask> create()
  {
    auto instance = std::shared_ptr<AirspeedTask>(new AirspeedTask("AirspeedTask"));
    instance->on_initialize();
    return instance;
  }

  /**
   * @brief Destructor. Ensures Airspeed streaming is stopped.
   */
  ~AirspeedTask();

protected:
  /**
   * @brief Constructs an AirspeedTask instance.
   * @param name The name of the task. Defaults to "AirspeedTask".
   */
  AirspeedTask(const std::string& name = "AirspeedTask") : task::Task(name) {}

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
  Airspeed airspeed_sensor; ///< Underlying Airspeed sensor abstraction.

  /**
   * @brief Processes a new Airspeed data message.
   * @param data The AirspeedDataMsg received from the Airspeed interface.
   */
  void process_airspeed_data(const msg::AirspeedDataMsg& data);
};
