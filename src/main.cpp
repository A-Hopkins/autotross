#include <iostream>
#include <gz/transport.hh>

#include "csc/sensors/imu/imu_task.h"

#include "protocore/include/broker.h"
#include "protocore/include/heart_beat.h"
#include "protocore/include/logger.h"
#include "protocore/include/state_manager.h"

int main()
{
  // ---------------------------------------------------------------------------
  // STEP 1: Initialize the Logger
  // ---------------------------------------------------------------------------
  // Generate a log file name with a timestamp
  std::ostringstream log_file_stream;
  std::time_t        now = std::time(nullptr);
  char               time_buffer[20]; // Buffer to hold the formatted time
  std::strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d_%H-%M-%S", std::localtime(&now));
  log_file_stream << time_buffer << "_autotank.log";
  std::string log_file = log_file_stream.str();

  // The logger is responsible for capturing and storing logs for debugging and
  // monitoring purposes. Here, we configure it to use a file sink with DEBUG level.
  auto& logger = Logger::instance();
  logger.set_level(LogLevel::DEBUG).add_sink(std::make_unique<FileSink>(log_file)).add_sink(std::make_unique<ConsoleSink>()).use_relative_timestamps(true);

  // ---------------------------------------------------------------------------
  // STEP 2: Initialize the Broker
  // ---------------------------------------------------------------------------
  // The Broker is the communication backbone of the system, enabling publishers
  // and subscribers to exchange messages. This must be initialized before any
  // tasks are created or registered.
  Broker::initialize();

  // ---------------------------------------------------------------------------
  // STEP 3: Create and Configure System Tasks
  // ---------------------------------------------------------------------------
  // Tasks represent the core functionality of the system, such as sensor data
  // processing, localization, mapping, and control. Each task is created and
  // registered with the StateManager, which manages their lifecycle.
  std::shared_ptr<StateManager>  state_manager = StateManager::create();
  std::shared_ptr<HeartBeatTask> heart_beat_task = HeartBeatTask::create("HeartBeat", state_manager);
  std::shared_ptr<IMUTask> imu_task = IMUTask::create();

  // ---------------------------------------------------------------------------
  // STEP 4: Register Tasks with the StateManager
  // ---------------------------------------------------------------------------
  // The StateManager is responsible for managing the lifecycle of all tasks.
  // Tasks are registered here so they can be started, stopped, or transitioned
  // between states as needed. The HeartBeatTask is also set as the observer
  // for task registration events.
  state_manager->set_task_registration_observer(heart_beat_task);

    // Register tasks with the state manager
  state_manager->register_task(heart_beat_task);
  state_manager->register_task(imu_task);

  // ---------------------------------------------------------------------------
  // STEP 5: Initialize and Start the StateManager
  // ---------------------------------------------------------------------------
  // The StateManager is initialized, which prepares all registered tasks for
  // execution. Once initialized, the tasks are transitioned to the RUNNING state.
  state_manager->initialize();
  state_manager->demand_state_transition(task::TaskState::RUNNING);

  // ---------------------------------------------------------------------------
  // STEP 6: Wait for Shutdown Signal
  // ---------------------------------------------------------------------------
  // The system will now wait for a shutdown signal from Gazebo Transport.
  // Once the signal is received, the StateManager will cleanly shut down all
  // tasks and release resources.
  gz::transport::waitForShutdown();
  state_manager->shutdown();

  return 0;
}
