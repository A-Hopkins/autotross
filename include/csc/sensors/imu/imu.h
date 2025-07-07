/**
 * @file imu.h
 * @brief Defines an abstract IMU (Inertial Measurement Unit) interface for sensor data retrieval.
 *
 * This header provides an abstract interface for IMU sensor interaction. The actual implementation
 * of the IMU functionality is determined at compile-time using conditional compilation, allowing
 * for flexibility in selecting between different IMU data sources (e.g., hardware-based or Gazebo
 * simulation).
 */

#pragma once

#include "msg/imu_msg.h"
#include <functional>

/**
 * @class IMU
 * @brief Abstract interface for IMU sensor data retrieval.
 *
 * The IMU class defines a generic interface for interacting with an inertial measurement unit.
 * The actual implementation is determined by conditional compilation, meaning that the source
 * of IMU data (e.g., real hardware or a simulation environment) is specified at build time.
 *
 * This abstraction allows for seamless switching between different IMU data sources without
 * modifying high-level code that relies on IMU readings.
 */
class IMU
{
public:
  /**
   * @brief Constructs an IMU interface.
   *
   * Since this is an abstract interface, the constructor does not initialize any hardware or
   * simulation. The actual initialization behavior depends on the compiled implementation.
   */
  IMU();

  /**
   * @brief Starts the IMU data stream and registers a callback function to receive data.
   *
   * This function initiates IMU data collection and calls the provided callback whenever new data
   * is available. The actual data retrieval mechanism depends on the implementation (e.g., hardware
   * polling, subscribing to simulation topics, event-driven updates). The specific implementation
   * is determined at compile time based on build configurations (e.g., `USE_SIM` flag).
   *
   * @param callback A `std::function` that will be invoked with `msg::IMUDataMsg` objects
   *                 representing the latest sensor readings. The callback should be thread-safe
   *                 if the underlying implementation operates asynchronously.
   */
  void start(std::function<void(const msg::IMUDataMsg&)> callback);

  /**
   * @brief Stops the IMU data stream.
   *
   * This function halts IMU data collection. The behavior of stopping (e.g., disabling hardware
   * polling, unsubscribing from simulation updates, etc.) is implementation-specific.
   */
  void stop();
};
