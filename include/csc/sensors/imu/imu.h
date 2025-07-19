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
#include <stdint.h>

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
  enum class Status
  {
    UNINITIALIZED, ///< IMU has not been initialized.
    VALID,         ///< IMU is initialized and ready to provide data.
    INVALID,       ///< IMU is initialized but data is not valid (e.g., sensor mafuntion).
  };
  /**
   * @brief Constructs an IMU interface.
   *
   * Since this is an abstract interface, the constructor does not initialize any hardware or
   * simulation. The actual initialization behavior depends on the compiled implementation.
   * 
   * @param imu_id Unique identifier for the IMU sensor instance.
   */
  IMU(uint16_t imu_id);

  /**
   * @brief Starts the IMU data stream.
   *
   * This function initiates IMU data collection. The actual data retrieval mechanism depends on 
   * the implementation (e.g., hardware polling, subscribing to simulation topics, event-driven updates).
   * The specific implementation is determined at compile time based on build configurations (e.g., `USE_SIM` flag).
   */
  void start();

  /**
   * @brief Stops the IMU data stream.
   *
   * This function halts IMU data collection. The behavior of stopping (e.g., disabling hardware
   * polling, unsubscribing from simulation updates, etc.) is implementation-specific.
   */
  void stop();

  /**
   * @brief Gets the unique identifier for this IMU instance.
   *
   * This function returns the unique identifier assigned to this IMU sensor instance.
   * The ID is typically used to distinguish between multiple IMU sensors in a system.
   *
   */
  uint16_t get_id() const;

  /**
   * @brief Reads the current IMU data.
   *
   * This function retrieves the most recent IMU data from the sensor. The actual data retrieval
   * mechanism is determined by the implementation (e.g., reading from hardware registers, fetching
   * from a simulation environment).
   */
  msg::IMUDataMsg get_current_data() const;

  /**
   * @brief Gets the current IMU status.
   * 
   * This function returns the current status of the IMU sensor, indicating the validity of the data.
   */
  Status get_status() const;

  /**
   * @brief Sets the IMU status.
   *
   * This function updates the current status of the IMU sensor.
   *
   * @param new_status The new status to set for the IMU sensor.
   */
  void set_status(Status new_status);

private:
  Status status; ///< Current status of the IMU sensor (e.g., UNINITIALIZED, VALID, INVALID).
  uint16_t imu_id; ///< Unique identifier for the IMU sensor instance.
  msg::IMUDataMsg current_imu_data{0}; ///< Holds the most recent IMU data read from the sensor.
};
