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

#include "kfplusplus/include/linalg.h"
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
  inline static constexpr size_t IMU_MEASUREMENT_DIM = 4;     ///< Dimension of the IMU orientation measurement (4: qx, qy, qz, qw).
  inline static const double     IMU_STDDEV          = 0.005; ///< [unitless]

  // --- Measurement Noise Covariance Matrices (R) ---
  inline static const linalg::Matrix<IMU_MEASUREMENT_DIM, IMU_MEASUREMENT_DIM> R_imu =
      linalg::Matrix<IMU_MEASUREMENT_DIM, IMU_MEASUREMENT_DIM>::identity() * (IMU_STDDEV * IMU_STDDEV); // Variance
  enum class Status
  {
    UNINITIALIZED, ///< IMU has not been initialized.
    VALID,         ///< IMU is initialized and ready to provide data.
    DEGRADED,      ///< IMU is initialized but data quality is degraded (e.g., single sensor, or not consistently valid).
    INVALID,       ///< IMU is initialized but data is not valid (e.g., sensor mafuntion).
  };

  struct IMUMetaData
  {
    msg::IMUDataMsg imu_data;            ///< The IMU data message containing sensor readings.
    Status          status;              ///< The current status of the IMU sensor.
    uint8_t         recovery_pass_count; ///< Counter for recovery passes, used to track sensor recovery attempts.
    uint8_t         recovery_fail_count; ///< Counter for recovery failures, used to track sensor failure attempts.
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
  virtual void start();

  /**
   * @brief Stops the IMU data stream.
   *
   * This function halts IMU data collection. The behavior of stopping (e.g., disabling hardware
   * polling, unsubscribing from simulation updates, etc.) is implementation-specific.
   */
  virtual void stop();

  /**
   * @brief Gets the unique identifier for this IMU instance.
   *
   * This function returns the unique identifier assigned to this IMU sensor instance.
   * The ID is typically used to distinguish between multiple IMU sensors in a system.
   *
   */
  virtual uint16_t get_id() const;

  /**
   * @brief Reads the current IMU data.
   *
   * This function retrieves the most recent IMU data from the sensor. The actual data retrieval
   * mechanism is determined by the implementation (e.g., reading from hardware registers, fetching
   * from a simulation environment).
   */
  virtual IMUMetaData get_current_data() const;

  /**
   * @brief Gets the current IMU status.
   *
   * This function returns the current status of the IMU sensor, indicating the validity of the data.
   */
  virtual Status get_status() const;

  /**
   * @brief Gets the number of recovery passes.
   *
   * This function returns the number of recovery attempts made by the IMU sensor.
   * It is used to track how many times the sensor has tried to recover from an invalid state.
   */
  virtual uint8_t get_recovery_pass_count() const;

  /**
   * @brief Sets the number of recovery passes.
   *
   * This function updates the number of recovery attempts made by the IMU sensor.
   *
   * @param count The new count of recovery passes.
   */
  virtual void set_recovery_pass_count(uint8_t count);

  /**
   * @brief Gets the number of recovery fails.
   *
   * This function returns the number of recovery failures made by the IMU sensor.
   * It is used to track how many times the sensor has failed to recover from a degraded state.
   */
  virtual uint8_t get_recovery_fail_count() const;

  /**
   * @brief Sets the number of recovery fails.
   *
   * This function updates the number of recovery failures made by the IMU sensor.
   *
   * @param count The new count of recovery fails.
   */
  virtual void set_recovery_fail_count(uint8_t count);

  /**
   * @brief Sets the IMU status.
   *
   * This function updates the current status of the IMU sensor.
   *
   * @param new_status The new status to set for the IMU sensor.
   */
  virtual void set_status(Status new_status);

private:
  Status          status;              ///< Current status of the IMU sensor (e.g., UNINITIALIZED, VALID, INVALID).
  uint16_t        imu_id;              ///< Unique identifier for the IMU sensor instance.
  msg::IMUDataMsg current_imu_data{0}; ///< Holds the most recent IMU data read from the sensor.
  uint8_t         recovery_pass_count; ///< Counter for recovery passes, used to track sensor recovery attempts.
  uint8_t         recovery_fail_count; ///< Counter for recovery failures, used to track sensor failure attempts.
  bool            running     = false; ///< Flag indicating whether the IMU data stream is currently active.
  bool            initialized = false; ///< Flag indicating whether the IMU has been initialized and is ready to provide data.
};
