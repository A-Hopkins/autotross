/**
 * @file gps.h
 * @brief Defines an abstract GPS interface for sensor data retrieval.
 *
 * This header provides an abstract interface for GPS sensor interaction. The actual implementation
 * of the GPS functionality is determined at compile-time using conditional compilation, allowing
 * for flexibility in selecting between different GPS data sources (e.g., hardware-based or Gazebo
 * simulation).
 */

#pragma once

#include "kfplusplus/include/linalg.h"
#include "msg/gps_msg.h"
#include <functional>

/**
 * @class GPS
 * @brief Abstract interface for GPS sensor data retrieval.
 *
 * The GPS class defines a generic interface for interacting with a GPS.
 * The actual implementation is determined by conditional compilation, meaning that the source
 * of GPS data (e.g., real hardware or a simulation environment) is specified at build time.
 *
 * This abstraction allows for seamless switching between different GPS data sources without
 * modifying high-level code that relies on GPS readings.
 */
class GPS
{
public:
  inline static constexpr size_t GPS_MEASUREMENT_DIM = 3;   ///< Dimension of the GPS position measurement (3: x, y, z).
  inline static const double     GPS_POS_STDDEV      = 1.5; ///< [m]
  inline static const double     GPS_VEL_STDDEV      = 0.5; ///< [m/s]

  // --- Measurement Noise Covariance Matrices (R) ---
  inline static const linalg::Matrix<GPS_MEASUREMENT_DIM, GPS_MEASUREMENT_DIM> R_gps_pos =
      linalg::Matrix<GPS_MEASUREMENT_DIM, GPS_MEASUREMENT_DIM>::identity() * (GPS_POS_STDDEV * GPS_POS_STDDEV); // Variance
  inline static const linalg::Matrix<GPS_MEASUREMENT_DIM, GPS_MEASUREMENT_DIM> R_gps_vel =
      linalg::Matrix<GPS_MEASUREMENT_DIM, GPS_MEASUREMENT_DIM>::identity() * (GPS_VEL_STDDEV * GPS_VEL_STDDEV); // Variance
  /**
   * @brief Constructs an GPS interface.
   *
   * Since this is an abstract interface, the constructor does not initialize any hardware or
   * simulation. The actual initialization behavior depends on the compiled implementation.
   *
   */
  GPS();

  /**
   * @brief Starts the GPS data stream and registers a callback function to receive data.
   *
   * This function initiates GPS data collection and calls the provided callback whenever new
   * data is available. The actual data retrieval mechanism depends on the implementation (hardware
   * polling, event-driven updates, etc.).
   *
   * @param callback A function that receives GPS data as a `msg::GPSDataMsg`, representing
   * sensor readings. The callback function will be invoked asynchronously whenever new GPS
   * data arrives. It is crucial that the callback function is thread-safe if the underlying
   * implementation uses multiple threads. The `msg::GPSDataMsg` object passed to the callback
   * contains the latest GPS position data including latitude, longitude, and other relevant
   * GPS measurements.
   */
  void start(std::function<void(const msg::GPSDataMsg&)> callback);

  /**
   * @brief Stops the GPS data stream.
   *
   * This function halts GPS data collection. The behavior of stopping (e.g., disabling hardware
   * polling, unsubscribing from simulation updates, etc.) is implementation-specific.
   */
  void stop();

private:
  bool running = false; ///< Indicates if the GPS data stream is currently running.
};
