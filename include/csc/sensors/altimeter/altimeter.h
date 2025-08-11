/**
 * @file altimeter.h
 * @brief Defines an abstract Altimeter interface for sensor data retrieval.
 *
 * This header provides an abstract interface for altimeter sensor interaction. The actual implementation
 * of the altimeter functionality is determined at compile-time using conditional compilation, allowing
 * for flexibility in selecting between different altimeter data sources (e.g., hardware-based or Gazebo
 * simulation).
 */

#pragma once

#include "kfplusplus/include/linalg.h"
#include "msg/altimeter_msg.h"
#include <functional>

/**
 * @class Altimeter
 * @brief Abstract interface for altimeter sensor data retrieval.
 *
 * The Altimeter class defines a generic interface for interacting with a barometric altimeter.
 * The actual implementation is determined by conditional compilation, meaning that the source
 * of altimeter data (e.g., real hardware or a simulation environment) is specified at build time.
 *
 * This abstraction allows for seamless switching between different altimeter data sources without
 * modifying high-level code that relies on barometric altitude or pressure readings.
 */
class Altimeter
{
public:
  inline static constexpr size_t ALTIMETER_MEASUREMENT_DIM = 2;   ///< Dim of the altimeter measurement (2: altitude, vertical velocity).
  inline static const double     ALTIMETER_STDDEV          = 0.5; ///< [m]

  // --- Measurement Noise Covariance Matrices (R) ---
  inline static const linalg::Matrix<ALTIMETER_MEASUREMENT_DIM, ALTIMETER_MEASUREMENT_DIM> R_altimeter =
      linalg::Matrix<ALTIMETER_MEASUREMENT_DIM, ALTIMETER_MEASUREMENT_DIM>::identity() * (ALTIMETER_STDDEV * ALTIMETER_STDDEV); // Variance
  /**
   * @brief Constructs an Altimeter interface.
   *
   * Since this is an abstract interface, the constructor does not initialize any hardware or
   * simulation. The actual initialization behavior depends on the compiled implementation.
   */
  Altimeter();

  /**
   * @brief Starts the altimeter data stream and registers a callback function to receive data.
   *
   * This function initiates altimeter data collection and calls the provided callback whenever new
   * data is available. The actual data retrieval mechanism depends on the implementation (hardware
   * polling, event-driven updates, etc.).
   *
   * @param callback A function that receives altimeter data as a `msg::AltimeterDataMsg`, representing
   * sensor readings. The callback function will be invoked asynchronously whenever new altimeter
   * data arrives. It is crucial that the callback function is thread-safe if the underlying
   * implementation uses multiple threads. The `msg::AltimeterDataMsg` object passed to the callback
   * contains the latest barometric altitude, pressure, and temperature measurements.
   */
  virtual void start(std::function<void(const msg::AltimeterDataMsg&)> callback);

  /**
   * @brief Stops the altimeter data stream.
   *
   * This function halts altimeter data collection. The behavior of stopping (e.g., disabling hardware
   * polling, unsubscribing from simulation updates, etc.) is implementation-specific.
   */
  virtual void stop();

private:
  bool running = false; ///< Indicates if the altimeter data stream is currently running.
};
