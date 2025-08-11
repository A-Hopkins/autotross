/**
 * @file airspeed.h
 * @brief Defines an abstract Airspeed interface for sensor data retrieval.
 *
 * This header provides an abstract interface for Airspeed sensor interaction. The actual
 * implementation of the Airspeed functionality is determined at compile-time using conditional
 * compilation, allowing for flexibility in selecting between different Airspeed data sources (e.g.,
 * hardware-based or Gazebo simulation).
 */

#pragma once

#include "kfplusplus/include/linalg.h"
#include "msg/airspeed_msg.h"
#include <functional>
#include <tuple>

/**
 * @class Airspeed
 * @brief Abstract interface for Airspeed sensor data retrieval.
 *
 * The Airspeed class defines a generic interface for interacting with an Airspeed sensor.
 * The actual implementation is determined by conditional compilation, meaning that the source
 * of Airspeed data (e.g., real hardware or a simulation environment) is specified at build time.
 *
 * This abstraction allows for seamless switching between different Airspeed data sources without
 * modifying high-level code that relies on Airspeed readings.
 */
class Airspeed
{
public:
  inline static constexpr size_t AIRSPEED_MEASUREMENT_DIM = 1;   ///< Dimension of the airspeed measurement (1: true airspeed).
  inline static const double     DP_STDDEV                = 1.0; ///< [Pa]

  // --- Measurement Noise Covariance Matrices (R) ---
  inline static const linalg::Matrix<AIRSPEED_MEASUREMENT_DIM, AIRSPEED_MEASUREMENT_DIM> R_airspeed =
      linalg::Matrix<AIRSPEED_MEASUREMENT_DIM, AIRSPEED_MEASUREMENT_DIM>::identity() * (DP_STDDEV * DP_STDDEV); // Variance

  /**
   * @brief Constructs an Airspeed interface.
   *
   * Since this is an abstract interface, the constructor does not initialize any hardware or
   * simulation. The actual initialization behavior depends on the compiled implementation.
   */
  Airspeed();

  /**
   * @brief Starts the Airspeed data stream and registers a callback function to receive data.
   *
   * This function initiates Airspeed data collection and calls the provided callback whenever new
   * data is available. The actual data retrieval mechanism depends on the implementation (hardware
   * polling, event-driven updates, etc.).
   *
   * @param callback A function that receives Airspeed data as a `msg::AirspeedDataMsg`,
   * representing sensor readings. The callback function will be invoked asynchronously whenever new
   * Airspeed data arrives. It is crucial that the callback function is thread-safe if the
   * underlying implementation uses multiple threads. The `msg::AirspeedDataMsg` object passed to
   * the callback contains the latest Airspeed data including velocity and other relevant
   * measurements.
   */
  virtual void start(std::function<void(const msg::AirspeedDataMsg&)> callback);

  /**
   * @brief Stops the Airspeed data stream.
   *
   * This function halts Airspeed data collection. The behavior of stopping (e.g., disabling
   * hardware polling, unsubscribing from simulation updates, etc.) is implementation-specific.
   */
  virtual void stop();

  /**
   * @brief Computes true airspeed and its variance from differential pressure and temperature.
   *
   * Uses Bernoulli's equation and the ideal gas law to estimate true airspeed and propagate
   * uncertainty from differential pressure and temperature input variances.
   *
   * @param diff_pressure       Differential pressure [Pa]
   * @param temp                Temperature [K]
   * @param pressure_variance   Variance of differential pressure [Pa^2]
   * @param temp_variance       Variance of temperature [K^2]
   * @return std::tuple<true_airspeed [m/s], variance [m^2/s^2]>
   */
  virtual std::tuple<double, double> compute_true_airspeed(double diff_pressure, double temp, double pressure_variance,
                                                           double temp_variance);

private:
  bool running = false; ///< Indicates if the Airspeed data stream is currently running.
};
