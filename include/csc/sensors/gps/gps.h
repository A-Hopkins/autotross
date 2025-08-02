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
  bool running = false;     ///< Indicates if the GPS data stream is currently running.
};
