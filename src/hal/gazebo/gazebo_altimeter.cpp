/**
 * @file gazebo_altimeter.cpp
 * @brief Implementation of the Altimeter class for interfacing with Gazebo altimeter data.
 *
 * This file provides the implementation for subscribing to Gazebo's altimeter topic,
 * processing the received messages, and invoking a user-defined callback with
 * the processed altimeter data.
 */
#include "msg/altimeter_msg.h"
#include "csc/sensors/altimeter/altimeter.h"
#include "gazebo_helpers.h"

#include <gz/msgs.hh>
#include <gz/transport.hh>

/// Gazebo transport node for communication.
static gz::transport::Node node;

/// Callback function to be invoked when new altimeter data is received.
static std::function<void(const msg::AltimeterDataMsg&)> altimeter_callback;

/**
 * @brief Constructs an Altimeter interface.
 *
 * Initializes the Altimeter interface for Gazebo implementation.
 */
Altimeter::Altimeter() : running(false) { }

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
void Altimeter::start(std::function<void(const msg::AltimeterDataMsg&)> callback)
{
  altimeter_callback = callback;
  running = true;

  // Subscribe to the altimeter topic in Gazebo
  node.Subscribe<gz::msgs::Altimeter>(
      "/sensors/altimeter",
      [this](const gz::msgs::Altimeter& msg)
      {
        if (!running)
        {
          return;
        }

        // Extract header information from the Gazebo message.
        msg::Header extracted_header = gazebo_helper::extract_header(msg);

        // Check to see if the sim time conversion needs to be done
        if (!gazebo_helper::g_t0_wall_set && !gazebo_helper::g_t0_sim_set)
        {
          gazebo_helper::g_t0_wall     = std::chrono::steady_clock::now();
          gazebo_helper::g_t0_sim_sec  = msg.header().stamp().sec();
          gazebo_helper::g_t0_sim_nsec = msg.header().stamp().nsec();
          gazebo_helper::g_t0_wall_set = true;
          gazebo_helper::g_t0_sim_set  = true;
        }

        // Convert Gazebo time to wall time
        msg::Timestamp t = gazebo_helper::sim_to_walltime(msg.header().stamp().sec(), msg.header().stamp().nsec());

        // Construct the AltimeterDataMsg from the received Gazebo message.
        msg::AltimeterDataMsg alt_data = {
            .header            = {.seq      = extracted_header.seq,
                                  .stamp    = {.sec = t.sec, .nsec = t.nsec},
                                  .frame_id = extracted_header.frame_id},
            .vertical_position  = msg.vertical_position(),
            .vertical_velocity  = msg.vertical_velocity(),
            .vertical_reference = msg.vertical_reference(),
            .covariance = linalg::Matrix<3, 3>::identity()
        };

        if (altimeter_callback)
        {
          altimeter_callback(alt_data);
        }
      });
}

/**
 * @brief Stops the altimeter data stream.
 *
 * This function halts altimeter data collection.
 */
void Altimeter::stop()
{
  running = false;
}
