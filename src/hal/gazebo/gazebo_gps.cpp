/**
 * @file gazebo_gps.cpp
 * @brief Implementation of the GPS class for interfacing with Gazebo NAVSAT data.
 *
 * This file provides the implementation for subscribing to Gazebo's NAVSAT topic,
 * processing the received messages, and invoking a user-defined callback with
 * the processed gps data.
 */
#include "msg/gps_msg.h"
#include "csc/sensors/gps/gps.h"
#include "gazebo_helpers.h"

#include <gz/msgs.hh>
#include <gz/transport.hh>

//// Gazebo transport node for communication.
static gz::transport::Node node;

/// Callback function to be invoked when new gps data is received.
static std::function<void(const msg::GPSDataMsg&)> gps_callback;

/**
 * @brief Constructs an GPS interface.
 *
 * Initializes the GPS interface for gazebo implementation.
 * 
 */
GPS::GPS() : running(false) { }

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
void GPS::start(std::function<void(const msg::GPSDataMsg&)> callback)
{
  gps_callback = callback;
  running = true;

  // Subscribe to the GPS topic in Gazebo
  node.Subscribe<gz::msgs::NavSat>(
      "/sensors/navsat",
      [this](const gz::msgs::NavSat &msg)
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

        // Construct the GpsDataMsg from the received Gazebo message.
        msg::GPSDataMsg gps_data = {
            .header    = {.seq      = extracted_header.seq,
                          .stamp    = {.sec = t.sec, .nsec = t.nsec},
                          .frame_id = extracted_header.frame_id},
            .latitude  = msg.latitude_deg(),
            .longitude = msg.longitude_deg(),
            .altitude  = msg.altitude(),
            .position_covariance = linalg::Matrix<3, 3>::identity(),
            .velocity = {msg.velocity_east(), msg.velocity_north(), msg.velocity_up()},
            .velocity_covariance = linalg::Matrix<3, 3>::identity()
        };

        if (gps_callback)
        {
          gps_callback(gps_data);
        }
      });
}

/**
 * @brief Stops the GPS data stream.
 *
 * This function halts GPS data collection.
 */
void GPS::stop()
{
  running = false;
}
