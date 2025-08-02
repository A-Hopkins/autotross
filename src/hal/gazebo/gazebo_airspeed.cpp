/**
 * @file gazebo_airspeed.cpp
 * @brief Implementation of the Airspeed class for interfacing with Gazebo Airspeed data.
 *
 * This file provides the implementation for subscribing to Gazebo's Airspeed topic,
 * processing the received messages, and invoking a user-defined callback with
 * the processed airspeed data.
 */
#include "msg/airspeed_msg.h"
#include "csc/sensors/airspeed/airspeed.h"
#include "gazebo_helpers.h"

#include <gz/msgs.hh>
#include <gz/transport.hh>

//// Gazebo transport node for communication.
static gz::transport::Node node;

/// Callback function to be invoked when new airspeed data is received.
static std::function<void(const msg::AirspeedDataMsg&)> airspeed_callback;

/**
 * @brief Constructs an Airspeed interface.
 *
 * Initializes the Airspeed interface for gazebo implementation.
 */
Airspeed::Airspeed() : running(false) { }

/**
 * @brief Starts the Airspeed data stream and registers a callback function to receive data.
 *
 * This function initiates Airspeed data collection and calls the provided callback whenever new
 * data is available. The actual data retrieval mechanism depends on the implementation (hardware
 * polling, event-driven updates, etc.).
 *
 * @param callback A function that receives Airspeed data as a `msg::AirspeedDataMsg`, representing
 * sensor readings. The callback function will be invoked asynchronously whenever new Airspeed
 * data arrives. It is crucial that the callback function is thread-safe if the underlying
 * implementation uses multiple threads. The `msg::AirspeedDataMsg` object passed to the callback
 * contains the latest Airspeed data including velocity and other relevant measurements.
 */
void Airspeed::start(std::function<void(const msg::AirspeedDataMsg&)> callback)
{
  airspeed_callback = callback;
  running = true;

  // Subscribe to the Airspeed topic in Gazebo
  node.Subscribe<gz::msgs::AirSpeed>(
      "/sensors/airspeed",
      [this](const gz::msgs::AirSpeed &msg)
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

        // Construct the AirspeedDataMsg from the received Gazebo message.
        msg::AirspeedDataMsg airspeed_data = {
            .header    = {.seq      = extracted_header.seq,
                          .stamp    = {.sec = t.sec, .nsec = t.nsec},
                          .frame_id = extracted_header.frame_id},
            .differential_pressure = msg.diff_pressure(),
            .temperature = msg.temperature(),
            .true_airspeed = 0.0, // Placeholder, will be computed after populating covariance of measured values
            .covariance = linalg::Matrix<3, 3>::identity()
        };

        auto [tas, tas_var] = compute_true_airspeed(airspeed_data.differential_pressure, airspeed_data.temperature, airspeed_data.covariance(0,0), airspeed_data.covariance(1,1));
        airspeed_data.true_airspeed = tas;
        airspeed_data.covariance(2, 2) = tas_var;

        if (airspeed_callback)
        {
          airspeed_callback(airspeed_data);
        }
      });
}

/**
 * @brief Stops the Airspeed data stream.
 *
 * This function halts Airspeed data collection.
 */
void Airspeed::stop()
{
  running = false;
}

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
std::tuple<double, double> Airspeed::compute_true_airspeed(double diff_pressure,
                                                           double temp,
                                                           double pressure_variance,
                                                           double temp_variance)
{
  // Constants
  constexpr double R = 287.05;                      // [J/(kg*K)]
  constexpr double P_STATIC = 101325.0;             // [Pa] standard atmospheric pressure
  constexpr double MIN_DIFFERENTIAL_PRESSURE = 0.1; // [Pa]


  // Handle invalid differential pressure gracefully
  if (diff_pressure <= MIN_DIFFERENTIAL_PRESSURE)
  {
    return {0.0, 0.0};
  }


  // Compute air density
  double rho = P_STATIC / (R * temp);

  // Compute true airspeed
  double v = std::sqrt(2.0 * diff_pressure / rho);

  // Error propagation: dv/dp and dv/dT
  double dv_dp = 1.0 / (std::sqrt(2.0 * diff_pressure / rho)) * (1.0 / rho);
  double dv_dT = 1.0 / (std::sqrt(2.0 * diff_pressure / rho)) * (diff_pressure * P_STATIC) / (std::pow(rho, 3) * R * std::pow(temp, 2));

  // Propagate variance
  double v_var = dv_dp * dv_dp * pressure_variance + dv_dT * dv_dT * temp_variance;

  return {v, v_var};
}
