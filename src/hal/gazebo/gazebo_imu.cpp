#include "csc/sensors/imu/imu.h"
#include "gazebo_helpers.h"
#include "msg/imu_msg.h"
#include <gz/msgs.hh>
#include <gz/transport.hh>

/** @brief Gazebo transport node for communication. */
static gz::transport::Node node;

/** @brief Flag indicating if the IMU data processing is active. */
static bool running = false;

/**
 * @brief Construct a new IMU object.
 *
 * Initializes the Gazebo IMU interface.
 */
IMU::IMU(uint16_t imu_id) : status(Status::UNINITIALIZED), imu_id(imu_id) {}

/**
 * @brief Gets the unique identifier for this IMU instance.
 *
 * This function returns the unique identifier assigned to this IMU sensor instance.
 * The ID is typically used to distinguish between multiple IMU sensors in a system.
 *
 */
uint16_t IMU::get_id() const
{
  return imu_id;
}

/**
 * @brief Reads the current IMU data.
 *
 * This function retrieves the most recent IMU data from the sensor. The actual data retrieval
 * mechanism is determined by the implementation (e.g., reading from hardware registers, fetching
 * from a simulation environment).
 */
IMU::IMUMetaData IMU::get_current_data() const
{
  IMUMetaData data;
  data.imu_data = current_imu_data;
  data.status = status;
  data.recovery_pass_count = recovery_pass_count;
  return data;
}

/**
 * @brief Gets the current IMU status.
 * 
 * This function returns the current status of the IMU sensor, indicating the validity of the data.
 */
IMU::Status IMU::get_status() const
{
  return status;
}

/**
 * @brief Gets the number of recovery passes.
 *
 * This function returns the number of recovery attempts made by the IMU sensor.
 * It is used to track how many times the sensor has tried to recover from an invalid state.
 */
uint8_t IMU::get_recovery_pass_count() const
{
  return recovery_pass_count;
}

/**
 * @brief Sets the number of recovery passes.
 *
 * This function updates the number of recovery attempts made by the IMU sensor.
 *
 * @param count The new count of recovery passes.
 */
void IMU::set_recovery_pass_count(uint8_t count)
{
  recovery_pass_count = count;
}

/**
 * @brief Gets the number of recovery fails.
 *
 * This function returns the number of recovery failures made by the IMU sensor.
 * It is used to track how many times the sensor has failed to recover from a degraded state.
 */
uint8_t IMU::get_recovery_fail_count() const
{
  return recovery_fail_count;
}

/**
 * @brief Sets the number of recovery fails.
 *
 * This function updates the number of recovery failures made by the IMU sensor.
 *
 * @param count The new count of recovery fails.
 */
void IMU::set_recovery_fail_count(uint8_t count)
{
  recovery_fail_count = count;
}

/**
 * @brief Gets the number of stale reads.
 *
 * This function returns the number of times the IMU data has been considered stale.
 * It is used to track how many times the sensor has not provided fresh data.
 */
uint8_t IMU::get_stale_read_count() const
{
  return stale_read_count;
}

/**
 * @brief Gets the number of stale reads.
 *
 * This function returns the number of times the IMU data has been considered stale.
 * It is used to track how many times the sensor has not provided fresh data.
 */
void IMU::set_stale_read_count(uint8_t count)
{
  stale_read_count = count;
}

/**
 * @brief Sets the IMU status.
 *
 * This function updates the current status of the IMU sensor.
 *
 * @param new_status The new status to set for the IMU sensor.
 */
void IMU::set_status(IMU::Status new_status)
{
  status = new_status;
}

void IMU::start()
{
  running = true;
  status = Status::VALID;

  node.Subscribe<gz::msgs::IMU>(
      "/sensors/imu_" + std::to_string(imu_id),
      [this](const gz::msgs::IMU &msg)
      {
        // Ignore messages if not running
        if (!running)
          return;

        // Extract header information using the helper function
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
        msg::Timestamp t =
            gazebo_helper::sim_to_walltime(msg.header().stamp().sec(), msg.header().stamp().nsec());

        // Convert Gazebo IMU message to internal IMUDataMsg format
        msg::IMUDataMsg imu_data = {
            .header      = {.seq      = extracted_header.seq,
                            .stamp    = {.sec = t.sec, .nsec = t.nsec},
                            .frame_id = extracted_header.frame_id},
            .sensor_id   = imu_id,
            .orientation = {msg.orientation().x(), msg.orientation().y(), msg.orientation().z(),
                            msg.orientation().w()},
            .orientation_covariance =
                {{msg.orientation_covariance().data()[0], msg.orientation_covariance().data()[1],
                  msg.orientation_covariance().data()[2]},
                 {msg.orientation_covariance().data()[3], msg.orientation_covariance().data()[4],
                  msg.orientation_covariance().data()[5]},
                 {msg.orientation_covariance().data()[6], msg.orientation_covariance().data()[7],
                  msg.orientation_covariance().data()[8]}},
            .angular_velocity            = {msg.angular_velocity().x(), msg.angular_velocity().y(),
                                            msg.angular_velocity().z()},
            .angular_velocity_covariance = {{msg.angular_velocity_covariance().data()[0],
                                             msg.angular_velocity_covariance().data()[1],
                                             msg.angular_velocity_covariance().data()[2]},
                                            {msg.angular_velocity_covariance().data()[3],
                                             msg.angular_velocity_covariance().data()[4],
                                             msg.angular_velocity_covariance().data()[5]},
                                            {msg.angular_velocity_covariance().data()[6],
                                             msg.angular_velocity_covariance().data()[7],
                                             msg.angular_velocity_covariance().data()[8]}},
            .linear_acceleration = {msg.linear_acceleration().x(), msg.linear_acceleration().y(),
                                    msg.linear_acceleration().z()},
            .linear_acceleration_covariance = {{msg.linear_acceleration_covariance().data()[0],
                                                msg.linear_acceleration_covariance().data()[1],
                                                msg.linear_acceleration_covariance().data()[2]},
                                               {msg.linear_acceleration_covariance().data()[3],
                                                msg.linear_acceleration_covariance().data()[4],
                                                msg.linear_acceleration_covariance().data()[5]},
                                               {msg.linear_acceleration_covariance().data()[6],
                                                msg.linear_acceleration_covariance().data()[7],
                                                msg.linear_acceleration_covariance().data()[8]}}};

        // Update the current IMU data
        current_imu_data = imu_data;
      }
  );
}

/**
 * @brief Stops the IMU data processing.
 *
 * Sets the running flag to false, preventing further processing of
 * incoming Gazebo messages in the subscription callback. Does not
 * explicitly unsubscribe from the topic.
 */
void IMU::stop()
{
  running = false;
}
