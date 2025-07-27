#include "csc/sensors/imu/imu.h"

#include <unordered_map>
#include <deque>

std::unordered_map<uint16_t, std::deque<IMU::IMUMetaData>> imu_data_queues;

IMU::IMU(uint16_t id) : status(Status::UNINITIALIZED), imu_id(id), current_imu_data{id}
{
}
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
 * @brief Gets the current IMU status.
 * 
 * This function returns the current status of the IMU sensor, indicating the validity of the data.
 */
IMU::Status IMU::get_status() const
{
  return status;
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
  status = Status::VALID;
  initialized = true;
  running = true;
  recovery_pass_count = 0;
  recovery_fail_count = 0;
  stale_read_count = 0;
}
void IMU::stop()
{ 
  status = Status::UNINITIALIZED;
}

IMU::IMUMetaData IMU::get_current_data() const
{
  IMUMetaData meta;
  meta.status = status;
  meta.recovery_pass_count = recovery_pass_count;
  meta.imu_data = current_imu_data;

  auto& queue = imu_data_queues[imu_id];
  if (!queue.empty())
  {
    IMUMetaData next = queue.front();
    queue.pop_front();
    queue.push_back(next); // Optional rotation
    meta = next;
  }

  return meta;
}

void inject_imu_data_sequence(uint16_t id, const std::deque<IMU::IMUMetaData>& data)
{
  imu_data_queues[id] = data;
}