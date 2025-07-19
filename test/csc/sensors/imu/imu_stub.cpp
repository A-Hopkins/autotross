#include "csc/sensors/imu/imu.h"

#include <unordered_map>
#include <deque>

std::unordered_map<uint16_t, std::deque<msg::IMUDataMsg>> imu_data_queues;

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
}
void IMU::stop()
{ 
  status = Status::UNINITIALIZED;
}

msg::IMUDataMsg IMU::get_current_data() const
{
  auto& q = imu_data_queues[imu_id];
  if (!q.empty()) {
    msg::IMUDataMsg result = q.front();
    q.pop_front();
    q.push_back(result); // Optional: rotate to loop
    return result;
  }
  return current_imu_data;  // fallback
}

void inject_imu_data_sequence(uint16_t id, const std::deque<msg::IMUDataMsg>& data)
{
  imu_data_queues[id] = data;
}
