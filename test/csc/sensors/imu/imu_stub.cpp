#include "csc/sensors/imu/imu.h"

IMU::IMU(uint16_t id) : status(Status::UNINITIALIZED), imu_id(id), current_imu_data{id}
{

}
void IMU::start()
{
  status = Status::VALID;
}
void IMU::stop()
{ 
  status = Status::UNINITIALIZED;
}
