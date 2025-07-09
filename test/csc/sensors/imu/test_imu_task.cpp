#include "csc/sensors/imu/imu_task.h"
#include "protocore/include/broker.h"
#include <gtest/gtest.h>

class IMUTaskTest : public ::testing::Test
{
protected:
  std::shared_ptr<IMUTask> task;

  void SetUp() override
  {
    // Initialize the publisher and subscriber broker
    Broker::initialize();
    task = IMUTask::create();
    task->start(); // Start the task to transition it to RUNNING state
  }
};
