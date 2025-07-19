#include "csc/sensors/imu/imu_task.h"
#include "protocore/include/state_manager.h"
#include "protocore/include/broker.h"
#include <gtest/gtest.h>
#include <deque>


void inject_imu_data_sequence(uint16_t id, const std::deque<msg::IMUDataMsg>& data);

class IMUTaskTest : public ::testing::Test
{
protected:
  std::shared_ptr<IMUTask> task;
  std::shared_ptr<StateManager> state_manager;

  void SetUp() override
  {
    Broker::initialize();
    task = IMUTask::create();
    state_manager = StateManager::create();
    state_manager->register_task(task);
  }

  void TearDown() override
  {
    state_manager->request_state_transition(task::TaskState::STOPPED);
  }
};

TEST_F(IMUTaskTest, TransitionsToRunningStartsAllSensors)
{
  task->transition_to_state(task::TaskState::RUNNING);

  for (const auto& imu : task->imu_sensors)
  {
    EXPECT_EQ(imu.get_status(), IMU::Status::VALID);
  }
}

TEST_F(IMUTaskTest, TransitionsToStopInvalidatesAllSensors)
{
  task->transition_to_state(task::TaskState::STOPPED);

  for (const auto& imu : task->imu_sensors)
  {
    EXPECT_EQ(imu.get_status(), IMU::Status::UNINITIALIZED);
  }
}

TEST_F(IMUTaskTest, VotingAllSensorsAgree)
{
  std::array<msg::IMUDataMsg, IMUTask::IMU_COUNT> data;
  for (size_t i = 0; i < IMUTask::IMU_COUNT; ++i)
  {
    data[i].angular_velocity = {1.0, 1.0, 1.0};
    data[i].linear_acceleration = {0.1, 0.1, 0.1};
    data[i].orientation = {0.0, 0.0, 0.0, 1.0};
  }

  std::array<bool, IMUTask::IMU_COUNT> active;
  active.fill(true);

  auto valid_imus = task->vote_valid_imus(data, active);

  for (bool valid_imu : valid_imus)
  {
    EXPECT_TRUE(valid_imu);
  }
}

TEST_F(IMUTaskTest, VotingRejectsOutlier)
{
  std::array<msg::IMUDataMsg, IMUTask::IMU_COUNT> data;

  data[0].angular_velocity = {1.0, 1.0, 1.0};
  data[1].angular_velocity = {1.01, 1.0, 1.0}; // slight noise
  data[2].angular_velocity = {20.0, 20.0, 20.0}; // far off

  std::array<bool, IMUTask::IMU_COUNT> active;
  active.fill(true);

  auto valid = task->vote_valid_imus(data, active);

  EXPECT_TRUE(valid[0]);
  EXPECT_TRUE(valid[1]);
  EXPECT_FALSE(valid[2]);
}

TEST_F(IMUTaskTest, VotingAllDisagreeReturnsNone)
{
  std::array<msg::IMUDataMsg, IMUTask::IMU_COUNT> data;

  data[0].angular_velocity = {1.0, 0.0, 0.0};
  data[1].angular_velocity = {5.0, 5.0, 5.0};
  data[2].angular_velocity = {-5.0, -5.0, -5.0};

  std::array<bool, IMUTask::IMU_COUNT> active;
  active.fill(true);

  auto valid = task->vote_valid_imus(data, active);

  for (bool v : valid)
  {
    EXPECT_FALSE(v);
  }
}

TEST_F(IMUTaskTest, VoteValidIMUsPerformance)
{
  constexpr int64_t max_allowed_duration_us = 1000; // 1 ms budget

  std::array<msg::IMUDataMsg, IMUTask::IMU_COUNT> data{};
  std::array<bool, IMUTask::IMU_COUNT> active;
  active.fill(true);
  for (size_t i = 0; i < IMUTask::IMU_COUNT; ++i)
  {
    data[i].angular_velocity(0) = 1.0;
  }

  auto start = std::chrono::high_resolution_clock::now();
  auto valid = task->vote_valid_imus(data, active);
  auto end = std::chrono::high_resolution_clock::now();

  auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

  EXPECT_LT(duration_us, max_allowed_duration_us)
      << "vote_valid_imus exceeded time budget: " << duration_us << " us";

}

TEST_F(IMUTaskTest, SingleActiveSensorIsAlwaysValidIfClusterOfOne)
{
  std::array<msg::IMUDataMsg, IMUTask::IMU_COUNT> data;
  std::array<bool, IMUTask::IMU_COUNT> active{};
  data[1].angular_velocity = {1.0, 1.0, 1.0};
  active[1] = true;

  auto valid = task->vote_valid_imus(data, active);

  EXPECT_FALSE(valid[0]);
  EXPECT_TRUE(valid[1]);
  EXPECT_FALSE(valid[2]);
}

TEST_F(IMUTaskTest, SimulateDriftingIMU)
{
  task->transition_to_state(task::TaskState::RUNNING);

  std::deque<msg::IMUDataMsg> stable, faulty;

  for (int i = 0; i < 3; ++i)
  {
    msg::IMUDataMsg d;
    d.angular_velocity = {1.0, 1.0, 1.0};
    stable.push_back(d);
  }

  faulty = stable;
  msg::IMUDataMsg bad;
  bad.angular_velocity = {10.0, 10.0, 10.0};
  faulty[2] = bad;

  inject_imu_data_sequence(1, stable);
  inject_imu_data_sequence(2, stable);
  inject_imu_data_sequence(3, faulty);

  task->process_imu_data();
  task->process_imu_data();
  task->process_imu_data();

}

TEST_F(IMUTaskTest, CompareIMUThresholdEdge)
{
  msg::IMUDataMsg a, b;
  a.angular_velocity = {1.0, 1.0, 1.0};
  b.angular_velocity = {1.0 + IMUTask::ANGULAR_VEL_THRESHOLD - 1e-6, 1.0, 1.0};

  bool agree = task->compare_imu(a, b);
  EXPECT_TRUE(agree);

  b.angular_velocity = {1.0 + IMUTask::ANGULAR_VEL_THRESHOLD + 1e-6, 1.0, 1.0};
  agree = task->compare_imu(a, b);
  EXPECT_FALSE(agree);
}
