#include "csc/sensors/imu/imu_task.h"
#include "protocore/include/state_manager.h"
#include "protocore/include/broker.h"
#include <gtest/gtest.h>
#include <deque>


void inject_imu_data_sequence(uint16_t id, const std::deque<IMU::IMUMetaData>& data);

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
  constexpr int64_t max_allowed_duration_us = 500; // .5 ms budget

  task->transition_to_state(task::TaskState::RUNNING);

  // First: stable inputs for initial VALID state
  std::deque<IMU::IMUMetaData> stable(1);
  for (auto& m : stable)
  {
    m.imu_data.angular_velocity = {1.0, 1.0, 1.0};
    m.imu_data.angular_velocity = {1.0, 1.0, 1.0};
    m.imu_data.linear_acceleration = {0.1, 0.1, 0.1};
    m.imu_data.orientation = {0.0, 0.0, 0.0, 1.0};
    m.status = IMU::Status::VALID;
  }

  // Load all sensors initially with valid data
  inject_imu_data_sequence(1, stable);
  inject_imu_data_sequence(2, stable);
  inject_imu_data_sequence(3, stable);

  auto start = std::chrono::high_resolution_clock::now();
  task->process_imu_data();
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

TEST_F(IMUTaskTest, DegradeThenRecover)
{
  task->transition_to_state(task::TaskState::RUNNING);

  // First: stable inputs for initial VALID state
  std::deque<IMU::IMUMetaData> stable(3);
  for (int i = 0; i < 3; ++i)
  {
    IMU::IMUMetaData m;
    m.imu_data.angular_velocity = {1.0 + i * 0.001, 1.0, 1.0}; // subtle variation to avoid stale
    m.imu_data.linear_acceleration = {0.1, 0.1, 0.1};
    m.imu_data.orientation = {0.0, 0.0, 0.0, 1.0};
    m.status = IMU::Status::VALID;
    stable[i] = m;
  }

  // Outlier sequence for sensor 2 to cause degradation
  std::deque<IMU::IMUMetaData> faulty = stable;
  for (int i = 0; i < 3; ++i)
  {
    IMU::IMUMetaData m;
    m.imu_data.angular_velocity = {20.0 + i * 0.001, 20.0, 20.0}; // subtle variation to avoid stale
    m.imu_data.linear_acceleration = {0.1, 0.1, 0.1};
    m.imu_data.orientation = {0.0, 0.0, 0.0, 1.0};
    m.status = IMU::Status::VALID;
    faulty[i] = m;
  }

  std::deque<IMU::IMUMetaData> now_valid_data = stable;
  for (auto& m : now_valid_data)
  {
    m.status = IMU::Status::DEGRADED;
  }

  // Load all sensors initially with valid data
  inject_imu_data_sequence(1, stable);
  inject_imu_data_sequence(2, stable);
  inject_imu_data_sequence(3, stable);

  task->process_imu_data(); // Should all be VALID

  // Cause degradation
  inject_imu_data_sequence(3, faulty);
  task->process_imu_data();

  EXPECT_EQ(task->imu_sensors[2].get_status(), IMU::Status::DEGRADED);

  // Recover: valid again
  inject_imu_data_sequence(3, now_valid_data);
  for (int i = 0; i < 3; ++i)
  {
    task->process_imu_data();
  }

  EXPECT_EQ(task->imu_sensors[2].get_status(), IMU::Status::VALID);
}

TEST_F(IMUTaskTest, StartDegradedThenRecover)
{
  task->transition_to_state(task::TaskState::RUNNING);

  task->imu_sensors[1].set_status(IMU::Status::DEGRADED);
  task->imu_sensors[1].set_recovery_pass_count(0);
  task->imu_sensors[1].set_recovery_fail_count(0);


  std::deque<IMU::IMUMetaData> valid_data(5);
  std::deque<IMU::IMUMetaData> now_valid_data(5);
  for (int i = 0; i < 5; ++i)
  {
    IMU::IMUMetaData m;
    m.imu_data.angular_velocity = {1.0 + i * 0.001, 1.0, 1.0}; // subtle variation to avoid stale
    m.imu_data.linear_acceleration = {0.1, 0.1, 0.1};
    m.imu_data.orientation = {0.0, 0.0, 0.0, 1.0};
    m.status = IMU::Status::VALID;
    valid_data[i] = m;
  }
  
  for (int i = 0; i < 5; ++i)
  {
    IMU::IMUMetaData m;
    m.imu_data.angular_velocity = {1.0 + i * 0.001, 1.0, 1.0}; // subtle variation to avoid stale
    m.imu_data.linear_acceleration = {0.1, 0.1, 0.1};
    m.imu_data.orientation = {0.0, 0.0, 0.0, 1.0};
    m.status = IMU::Status::DEGRADED;
    now_valid_data[i] = m;
  }

  inject_imu_data_sequence(1, valid_data);
  inject_imu_data_sequence(2, now_valid_data);
  inject_imu_data_sequence(3, valid_data);

  // Let it recover
  for (int i = 0; i < 3; ++i)
  {
    task->process_imu_data();
  }

  EXPECT_EQ(task->imu_sensors[1].get_status(), IMU::Status::VALID);
}

TEST_F(IMUTaskTest, StartDegradedThenInvalid)
{
  task->transition_to_state(task::TaskState::RUNNING);

  task->imu_sensors[2].set_status(IMU::Status::DEGRADED);
  task->imu_sensors[2].set_recovery_pass_count(0);
  task->imu_sensors[2].set_recovery_fail_count(0);


  std::deque<IMU::IMUMetaData> valid_data(5);
  for (int i = 0; i < 5; ++i)
  {
    IMU::IMUMetaData m;
    m.imu_data.angular_velocity = {1.0 + i * 0.001, 1.0, 1.0}; // subtle variation to avoid stale
    m.imu_data.linear_acceleration = {0.1, 0.1, 0.1};
    m.imu_data.orientation = {0.0, 0.0, 0.0, 1.0};
    m.status = IMU::Status::VALID;
    valid_data[i] = m;
  }

  std::deque<IMU::IMUMetaData> faulty_data(5);
  for (int i = 0; i < 5; ++i)
  {
    IMU::IMUMetaData m;
    m.imu_data.angular_velocity = {100.0 + i * 0.001, 100.0, 100.0}; // subtle variation to avoid stale
    m.imu_data.linear_acceleration = {0.1, 0.1, 0.1};
    m.imu_data.orientation = {0.0, 0.0, 0.0, 1.0};
    m.status = IMU::Status::DEGRADED;
    faulty_data[i] = m;
  }

  inject_imu_data_sequence(1, valid_data);
  inject_imu_data_sequence(2, valid_data);
  inject_imu_data_sequence(3, faulty_data);

  for (int i = 0; i < 5; ++i)
  {
    task->process_imu_data();
  }

  EXPECT_EQ(task->imu_sensors[2].get_status(), IMU::Status::INVALID);
}

TEST_F(IMUTaskTest, StaleSensorDetection)
{
  task->transition_to_state(task::TaskState::RUNNING);

  std::deque<IMU::IMUMetaData> stable(5);
  for (int i = 0; i < 5; ++i)
  {
    IMU::IMUMetaData m;
    m.imu_data.angular_velocity = {1.0 + i * 0.001, 1.0, 1.0}; // prevent stale
    m.imu_data.linear_acceleration = {0.1, 0.1, 0.1};
    m.imu_data.orientation = {0.0, 0.0, 0.0, 1.0};
    m.status = IMU::Status::VALID;
    stable[i] = m;
  }

  std::deque<IMU::IMUMetaData> stale(5);
  for (auto& m : stale)
  {
    m.imu_data.angular_velocity = {2.0, 2.0, 2.0};
    m.imu_data.linear_acceleration = {0.2, 0.2, 0.2};
    m.imu_data.orientation = {0.0, 0.0, 0.0, 1.0};
    m.status = IMU::Status::VALID;
  }

  inject_imu_data_sequence(1, stable);
  inject_imu_data_sequence(2, stable);
  inject_imu_data_sequence(3, stale); // sensor 3 will go stale

  for (int i = 0; i < 5; ++i)
  {
    task->process_imu_data();
  }

  EXPECT_EQ(task->imu_sensors[2].get_status(), IMU::Status::INVALID);
}