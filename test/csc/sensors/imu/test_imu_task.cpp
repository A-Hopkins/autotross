#include "csc/sensors/imu/imu_task.h"
#include "protocore/include/broker.h"
#include "protocore/include/state_manager.h"
#include <gtest/gtest.h>


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
    state_manager->initialize();
  }

  void TearDown() override
  {
    state_manager->shutdown();
  }
};

TEST_F(IMUTaskTest, PeriodicityIsRoughlyCorrect)
{
  using namespace std::chrono;
  state_manager->request_state_transition(task::TaskState::RUNNING);

  auto start = steady_clock::now();
  std::this_thread::sleep_for(5ms); // Give it time to run some cycles
  auto end = steady_clock::now();
  auto elapsed = duration_cast<milliseconds>(end - start).count();

}