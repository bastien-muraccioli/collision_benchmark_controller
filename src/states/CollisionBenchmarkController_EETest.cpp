#include "CollisionBenchmarkController_EETest.h"

void CollisionBenchmarkController_EETest::configure(const mc_rtc::Configuration & config) {}

void CollisionBenchmarkController_EETest::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  ctl.datastore().assign<std::string>("ControlMode", "Position");
  ctl.postureTask->target(ctl.postureHome);
  ctl.postureTask->stiffness(0.5);
  ctl.postureTask->weight(0.1);
  ctl.eeTask->reset();
  ctl.eeTask->positionTask->position(ctl.taskPosHome);
  ctl.eeTask->positionTask->stiffness(100);
  ctl.eeTask->orientationTask->orientation(ctl.taskOriHome.toRotationMatrix());
  ctl.eeTask->orientationTask->stiffness(100);
  ctl.eeTask->positionTask->weight(10000);

  ctl.solver().addTask(ctl.eeTask);
}

bool CollisionBenchmarkController_EETest::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  if(ctl.datastore().get<bool>("Obstacle detected"))
  {
    ctl.postureTask->reset();
    ctl.eeTask->reset();
    ctl.postureTask->stiffness(500);
    output("OK");
    return true;
  }
  return false;
}

void CollisionBenchmarkController_EETest::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  ctl.solver().removeTask(ctl.eeTask);
}

EXPORT_SINGLE_STATE("CollisionBenchmarkController_EETest", CollisionBenchmarkController_EETest)
