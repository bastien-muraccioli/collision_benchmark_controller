#include "CollisionBenchmarkController_BigRotate_InitialPose.h"

void CollisionBenchmarkController_BigRotate_InitialPose::configure(const mc_rtc::Configuration & config) {}

void CollisionBenchmarkController_BigRotate_InitialPose::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  // ctl.datastore().assign<std::string>("ControlMode", "Position");
  ctl.compPostureTask->target(ctl.postureBigRotate);
  ctl.compPostureTask->stiffness(1);
  ctl.compPostureTask->damping(2);
}

bool CollisionBenchmarkController_BigRotate_InitialPose::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  // ctl.datastore().assign<std::string>("State", "BigRotate_InitialPose");

  if (ctl.compPostureTask->eval().norm() < 0.1)
  {
    output("OK");
    return true;
  }

  return false;
}

void CollisionBenchmarkController_BigRotate_InitialPose::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
}

EXPORT_SINGLE_STATE("CollisionBenchmarkController_BigRotate_InitialPose", CollisionBenchmarkController_BigRotate_InitialPose)
