#include "CollisionBenchmarkController_BirjandiRotateHorizontal_InitialPose.h"

void CollisionBenchmarkController_BirjandiRotateHorizontal_InitialPose::configure(const mc_rtc::Configuration & config) {}

void CollisionBenchmarkController_BirjandiRotateHorizontal_InitialPose::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  // ctl.isTorqueControl = false;
  ctl.compPostureTask->target(ctl.postureHorizontalStart);

  ctl.compPostureTask->stiffness(1);
  ctl.compPostureTask->damping(2);
}

bool CollisionBenchmarkController_BirjandiRotateHorizontal_InitialPose::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  // ctl.datastore().assign<std::string>("State", "Rotate");

  if (ctl.compPostureTask->eval().norm() < 0.1)
  {
    output("OK");
    return true;
  }


  return false;
}

void CollisionBenchmarkController_BirjandiRotateHorizontal_InitialPose::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
}

EXPORT_SINGLE_STATE("CollisionBenchmarkController_BirjandiRotateHorizontal_InitialPose", CollisionBenchmarkController_BirjandiRotateHorizontal_InitialPose)
