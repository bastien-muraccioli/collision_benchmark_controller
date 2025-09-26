#include "CollisionBenchmarkController_Initial.h"

void CollisionBenchmarkController_Initial::configure(const mc_rtc::Configuration & config) {}

void CollisionBenchmarkController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  // ctl.datastore().assign<std::string>("ModeState", "PositionTransition");
  // ctl.isTorqueControl = false;
  ctl.compPostureTask->stiffness(1);
  ctl.compPostureTask->damping(2);
  ctl.compPostureTask->target(ctl.postureHome);
  // ctl.compPostureTask->makeCompliant(false);
  ctl.solver().removeTask(ctl.compEETask);

  // ctl.datastore().assign<std::string>("ControlMode", "Position");
  task_achieved_ = false;
}

bool CollisionBenchmarkController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  ctl.datastore().assign<std::string>("State", "Initial");
  if(ctl.datastore().has("Obstacle detected"))
  {
    if(ctl.datastore().get<bool>("Obstacle detected"))
    {
      ctl.datastore().get<bool>("Obstacle detected") = false;
    }
  }
  if(ctl.compPostureTask->eval().norm() < 0.05 && !task_achieved_)
  {
    task_achieved_ = true;
    mc_rtc::log::success("[CollisionBenchmarkController] Get back to initial posture");
  }

  isTorqueControl = ctl.isTorqueControl;
  if(isTorqueControl != controlModeRequest)
  {
    controlModeRequest = isTorqueControl;
    if(isTorqueControl)
    {
      ctl.compPostureTask->reset();
      ctl.compPostureTask->stiffness(100);
    }
    else
    {
      ctl.compPostureTask->reset();
      ctl.compPostureTask->stiffness(1);
      ctl.compPostureTask->damping(2);
    }
  }

  // output("OK");
  // return true;
  return false;
}

void CollisionBenchmarkController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
}

EXPORT_SINGLE_STATE("CollisionBenchmarkController_Initial", CollisionBenchmarkController_Initial)
