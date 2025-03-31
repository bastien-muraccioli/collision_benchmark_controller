#include "CollisionBenchmarkController_Initial.h"

#include "../CollisionBenchmarkController.h"

void CollisionBenchmarkController_Initial::configure(const mc_rtc::Configuration & config) {}

void CollisionBenchmarkController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  // ctl.compPostureTask->stiffness(100.0);
  ctl.compPostureTask->stiffness(0.5);
  ctl.compPostureTask->target(ctl.postureHome);
  ctl.compPostureTask->makeCompliant(false);
  ctl.solver().removeTask(ctl.compEETask);

  ctl.datastore().assign<std::string>("ControlMode", "Position");
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
  // output("OK");
  // return true;
  return false;
}

void CollisionBenchmarkController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
}

EXPORT_SINGLE_STATE("CollisionBenchmarkController_Initial", CollisionBenchmarkController_Initial)
