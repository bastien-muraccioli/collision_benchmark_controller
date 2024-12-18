#include "CircularOriController_Initial.h"

#include "../CircularOriController.h"

void CircularOriController_Initial::configure(const mc_rtc::Configuration & config) {}

void CircularOriController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CircularOriController &>(ctl_);
  // ctl.compPostureTask->stiffness(100.0);
  ctl.compPostureTask->stiffness(0.5);
  ctl.compPostureTask->target(ctl.postureHome);
  ctl.compPostureTask->makeCompliant(false);
  ctl.solver().removeTask(ctl.compEETask);

  ctl.datastore().assign<std::string>("ControlMode", "Position");

  if(!ctl.plot_initialized)
  {
    // Show the plots
    ctl.datastore().call("ObstacleDetectionJerkEstimator::ResetPlot");
    ctl.plot_initialized = true;
  }
  task_achieved_ = false;
}

bool CircularOriController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CircularOriController &>(ctl_);
  ctl.datastore().assign<std::string>("State", "Initial");
  if(ctl.compPostureTask->eval().norm() < 0.05 && !task_achieved_)
  {
    task_achieved_ = true;
    mc_rtc::log::success("[CircularOriController] Get back to initial posture");
  }
  // output("OK");
  // return true;
  return false;
}

void CircularOriController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CircularOriController &>(ctl_);
}

EXPORT_SINGLE_STATE("CircularOriController_Initial", CircularOriController_Initial)
