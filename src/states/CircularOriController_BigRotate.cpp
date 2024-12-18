#include "CircularOriController_BigRotate.h"

// #include <obstacle_detection_jerk_estimator/ObstacleDetectionJerkEstimator.h>

#include "../CircularOriController.h"

void CircularOriController_BigRotate::configure(const mc_rtc::Configuration & config) {}

void CircularOriController_BigRotate::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CircularOriController &>(ctl_);
  // Activate feedback from external forces estimator (safer)
  if(!ctl.datastore().call<bool>("EF_Estimator::isActive"))
  {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  // Activate force sensor usage if not used yet
  if(!ctl.datastore().call<bool>("EF_Estimator::useForceSensor"))
  {
    ctl.datastore().call("EF_Estimator::toggleForceSensor");
  }
  ctl.datastore().call<void, double>("EF_Estimator::setGain", 10.0);

  // ctl.compPostureTask->makeCompliant(false);

  ctl.datastore().assign<std::string>("ControlMode", "Position");
  ctl.compPostureTask->target(ctl.postureBigRotate);
  ctl.compPostureTask->stiffness(10);
}

bool CircularOriController_BigRotate::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CircularOriController &>(ctl_);
  ctl.datastore().assign<std::string>("State", "BigRotate");

  if(ctl.datastore().get<bool>("Obstacle detected"))
  {
    ctl.compPostureTask->reset();
    ctl.compPostureTask->stiffness(500);
    output(ctl.reaction_mode);
    return true;
  }

  if (ctl.compPostureTask->eval().norm() < 0.1)
  {
    if(need_home_)
    {
      ctl.compPostureTask->target(ctl.postureBigRotate);
      need_home_ = false;
    }
    else
    {
      ctl.compPostureTask->target(ctl.postureBigRotate_end);
      need_home_ = true;
    }
  }

// output("OK");
//   return true;
  return false;
}

void CircularOriController_BigRotate::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CircularOriController &>(ctl_);
}

EXPORT_SINGLE_STATE("CircularOriController_BigRotate", CircularOriController_BigRotate)
