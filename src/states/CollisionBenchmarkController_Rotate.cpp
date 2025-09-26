#include "CollisionBenchmarkController_Rotate.h"

void CollisionBenchmarkController_Rotate::configure(const mc_rtc::Configuration & config) {}

void CollisionBenchmarkController_Rotate::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  // Activate feedback from external forces estimator (safer)
  // if(!ctl.datastore().call<bool>("EF_Estimator::isActive"))
  // {
  //   ctl.datastore().call("EF_Estimator::toggleActive");
  // }
  // // Activate force sensor usage if not used yet
  // if(!ctl.datastore().call<bool>("EF_Estimator::useForceSensor"))
  // {
  //   ctl.datastore().call("EF_Estimator::toggleForceSensor");
  // }
  // ctl.datastore().call<void, double>("EF_Estimator::setGain", 10.0);

  // ctl.compPostureTask->makeCompliant(false);

  ctl.datastore().assign<std::string>("ControlMode", "Position");

  ctl.compPostureTask->target(ctl.postureHome);

  ctl.compPostureTask->stiffness(100);
}

bool CollisionBenchmarkController_Rotate::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  ctl.datastore().assign<std::string>("State", "Rotate");

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
      ctl.compPostureTask->target(ctl.postureHome);
      need_home_ = false;
    }
    else
    {
      switch (state_) 
      {
        case 0:
          ctl.compPostureTask->target(ctl.postureUp);
          state_ = 1;
          break;
        case 1:
          ctl.compPostureTask->target(ctl.postureDown);
          state_ = 2;
          break;
        case 2:
          ctl.compPostureTask->target(ctl.postureRight);
          state_ = 3;
          break;
        case 3:
          ctl.compPostureTask->target(ctl.postureLeft);
          state_ = 0;
          break;
      }
      need_home_ = true;
    }
  }

  // reset_plot_timer += ctl.timeStep;
  // if(reset_plot_timer > reset_plot_max_time)
  // {
  //   ctl.datastore().call("ObstacleDetectionJerkEstimator::ResetPlot");
  //   reset_plot_timer = 0.0;
  // }
// output("OK");
//   return true;
  return false;
}

void CollisionBenchmarkController_Rotate::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  // ctl.datastore().call("ObstacleDetectionJerkEstimator::RemovePlot");
}

EXPORT_SINGLE_STATE("CollisionBenchmarkController_Rotate", CollisionBenchmarkController_Rotate)
