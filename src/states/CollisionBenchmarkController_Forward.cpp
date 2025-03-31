#include "CollisionBenchmarkController_Forward.h"
#include "../CollisionBenchmarkController.h"

void CollisionBenchmarkController_Forward::configure(const mc_rtc::Configuration & config) {}

void CollisionBenchmarkController_Forward::start(mc_control::fsm::Controller & ctl_)
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
  ctl.compPostureTask->stiffness(0.5);
  ctl.compPostureTask->weight(0.1);
  ctl.compEETask->reset();
  ctl.compEETask->positionTask->position(ctl.taskPosHome);
  ctl.compEETask->positionTask->stiffness(100);
  ctl.compEETask->orientationTask->stiffness(20);
  ctl.compEETask->positionTask->weight(10000);
  
  ctl.solver().addTask(ctl.compEETask);
}

bool CollisionBenchmarkController_Forward::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  ctl.datastore().assign<std::string>("State", "Forward");
  if(ctl.datastore().get<bool>("Obstacle detected"))
  {
    ctl.compPostureTask->reset();
    ctl.compEETask->reset();
    ctl.compPostureTask->stiffness(500);
    output(ctl.reaction_mode);
    return true;
  }

  if (ctl.compEETask->positionTask->eval().norm() < 0.01)
  {
   
    if(need_home_)
    {
      ctl.compEETask->positionTask->position(ctl.taskPosHome);
      need_home_ = false;
      // mc_rtc::log::info("Need home");
      ctl.compEETask->positionTask->stiffness(10);
    }
    else
    {
      ctl.compEETask->positionTask->position(ctl.taskPosForward);
      need_home_ = true;
      // mc_rtc::log::info("Need forward");
      ctl.compEETask->positionTask->stiffness(100);
    }
  }

// output("OK");
//   return true;
  return false;
}

void CollisionBenchmarkController_Forward::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  ctl.solver().removeTask(ctl.compEETask);
}

EXPORT_SINGLE_STATE("CollisionBenchmarkController_Forward", CollisionBenchmarkController_Forward)
