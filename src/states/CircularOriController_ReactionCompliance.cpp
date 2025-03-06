#include "CircularOriController_ReactionCompliance.h"
#include "../CircularOriController.h"

void CircularOriController_ReactionCompliance::configure(const mc_rtc::Configuration & config) {}

void CircularOriController_ReactionCompliance::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CircularOriController &>(ctl_);
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
  // ctl.datastore().call<void, double>("EF_Estimator::setGain", 30.0);

  ctl.compPostureTask->stiffness(0.0);
  ctl.compPostureTask->damping(5.0);
  ctl.compPostureTask->weight(1);
  ctl.compPostureTask->makeCompliant(true);

  // auto & eeTarget = ctl.robot().bodyPosW("FT_sensor_mounting");

  ctl.compEETask->reset();
  // ctl.compEETask->positionTask->reset();
  // ctl.compEETask->positionTask->position(eeTarget.translation());
  ctl.compEETask->positionTask->stiffness(1);
  ctl.compEETask->positionTask->weight(10000);
  ctl.compEETask->orientationTask->reset();
  // ctl.compEETask->orientationTask->orientation(eeTarget.rotation());
  ctl.compEETask->orientationTask->stiffness(1);
  ctl.compEETask->orientationTask->weight(10000);
  ctl.compEETask->makeCompliant(true);
  ctl.solver().addTask(ctl.compEETask);

  // auto & eeTarget = ctl.robot().bodyPosW("FT_sensor_mounting");

  // ctl.compEETask->reset();
  // ctl.compEETask->positionTask->reset();
  // ctl.compEETask->positionTask->position(eeTarget.translation());
  // ctl.compEETask->positionTask->weight(10000);
  // ctl.compEETask->positionTask->stiffness(100);
  // ctl.compEETask->orientationTask->reset();
  // ctl.compEETask->orientationTask->weight(10000);
  // ctl.compEETask->orientationTask->stiffness(30);
  
  // ctl.compEETask->orientationTask->orientation(eeTarget.rotation());
  // ctl.compEETask->makeCompliant(false);
  // ctl.solver().addTask(ctl.compEETask);

  

  ctl.datastore().assign<std::string>("ControlMode", "Torque");
}

bool CircularOriController_ReactionCompliance::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CircularOriController &>(ctl_);
  return false;
}

void CircularOriController_ReactionCompliance::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CircularOriController &>(ctl_);
}

EXPORT_SINGLE_STATE("CircularOriController_ReactionCompliance", CircularOriController_ReactionCompliance)
