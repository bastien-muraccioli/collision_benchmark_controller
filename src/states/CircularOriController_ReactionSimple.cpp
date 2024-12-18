#include "CircularOriController_ReactionSimple.h"
#include "../CircularOriController.h"

void CircularOriController_ReactionSimple::configure(const mc_rtc::Configuration & config) {}

void CircularOriController_ReactionSimple::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CircularOriController &>(ctl_);
  auto & robot = ctl.robot();
  auto & rjo = robot.refJointOrder();
  jointNumber_ = rjo.size();
  ctl.compPostureTask->reset();
  ctl.compPostureTask->stiffness(500);
  joint_stop_.resize(jointNumber_);
  
  for(int i = 0; i < jointNumber_; i++)
  {
    joint_stop_[i] = false;
  }
}

bool CircularOriController_ReactionSimple::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CircularOriController &>(ctl_);
  auto & robot = ctl.robot();
  auto & rjo = robot.refJointOrder();
  ctl.datastore().assign<std::string>("State", "ReactionSimple");

  all_joint_stop_ = true;
  for(int i = 0; i < jointNumber_; i++)
  {
    double velocity = robot.alpha()[robot.jointIndexByName(rjo[i])][0];
    mc_rtc::log::info("Joint: {}, velocity: {}", rjo[i], velocity);
    if(velocity > 0.001 && !joint_stop_[i])
    {
      ctl.compPostureTask->reset();
      all_joint_stop_ = false;
    }
    else
    {
      joint_stop_[i] = true;
    }
  }

  if(all_joint_stop_)
  {
    mc_rtc::log::info("All joint stopped");
    output("Init");
    return true;
  }

  return false;
}

void CircularOriController_ReactionSimple::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CircularOriController &>(ctl_);
}

EXPORT_SINGLE_STATE("CircularOriController_ReactionSimple", CircularOriController_ReactionSimple)
