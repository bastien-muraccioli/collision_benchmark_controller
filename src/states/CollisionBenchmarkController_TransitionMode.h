#pragma once

#include <mc_control/fsm/State.h>
#include "../CollisionBenchmarkController.h"
#include <mc_tvm/Robot.h>

struct CollisionBenchmarkController_TransitionMode : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;


private:
  bool isTorqueControl_;
  std::map<std::string, std::vector<double>> target_;
  double stiffness_;
  double damping_;
  double weight_;
};