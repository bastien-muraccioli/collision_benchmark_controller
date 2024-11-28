#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/CompliantPostureTask.h>

#include "api.h"

struct CircularOriController_DLLAPI CircularOriController : public mc_control::fsm::Controller
{
  CircularOriController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  std::map<std::string, std::vector<double>> postureHome;
  std::map<std::string, std::vector<double>> postureUp;
  std::map<std::string, std::vector<double>> postureDown;
  std::map<std::string, std::vector<double>> postureRight;
  std::map<std::string, std::vector<double>> postureLeft;
  std::shared_ptr<mc_tasks::CompliantPostureTask> compPostureTask;

private:
  mc_rtc::Configuration config_;
};
