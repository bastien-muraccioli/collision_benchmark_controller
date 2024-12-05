#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/CompliantPostureTask.h>
#include <mc_tasks/CompliantEndEffectorTask.h>

#include "api.h"

struct CircularOriController_DLLAPI CircularOriController : public mc_control::fsm::Controller
{
  CircularOriController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  // Targets
  std::map<std::string, std::vector<double>> postureHome;
  std::map<std::string, std::vector<double>> postureUp;
  std::map<std::string, std::vector<double>> postureDown;
  std::map<std::string, std::vector<double>> postureRight;
  std::map<std::string, std::vector<double>> postureLeft;
  std::map<std::string, std::vector<double>> postureBigRotate;
  std::map<std::string, std::vector<double>> postureBigRotate_end;

  Eigen::Vector3d taskPosHome;
  Eigen::Vector3d taskPosForward;

  // Tasks
  std::shared_ptr<mc_tasks::CompliantPostureTask> compPostureTask;
  std::shared_ptr<mc_tasks::CompliantEndEffectorTask> compEETask;

  bool plot_initialized = false;


private:
  mc_rtc::Configuration config_;
};
