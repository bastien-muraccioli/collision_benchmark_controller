#pragma once

#include <mc_control/mc_controller.h>

#include "api.h"

struct CircularOriController_DLLAPI CircularOriController : public mc_control::MCController
{
  CircularOriController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

private:
  mc_rtc::Configuration config_;
};
