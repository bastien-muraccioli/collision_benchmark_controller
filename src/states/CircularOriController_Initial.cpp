#include "CircularOriController_Initial.h"

#include "../CircularOriController.h"

void CircularOriController_Initial::configure(const mc_rtc::Configuration & config) {}

void CircularOriController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CircularOriController &>(ctl_);
}

bool CircularOriController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CircularOriController &>(ctl_);
  // output("OK");
  // return true;
  return false;
}

void CircularOriController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CircularOriController &>(ctl_);
}

EXPORT_SINGLE_STATE("CircularOriController_Initial", CircularOriController_Initial)
