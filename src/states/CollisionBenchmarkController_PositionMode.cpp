#include "CollisionBenchmarkController_PositionMode.h"

void CollisionBenchmarkController_PositionMode::configure(const mc_rtc::Configuration & config) {}

void CollisionBenchmarkController_PositionMode::start(mc_control::fsm::Controller & ctl_) 
{
    auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
    // ctl.compPostureTask->makeCompliant(false);

    // Disable feedback from external forces estimator (safer)
    if(ctl.datastore().has("SuperTwisting::isActive")) {
        if (ctl.datastore().call<bool>("SuperTwisting::isActive")) {
            ctl.datastore().call("SuperTwisting::toggleActive");
        }
        }

    if(ctl.datastore().has("EF_Estimator::isActive")) {
        if (ctl.datastore().call<bool>("EF_Estimator::isActive")) {
            ctl.datastore().call("EF_Estimator::toggleActive");
        }
    }

    ctl.datastore().assign<std::string>("ControlMode", "Position");
    mc_rtc::log::info("[CollisionBenchmarkController_PositionMode] Position mode activated");
}

bool CollisionBenchmarkController_PositionMode::run(mc_control::fsm::Controller & ctl_) 
{
    auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
    if(ctl.isTorqueControl)
    {
        output("Torque");
        return true;
    }
    return false;
}

void CollisionBenchmarkController_PositionMode::teardown(mc_control::fsm::Controller & ctl) {}

EXPORT_SINGLE_STATE("CollisionBenchmarkController_PositionMode", CollisionBenchmarkController_PositionMode)