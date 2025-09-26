#include "CollisionBenchmarkController_TorqueMode.h"

void CollisionBenchmarkController_TorqueMode::configure(const mc_rtc::Configuration & config) {}

void CollisionBenchmarkController_TorqueMode::start(mc_control::fsm::Controller & ctl_) 
{
    auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
    // Activate external force feedback
    estimatorManager(ctl);
    ctl.datastore().assign<std::string>("ControlMode", "Torque");
    mc_rtc::log::info("[CollisionBenchmarkController_TorqueMode] Torque mode activated");
}

bool CollisionBenchmarkController_TorqueMode::run(mc_control::fsm::Controller & ctl_) 
{
    auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
    if(!ctl.isTorqueControl)
    {
        output("Position");
        return true;
    }

    if(estimatorHasChanged != ctl.isSuperTwisting) estimatorManager(ctl);

    return false; 
}

void CollisionBenchmarkController_TorqueMode::estimatorManager(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
    if(ctl.isSuperTwisting)
    {
        if(ctl.datastore().has("SuperTwisting::isActive")) {
            if (!ctl.datastore().call<bool>("SuperTwisting::isActive")) {
              ctl.datastore().call("SuperTwisting::toggleActive");
            }
        }
        if(ctl.datastore().has("EF_Estimator::isActive")) {
            if (ctl.datastore().call<bool>("EF_Estimator::isActive")) {
              ctl.datastore().call("EF_Estimator::toggleActive");
            }
        }
    }
    else
    {
        if(ctl.datastore().has("EF_Estimator::isActive")) {
            if (!ctl.datastore().call<bool>("EF_Estimator::isActive")) {
              ctl.datastore().call("EF_Estimator::toggleActive");
            }
        }
        if(ctl.datastore().has("SuperTwisting::isActive")) {
            if (ctl.datastore().call<bool>("SuperTwisting::isActive")) {
              ctl.datastore().call("SuperTwisting::toggleActive");
            }
        }
    }
    estimatorHasChanged = ctl.isSuperTwisting;
}

void CollisionBenchmarkController_TorqueMode::teardown(mc_control::fsm::Controller & ctl) {}

EXPORT_SINGLE_STATE("CollisionBenchmarkController_TorqueMode", CollisionBenchmarkController_TorqueMode)