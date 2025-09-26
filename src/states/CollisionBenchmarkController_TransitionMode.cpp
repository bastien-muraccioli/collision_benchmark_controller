#include "CollisionBenchmarkController_TransitionMode.h"
#include <string>
#include <vector>

void CollisionBenchmarkController_TransitionMode::configure(const mc_rtc::Configuration & config) {}

void CollisionBenchmarkController_TransitionMode::start(mc_control::fsm::Controller & ctl_) 
{
    auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
    auto & robot = ctl.robot(ctl.robots()[0].name());
    isTorqueControl_ = !ctl.isTorqueControl; // ctl.isTorqueControl is the desired state (position or torque), but the current state is the opposite

    // Save the gains of the posture task
    stiffness_ = ctl.compPostureTask->stiffness();
    damping_ = ctl.compPostureTask->damping();
    weight_ = ctl.compPostureTask->weight();

    // Save the current target of the robot
    auto posture = ctl.compPostureTask->posture();
    std::vector<std::string> joint_names;
    joint_names.reserve(robot.mb().joints().size());  // Reserve space to avoid reallocation

    for (const auto &j : robot.mb().joints()) {
        const std::string &joint_name = j.name();
        joint_names.emplace_back(joint_name);  

        if (const auto &t = posture[robot.jointIndexByName(joint_name)]; !t.empty()) {
            target_[joint_name] = t;  
        }
    }


    // Print the target map
    // std::ostringstream oss;
    // for (const auto &pair : target_)
    // {
    //     oss << "Joint name: " << pair.first << ", Target: ";
    //     for (const auto &val : pair.second)
    //     {
    //         oss << val << " ";
    //     }
    //     oss << "\n";
    // }
    // mc_rtc::log::info(oss.str());

    ctl.compPostureTask->reset();
    // ctl.compPostureTask->stiffness(400.0);
    ctl.compPostureTask->setGains(10.0, 30.0);
    // ctl.compPostureTask->refVel(Eigen::VectorXd::Zero(target_.size()));

    mc_rtc::log::info("[CollisionBenchmarkController_TransitionMode] Transition mode to {} activated", ctl.isTorqueControl ? "Torque" : "Position");
}

bool CollisionBenchmarkController_TransitionMode::run(mc_control::fsm::Controller & ctl_) 
{ 
    auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);

    double velocity = ctl.robot().tvmRobot().alpha()->value().norm();
    mc_rtc::log::info("[CollisionBenchmarkController_TransitionMode] Joint velocity norm {}", velocity);
    if (velocity < 0.02)
    {   
        ctl.compPostureTask->setGains(0.0, 40.0);
    }

    if (velocity < 0.001)
    {
        ctl.compPostureTask->reset();
        ctl.compPostureTask->target(target_);
        ctl.compPostureTask->setGains(stiffness_, damping_);
        ctl.compPostureTask->weight(weight_);
        // From Torque to Position
        if(isTorqueControl_)
        {
            output("Position");
            return true;

        }
        // From Position to Torque
        else
        {
            output("Torque");
            return true;
        }  
    }
    return false;
}

void CollisionBenchmarkController_TransitionMode::teardown(mc_control::fsm::Controller & ctl) {}

EXPORT_SINGLE_STATE("CollisionBenchmarkController_TransitionMode", CollisionBenchmarkController_TransitionMode)