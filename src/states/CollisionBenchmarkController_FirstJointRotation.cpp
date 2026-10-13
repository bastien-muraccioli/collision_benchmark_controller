#include "CollisionBenchmarkController_FirstJointRotation.h"
#include <mc_rtc/logging.h>
#include <RBDyn/MultiBodyConfig.h>
#include <Eigen/src/Core/Matrix.h>
#include <cstdlib>

void CollisionBenchmarkController_FirstJointRotation::configure(const mc_rtc::Configuration & config) {}

void CollisionBenchmarkController_FirstJointRotation::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  jointNumber = ctl.robot(ctl.robots()[0].name()).refJointOrder().size();

  ctl.postureTask->target(ctl.postureFirstJointRotation);
  ctl.postureTask->stiffness(400);
  velocity_ = Eigen::VectorXd::Zero(jointNumber);
  accel_ = Eigen::VectorXd::Zero(jointNumber);
  q_d_ = ctl.postureFirstJointRotation;

  q_start_ = ctl.postureFirstJointRotation.at("joint_1")[0];
  q_end_ = ctl.postureFirstJointRotation_end.at("joint_1")[0];
  dir0_ = (q_end_ >= q_start_) ? 1.0 : -1.0; // direction derived from the targets
  delta_q_ = std::abs(q_end_ - q_start_);
  forward_ = true;
  time_counter_ = 0.0;
  q_d_zero_ = q_start_;

  total_time_ = delta_q_ / vel_max_ / (1 - accel_ratio_);
  tf_acc_ = total_time_ * accel_ratio_;
  tf_const_ = total_time_ - 2 * tf_acc_;
  accel_constant_ = vel_max_ / tf_acc_;

  mc_rtc::log::info("[CollisionBenchmarkController_FirstJointRotation] q_start_ {}, q_end_ {}, dir {}, total_time_ {}",
                    q_start_, q_end_, dir0_, total_time_);
}

bool CollisionBenchmarkController_FirstJointRotation::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);

  if(ctl.datastore().get<bool>("Obstacle detected"))
  {
    ctl.postureTask->reset();
    ctl.postureTask->stiffness(500);
    output("OK");
    return true;
  }

  computeTrapezoidVelocity(ctl);
  ctl.postureTask->refAccel(accel_);
  ctl.postureTask->refVel(velocity_);
  ctl.postureTask->target(q_d_);

  return false;
}

void CollisionBenchmarkController_FirstJointRotation::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
}

void CollisionBenchmarkController_FirstJointRotation::computeTrapezoidVelocity(mc_control::fsm::Controller & ctl)
{
  time_counter_ += ctl.timeStep;
  const double t = std::min(time_counter_, total_time_); // no overshoot past the end

  const double dir = forward_ ? dir0_ : -dir0_;
  const double q0 = forward_ ? q_start_ : q_end_;

  double a, v, s; // magnitudes along the path
  if(t <= tf_acc_)
  {
    a = accel_constant_;
    v = accel_constant_ * t;
    s = 0.5 * accel_constant_ * t * t;
  }
  else if(t <= tf_acc_ + tf_const_)
  {
    a = 0.0;
    v = vel_max_;
    s = 0.5 * accel_constant_ * tf_acc_ * tf_acc_ + vel_max_ * (t - tf_acc_);
  }
  else
  {
    const double td = t - (tf_acc_ + tf_const_);
    a = -accel_constant_;
    v = vel_max_ - accel_constant_ * td;
    s = 0.5 * accel_constant_ * tf_acc_ * tf_acc_ + vel_max_ * tf_const_ + vel_max_ * td
        - 0.5 * accel_constant_ * td * td;
  }

  q_d_zero_ = q0 + dir * s;
  double v_t = dir * v;
  double a_t = dir * a;

  if(time_counter_ >= total_time_)
  {
    // End of stroke: snap to the target and reverse
    q_d_zero_ = forward_ ? q_end_ : q_start_;
    v_t = 0.0;
    a_t = 0.0;
    time_counter_ = 0.0;
    forward_ = !forward_;
  }

  q_d_["joint_1"][0] = q_d_zero_;
  velocity_[0] = v_t;
  accel_[0] = a_t;
}

EXPORT_SINGLE_STATE("CollisionBenchmarkController_FirstJointRotation", CollisionBenchmarkController_FirstJointRotation)
