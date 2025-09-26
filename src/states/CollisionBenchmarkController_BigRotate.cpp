#include "CollisionBenchmarkController_BigRotate.h"
#include <mc_rtc/logging.h>
#include <RBDyn/MultiBodyConfig.h>
#include <Eigen/src/Core/Matrix.h>
#include <cstdlib>

void CollisionBenchmarkController_BigRotate::configure(const mc_rtc::Configuration & config) {}

void CollisionBenchmarkController_BigRotate::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  // Activate feedback from external forces estimator (safer)
  // if(ctl.datastore().call<bool>("EF_Estimator::isActive"))
  // {
  //   ctl.datastore().call("EF_Estimator::toggleActive");
  // }
  // // Activate force sensor usage if not used yet
  // if(!ctl.datastore().call<bool>("EF_Estimator::useForceSensor"))
  // {
  //   ctl.datastore().call("EF_Estimator::toggleForceSensor");
  // }
  // ctl.datastore().call<void, double>("EF_Estimator::setGain", 10.0);

  // ctl.compPostureTask->makeCompliant(false);
  jointNumber = ctl.robot(ctl.robots()[0].name()).refJointOrder().size();
// ctl.datastore().assign<std::string>("ControlMode", "Position");
  
  ctl.compPostureTask->target(ctl.postureBigRotate);
  ctl.compPostureTask->stiffness(200);
  velocity_ = Eigen::VectorXd::Zero(jointNumber);
  accel_ = Eigen::VectorXd::Zero(jointNumber);
  q_d_ = ctl.postureBigRotate;
  
  auto it = ctl.postureBigRotate.begin();
  q_min_ = it->second[0];
  q_d_zero_ = q_min_;
  auto it2 = ctl.postureBigRotate_end.begin();
  q_max_ = it2->second[0];
  delta_q_ = std::abs(q_max_ - q_min_);

  // Dynamically calculate the total time that is determined by target position and speed limits
  total_time_ = delta_q_ / vel_max_ / (1 - accel_ratio_);  // Calculate the required time based on target position and peak speed
  
  // Time partitioning
  tf_acc_ = total_time_ * accel_ratio_; // Acceleration time
  tf_const_ = total_time_ - 2 * tf_acc_; // Constant speed time

  // Calculate constant acceleration
  accel_constant_ = vel_max_ / tf_acc_;

  mc_rtc::log::info("[CollisionBenchmarkController_BigRotate] q_min_ {}, q_max_ {}, delta_q_ {}, vel_max_ {}, total_time_ {}, tf_acc_ {}, tf_const_ {}, accel_constant_ {}",
                    q_min_, q_max_, delta_q_, vel_max_, total_time_, tf_acc_, tf_const_, accel_constant_);
}

bool CollisionBenchmarkController_BigRotate::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
  ctl.datastore().assign<std::string>("State", "BigRotate");

  if(ctl.datastore().get<bool>("Obstacle detected"))
  {
    ctl.compPostureTask->reset();
    ctl.compPostureTask->stiffness(500);
    output(ctl.reaction_mode);
    return true;
  }

  // if (ctl.compPostureTask->eval().norm() < 0.01)
  // {
  computeTrapezoidVelocity(ctl);
  ctl.compPostureTask->refAccel(accel_);
  ctl.compPostureTask->refVel(velocity_);
  ctl.compPostureTask->target(q_d_);
  // }

  // if (ctl.compPostureTask->eval().norm() < 0.1)
  // {
  //   if(need_home_)
  //   {
  //     ctl.compPostureTask->target(ctl.postureBigRotate);
  //     need_home_ = false;
  //   }
  //   else
  //   {
  //     ctl.compPostureTask->target(ctl.postureBigRotate_end);
  //     need_home_ = true;
  //   }
  // }

// output("OK");
//   return true;
  return false;
}

void CollisionBenchmarkController_BigRotate::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<CollisionBenchmarkController &>(ctl_);
}

void CollisionBenchmarkController_BigRotate::computeTrapezoidVelocity(mc_control::fsm::Controller & ctl)
{

  auto & realRobot = ctl.realRobot(ctl.robots()[0].name());

  time_counter_ += ctl.timeStep;
  double a_t;
  double v_t;
  double q_zero;

  if(sign_ == -1.0){
    q_zero = q_max_;
  }
  else {
    q_zero = q_min_;
  }

  if(time_counter_ <= tf_acc_)
  {
    // mc_rtc::log::info(
    //   "[CollisionBenchmarkController_BigRotate] Acceleration phase");
    // Acceleration phase
    a_t = accel_constant_ * sign_;
    v_t = accel_constant_ * time_counter_ * sign_;
    q_d_zero_ = q_zero + 0.5 * accel_constant_ * time_counter_ * time_counter_ * sign_;
  }
  else if(time_counter_ > tf_acc_ && time_counter_ <= tf_const_ + tf_acc_)
  {
    // mc_rtc::log::info(
    //   "[CollisionBenchmarkController_BigRotate] Constant speed phase");
    // Constant speed phase
    a_t = 0.0;
    v_t = vel_max_* sign_;
    q_d_zero_ = q_zero + 0.5 * accel_constant_ * tf_acc_ * tf_acc_ * sign_ + vel_max_ * (time_counter_ - tf_acc_) * sign_;
  }
  else if(time_counter_ > tf_const_ + tf_acc_)
  {
    // mc_rtc::log::info(
    //   "[CollisionBenchmarkController_BigRotate] Deceleration phase");
    // Deceleration phase
    double t_dec = time_counter_ - (tf_acc_ + tf_const_);
    a_t = -accel_constant_ * sign_;
    v_t = vel_max_ * sign_ - accel_constant_ * t_dec * sign_;
    q_d_zero_ = q_zero + 0.5 * accel_constant_ * tf_acc_ * tf_acc_ * sign_ +
     vel_max_ * tf_const_ * sign_ + 
     vel_max_ * t_dec * sign_ - 0.5 * accel_constant_ * t_dec * t_dec * sign_;

    if(time_counter_ >= total_time_)
    {
      // mc_rtc::log::info(
      //   "[CollisionBenchmarkController_BigRotate] Reset phase");
      // Reset
      time_counter_ = 0.0;
      if(sign_ == 1.0)
      { 
        q_d_zero_ = q_max_;
        sign_ = -1.0;
        // mc_rtc::log::info(
        //   "[CollisionBenchmarkController_BigRotate] negative_direction");
      }
      else
      {
        // mc_rtc::log::info(
        //   "[CollisionBenchmarkController_BigRotate] positive_direction");
        q_d_zero_ = q_min_;
        sign_ = 1.0;
      }

    }
  }
  else
  {
    mc_rtc::log::warning(
      "[CollisionBenchmarkController_BigRotate] Out of time");
  }

  // mc_rtc::log::info(
  //   "[CollisionBenchmarkController_BigRotate] time_counter_ {}, a_t {}, v_t {}, q_d_zero_ {}",
  //   time_counter_, a_t, v_t, q_d_zero_);

  
  auto it = q_d_.begin();
  q_d_[it->first][0] = q_d_zero_;
  velocity_[0] = v_t;
  accel_[0] = a_t;
}

EXPORT_SINGLE_STATE("CollisionBenchmarkController_BigRotate", CollisionBenchmarkController_BigRotate)
