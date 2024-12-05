#include "CircularOriController.h"

CircularOriController::CircularOriController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  // Initialize the constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {1.8, 70.0});
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, 0.0, 1.8, 70.0}, 0.9, true));
  solver().addConstraintSet(dynamicsConstraint);

  postureHome = {{"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                   {"joint_5", {0}}, {"joint_6", {0.96}},  {"joint_7", {1.57}}};
  postureUp = {{"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                   {"joint_5", {0}}, {"joint_6", {1.40}},  {"joint_7", {1.57}}};
  postureDown = {{"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                   {"joint_5", {0}}, {"joint_6", {0.60}},  {"joint_7", {1.57}}};
  postureRight = {{"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                   {"joint_5", {0.96}}, {"joint_6", {0.96}},  {"joint_7", {1.57}}};
  postureLeft = {{"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                   {"joint_5", {-0.96}}, {"joint_6", {0.96}},  {"joint_7", {1.57}}};
  // postureBigRotate = {{"joint_1", {1.57}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                  //  {"joint_5", {0}}, {"joint_6", {0.96}},  {"joint_7", {1.57}}};
  postureBigRotate = {{"joint_1", {0}}, {"joint_2", {0.8}}, {"joint_3", {3.14}}, {"joint_4", {-1.3}},
                   {"joint_5", {0}}, {"joint_6", {0.55}},  {"joint_7", {1.57}}};
  postureBigRotate_end = {{"joint_1", {1.57}}, {"joint_2", {0.8}}, {"joint_3", {3.14}}, {"joint_4", {-1.3}},
                   {"joint_5", {0}}, {"joint_6", {0.55}},  {"joint_7", {1.57}}};

  taskPosHome = Eigen::Vector3d(0.45, 0.0, 0.45);
  taskPosForward = Eigen::Vector3d(0.65, 0.0, 0.45);

  

  solver().removeTask(getPostureTask(robot().name()));
  compEETask = std::make_shared<mc_tasks::CompliantEndEffectorTask>("FT_sensor_mounting", robots(),
                                                                    robot().robotIndex(), 1, 1);
  // compEETask->reset();
  // // compEETask->positionTask->refVel(Eigen::Vector3d(1.5, 1.5, 1.5));
  // solver().addTask(compEETask);
  
  compPostureTask = std::make_shared<mc_tasks::CompliantPostureTask>(solver(), robot().robotIndex(), 1, 1);
  compPostureTask->reset();
  compPostureTask->stiffness(100.0);
  // compPostureTask->damping(3.0);
  solver().addTask(compPostureTask);

  datastore().make<std::string>("ControlMode", "Position");
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return compPostureTask; });
  logger().addLogEntry("EndEffectorVel", [this]() { return robot().bodyVelW("FT_sensor_mounting"); });

  mc_rtc::log::success("CircularOriController init done ");
}

bool CircularOriController::run()
{
  return mc_control::fsm::Controller::run();
}

void CircularOriController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}
