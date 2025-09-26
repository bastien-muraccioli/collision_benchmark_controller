#include "CollisionBenchmarkController.h"
#include <mc_rtc/gui/Checkbox.h>

CollisionBenchmarkController::CollisionBenchmarkController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
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

  postureBigUp = {{"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                   {"joint_5", {0}}, {"joint_6", {1.7}},  {"joint_7", {1.57}}};
  postureBigDown = {{"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                   {"joint_5", {0}}, {"joint_6", {-0.50}},  {"joint_7", {1.57}}};
  postureHorizontalStart = {{"joint_1", {0}}, {"joint_2", {0.0}}, {"joint_3", {3.14}}, {"joint_4", {-1.57}},
                   {"joint_5", {1.57}}, {"joint_6", {0}},  {"joint_7", {0}}};
  postureBigLeft = {{"joint_1", {0}}, {"joint_2", {0.0}}, {"joint_3", {3.14}}, {"joint_4", {-1.57}},
                   {"joint_5", {1.57}}, {"joint_6", {1.6}},  {"joint_7", {0}}};
  postureBigRight = {{"joint_1", {0}}, {"joint_2", {0.0}}, {"joint_3", {3.14}}, {"joint_4", {-1.57}},
                   {"joint_5", {1.57}}, {"joint_6", {-1.6}},  {"joint_7", {0}}};

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

  // Kinova Gen3 datastore
  datastore().make<std::string>("ControlMode", "Position");
  datastore().make<std::string>("TorqueMode", "Custom");

  // ConntrollerDatastore
  datastore().make<std::string>("State", "Initial");
  datastore().make<std::string>("ReactionMode", "ReactionSimple");
  // datastore().make<std::string>("ModeState", "Position");
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return compPostureTask; });
  logger().addLogEntry("EndEffectorVel", [this]() { return robot().bodyVelW("FT_sensor_mounting"); });
  logger().addLogEntry("CollisionBenchmarkController_fsmState",[this]() 
  { auto mode = datastore().get<std::string>("State");
    if(mode.compare("") == 0) return 0;
    if(mode.compare("Initial") == 0) return 1;
    if(mode.compare("Forward") == 0) return 2;
    if(mode.compare("Rotate") == 0) return 3;
    if(mode.compare("BigRotate") == 0) return 4;
    if(mode.compare("ReactionSimple") == 0) return 5; 
    });
  gui()->addElement({"Controller", "CollisionBenchmarkController"},
    mc_rtc::gui::ComboInput(
      "Reaction Mode", {"NoReaction", "ReactionSimple", "ReactionCompliance"},
      [this]() {return reaction_mode;},
      [this](const std::string & t){reaction_mode = t;}),
      mc_rtc::gui::Checkbox("Torque Control", isTorqueControl),
      mc_rtc::gui::Checkbox("Super Twisting", isSuperTwisting)
    );

  mc_rtc::log::success("CollisionBenchmarkController init done ");
}

bool CollisionBenchmarkController::run()
{
  // if(isTorqueControl != controlModeRequest)
  // {
  //   controlModeRequest = isTorqueControl;
  //   if(isTorqueControl)
  //   {
  //     datastore().assign<std::string>("ModeState", "Torque");
  //   }
  //   else
  //   {
  //     datastore().assign<std::string>("ModeState", "Position");
  //   }
  // }
  auto ctrl_mode = datastore().get<std::string>("ControlMode");
  if(ctrl_mode.compare("Position") == 0)
  {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::OpenLoop);
  }
  else
  {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  }
}

void CollisionBenchmarkController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}
