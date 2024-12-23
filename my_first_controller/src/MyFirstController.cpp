#include "MyFirstController.h"

MyFirstController::MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  jointIndex = robot().jointIndexByName("NECK_Y");
  
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addTask(postureTask);
  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});

  comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0, 10.0, 1000.0); //com task
  solver().addTask(comTask);
  postureTask->stiffness(1);

  mc_rtc::log::success("MyFirstController init done ");
}

bool MyFirstController::run()
{
  if(comTask->eval().norm() < 0.01)
  {
    switch_com_target();
  }
  return mc_control::MCController::run(); //delegate to MCController to run the QP
  }

void MyFirstController::reset(const mc_control::ControllerResetData & reset_data)
{
  comTask->reset();
  comZero = comTask->com();
  mc_control::MCController::reset(reset_data);
}

void MyFirstController::switch_target()
{
  if(goingLeft)
  {
    postureTask->target({{"NECK_Y", robot().qu()[jointIndex]}});
  }
  else
  {
    postureTask->target({{"NECK_Y", robot().ql()[jointIndex]}});
  }
  goingLeft = !goingLeft;
}

void MyFirstController::switch_com_target()
{
  // comZero is obtained by doing:
  // comZero = comTask->com();
  // in the reset function
  if(comDown) { comTask->com(comZero - Eigen::Vector3d{0, 0, 0.2}); }
  else { comTask->com(comZero); }
  comDown = !comDown;
}



CONTROLLER_CONSTRUCTOR("MyFirstController", MyFirstController)
