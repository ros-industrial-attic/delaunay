#include "teleop_robot/clik_planner.h"

teleop_robot::ClikSolver::ClikSolver()
  : goal_position_(new constrained_ik::constraints::GoalPosition())
  , goal_orientation_(new constrained_ik::constraints::GoalOrientation())
  , goal_jointlimits_(new constrained_ik::constraints::AvoidJointLimits())
{
  addConstraint(goal_position_,constrained_ik::constraint_types::Primary);
  addConstraint(goal_orientation_,constrained_ik::constraint_types::Auxiliary);
  addConstraint(goal_jointlimits_, constrained_ik::constraint_types::Auxiliary);
  Eigen::Vector3d w_ori;
  w_ori << 1,1,1;
  goal_orientation_->setWeight(w_ori);
}

teleop_robot::ClikSolver::~ClikSolver() {}

/*
//create solver and initialize with kinematic model
seed.setZero(dimension);
seed << 169, -10, 10, -10, 10, -10, 10;
seed = (M_PI/180) * seed;
kin.calcFwdKin(seed, goal);
std::cout << "Goal" << std::endl;
std::cout << goal.matrix() << std::endl;
seed << 0, -30, 30, -30, 30, -30, 10;
//seed << 169, -10, 10, -10, 10, -10, 10;
seed = (M_PI/180) * seed;
solver.init(kin);

//Do IK and report results
try
{
  solver.calcInvKin(goal, seed, planning_scene, joint_angles);
}
catch (std::exception &e)
{
  ROS_ERROR_STREAM("Caught exception from IK: " << e.what());
  return false;
}
solution.resize(dimension);
for(size_t ii=0; ii < dimension; ++ii)
{
  solution[ii] = joint_angles(ii);
}
kin.calcFwdKin(joint_angles,goal);
std::cout << "Goal" << std::endl;
std::cout << goal.matrix() << std::endl;
//ROS_INFO_STREAM("GOAL:\n" << goal.matrix() << std::endl);
return true;
}*/
