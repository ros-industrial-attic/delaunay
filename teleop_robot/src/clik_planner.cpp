#include "teleop_robot/clik_planner.h"

teleop_robot::ClikSolver::ClikSolver()
  : goal_position_(new constrained_ik::constraints::GoalPosition())
  , goal_orientation_(new constrained_ik::constraints::GoalToolOrientation())
  , goal_jointlimits_(new constrained_ik::constraints::AvoidJointLimits())
  , goal_singularities_(new constrained_ik::constraints::AvoidSingularities())
{
  // Set weigts
  goal_orientation_->setWeight(Eigen::Vector3d(1.0, 1.0, 0.1));
  // Add constraints
  addConstraint(goal_position_,constrained_ik::constraint_types::Primary);
  addConstraint(goal_orientation_,constrained_ik::constraint_types::Auxiliary);
  addConstraint(goal_jointlimits_, constrained_ik::constraint_types::Auxiliary);
//  addConstraint(goal_singularities_, constrained_ik::constraint_types::Auxiliary);
}

teleop_robot::ClikSolver::~ClikSolver() {}

