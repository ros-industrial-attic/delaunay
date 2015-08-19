#ifndef CLIK_PLANNER_H
#define CLIK_PLANNER_H

#include <constrained_ik/constrained_ik.h>
#include <constrained_ik/constraints/goal_orientation.h>
#include <constrained_ik/constraints/goal_position.h>
#include <constrained_ik/constraints/goal_pose.h>
#include <constrained_ik/constraints/avoid_singularities.h>
#include <constrained_ik/constraints/avoid_joint_limits.h>
#include <eigen3/Eigen/Eigen>
#include <constrained_ik/enum_types.h>

namespace teleop_robot
{

class ClikSolver : public constrained_ik::Constrained_IK
{
public:
  ClikSolver();
  ~ ClikSolver();

private:
  constrained_ik::constraints::GoalPosition* goal_position_;
  constrained_ik::constraints::GoalOrientation* goal_orientation_;
  constrained_ik::constraints::AvoidJointLimits* goal_jointlimits_;
};


}

#endif // CLIK_PLANNER_H

