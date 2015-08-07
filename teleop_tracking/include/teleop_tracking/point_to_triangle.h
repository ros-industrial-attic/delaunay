#ifndef POINT_TO_TRIANGLE_H
#define POINT_TO_TRIANGLE_H

#include <eigen3/Eigen/Dense>

namespace teleop_tracking
{

Eigen::Vector3d closesPointOnTriangle(const Eigen::Vector3d *triangle, const Eigen::Vector3d &sourcePosition);


bool intersect(const Eigen::ParametrizedLine<double, 3>& a,
               const Eigen::ParametrizedLine<double, 3>& b,
               Eigen::Vector3d& out);

bool intersectPlanes(const Eigen::ParametrizedLine<double, 3>& a,
                     const Eigen::ParametrizedLine<double, 3>& b, const Eigen::Hyperplane<double, 3> &plane,
                     double &out);


}

#endif // POINT_TO_TRIANGLE_H

