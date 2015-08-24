#ifndef POINT_TO_TRIANGLE_H
#define POINT_TO_TRIANGLE_H

#include <eigen3/Eigen/Dense>

namespace teleop_tracking
{

Eigen::Vector3d closestPointOnTriangle(const Eigen::Vector3d *triangle, 
                                       const Eigen::Vector3d &sourcePosition);

bool intersect(const Eigen::ParametrizedLine<double, 3>& a,
               const Eigen::ParametrizedLine<double, 3>& b,
               Eigen::Vector3d& out);

bool intersectRayTriangle(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, 
                          const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& v3,
                          double& dist_out);
 


}

#endif // POINT_TO_TRIANGLE_H

