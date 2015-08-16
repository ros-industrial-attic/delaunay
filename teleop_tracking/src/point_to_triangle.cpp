#include "teleop_tracking/point_to_triangle.h"
#include <iostream>

template<typename T>
static T clamp(T d, T min, T max) {
  const T t = d < min ? min : d;
  return t > max ? max : t;
}

Eigen::Vector3d teleop_tracking::closestPointOnTriangle(const Eigen::Vector3d* triangle,
                                                       const Eigen::Vector3d& sourcePosition)
{
    Eigen::Vector3d edge0 = triangle[1] - triangle[0];
    Eigen::Vector3d edge1 = triangle[2] - triangle[0];
    Eigen::Vector3d v0 = triangle[0] - sourcePosition;

    float a = edge0.dot( edge0 );
    float b = edge0.dot( edge1 );
    float c = edge1.dot( edge1 );
    float d = edge0.dot( v0 );
    float e = edge1.dot( v0 );

    float det = a*c - b*b;
    float s = b*e - c*d;
    float t = b*d - a*e;

    if ( s + t < det )
    {
        if ( s < 0.f )
        {
            if ( t < 0.f )
            {
                if ( d < 0.f )
                {
                    s = clamp( -d/a, 0.f, 1.f );
                    t = 0.f;
                }
                else
                {
                    s = 0.f;
                    t = clamp( -e/c, 0.f, 1.f );
                }
            }
            else
            {
                s = 0.f;
                t = clamp( -e/c, 0.f, 1.f );
            }
        }
        else if ( t < 0.f )
        {
            s = clamp( -d/a, 0.f, 1.f );
            t = 0.f;
        }
        else
        {
            float invDet = 1.f / det;
            s *= invDet;
            t *= invDet;
        }
    }
    else
    {
        if ( s < 0.f )
        {
            float tmp0 = b+d;
            float tmp1 = c+e;
            if ( tmp1 > tmp0 )
            {
                float numer = tmp1 - tmp0;
                float denom = a-2*b+c;
                s = clamp( numer/denom, 0.f, 1.f );
                t = 1-s;
            }
            else
            {
                t = clamp( -e/c, 0.f, 1.f );
                s = 0.f;
            }
        }
        else if ( t < 0.f )
        {
            if ( a+d > b+e )
            {
                float numer = c+e-b-d;
                float denom = a-2*b+c;
                s = clamp( numer/denom, 0.f, 1.f );
                t = 1-s;
            }
            else
            {
                s = clamp( -e/c, 0.f, 1.f );
                t = 0.f;
            }
        }
        else
        {
            float numer = c+e-b-d;
            float denom = a-2*b+c;
            s = clamp( numer/denom, 0.f, 1.f );
            t = 1.f - s;
        }
    }

    return triangle[0] + s * edge0 + t * edge1;
}

bool teleop_tracking::intersect(const Eigen::ParametrizedLine<double, 3> &a,
                                const Eigen::ParametrizedLine<double,3> &b,
                                Eigen::Vector3d &out)
{
  Eigen::Vector3d da = a.direction();
  Eigen::Vector3d db = b.direction();
  Eigen::Vector3d dc = b.origin()- a.origin();

  Eigen::Vector3d dc_cross_db = dc.cross(db);
  Eigen::Vector3d da_cross_db = da.cross(db);

  if ( dc.dot(da.cross(db)) != 0.0) return false;

  double s = dc_cross_db.dot(da_cross_db) / da_cross_db.squaredNorm();
  if (s >= 0.0 && s <= 1.0)
  {
    out = a.origin() + da * s;
    return true;
  }

  return false;
}

bool teleop_tracking::intersectPlanes(const Eigen::ParametrizedLine<double, 3> &walk_dir,
                                      const Eigen::ParametrizedLine<double, 3> &edge,
                                      const Eigen::Hyperplane<double, 3> &plane,
                                      double &out)
{
  Eigen::Vector3d n = plane.normal();
  Eigen::Vector3d dir = edge.direction();
  Eigen::Hyperplane<double, 3> iplane(n.cross(dir), edge.origin());

  out = walk_dir.intersection(iplane);
  std::cout << "S1: " << out << '\n';

  return false;
}

bool teleop_tracking::intersectRayTriangle(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, 
                                           const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, 
                                           const Eigen::Vector3d& v3, double& dist_out)
{
  using namespace Eigen;
  Vector3d e1 = v2 - v1;
  Vector3d e2 = v3 - v1;

  Vector3d p = direction.cross(e2);
  double det = e1.dot(p);

  if (det > -0.00001 && det < 0.00001) return false;
  double inv_det = 1.0 / det;

  Vector3d T = origin - v1;
  double u = T.dot(p) * inv_det;

  if (u < 0.0 || u > 1.0) return false;

  Vector3d Q = T.cross(e1);

  double v = direction.dot(Q) * inv_det;
  if (v < 0.0 || v > 1.0) return false;

  double dist = e2.dot(Q) * inv_det;

  if (dist > 0.00001) {
    dist_out = dist;
    return true;
  }

  return false;
}
