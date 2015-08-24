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

bool teleop_tracking::intersectRayTriangle(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, 
                                           const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, 
                                           const Eigen::Vector3d& v3, double& dist_out)
{
  const static double SMALL_NUMBER = 1e-8;
  Eigen::Vector3d    u, v, n;              // triangle vectors
  Eigen::Vector3d    dir, w0, w;           // ray vectors
  double     r, a, b;              // params to calc ray-plane intersect

  // get triangle edge vectors and plane normal
  u = v2 - v1;
  v = v3 - v1;
  n = u.cross(v);              // cross product
  if (n == Eigen::Vector3d::Zero())             // triangle is degenerate
      return false;                  // do not deal with this case

  dir = direction;              // ray direction vector
  w0 = origin - v1;
  a = -(n.dot(w0));
  b = n.dot(dir);
  if (std::fabs(b) < SMALL_NUMBER) {     // ray is  parallel to triangle plane
      if (a == 0.0)                 // ray lies in triangle plane
          return true;
      else return false;             // ray disjoint from plane
  }

  // get intersect point of ray with triangle plane
  r = a / b;
  if (r < 0.0)                    // ray goes away from triangle
      return false;                   // => no intersect
  // for a segment, also test if (r > 1.0) => no intersect

  Eigen::Vector3d I;
  I = origin + r * dir;            // intersect point of ray and plane

  // is I inside T?
  double    uu, uv, vv, wu, wv, D;
  uu = u.dot(u);
  uv = u.dot(v);
  vv = v.dot(v);
  w = I - v1;
  wu = w.dot(u);
  wv = w.dot(v);
  D = uv * uv - uu * vv;

  // get and test parametric coords
  double s, t;
  s = (uv * wv - vv * wu) / D;
  if (s < 0.0 || s > 1.0)         // I is outside T
      return false;
  t = (uv * wu - uu * wv) / D;
  if (t < 0.0 || (s + t) > 1.0)  // I is outside T
      return false;

  dist_out = r;
  return true;                       // I is in T
}
