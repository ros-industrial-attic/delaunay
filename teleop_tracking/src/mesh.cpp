#include "teleop_tracking/mesh.h"
#include "teleop_tracking/point_to_triangle.h"

#include "teleop_tracking/stl_loader.h"
#include "combine_vertices.h"

#include <ros/console.h>
#include <cmath>

inline Eigen::Affine3d makePose(const Eigen::Vector3d& origin,
                                const Eigen::Vector3d& x,
                                const Eigen::Vector3d& y,
                                const Eigen::Vector3d& z)
{
  Eigen::Affine3d p = Eigen::Affine3d::Identity();
  p.translation() = origin;
  p.matrix().col(0).head<3>() = x;
  p.matrix().col(1).head<3>() = y;
  p.matrix().col(2).head<3>() = z;
  return p;
}

inline Eigen::Affine3d makeArbitraryPose(const Eigen::Vector3d& origin,
                                         const Eigen::Vector3d& normal)
{
  Eigen::Hyperplane<double, 3> plane (normal, origin);
  Eigen::Affine3d p = Eigen::Affine3d::Identity();
  Eigen::Vector3d x = Eigen::Vector3d::UnitX();
  if (x.dot(normal) < 0.00001)
  {
    x = Eigen::Vector3d::UnitY();
  }

  x = origin - plane.projection(x + origin);
  x.normalize();

  Eigen::Vector3d y = x.cross(normal);

  return makePose(origin, x, y, normal);
}

teleop_tracking::Mesh::Mesh(const std::string& meshfile)
{
  StlLoader loader (meshfile);
  std::cout << "Combine step...\n";
  combineVertices(loader.facets(), vertices_, face_normals_, triangle_indices_);
}

teleop_tracking::TrianglePosition teleop_tracking::Mesh::closestPoint(const Eigen::Vector3d &source) const
{
  double min_dist = std::numeric_limits<double>::max();
  Eigen::Vector3d closest_point (0, 0, 0);
  unsigned int closest_triangle = 0;

  for (unsigned i = 0; i < triangle_indices_.size(); i += 3)
  {
    // triangle built from 3 vertexes
    Eigen::Vector3d t[3] = {
      (vertices_[triangle_indices_[i + 0]]),
      (vertices_[triangle_indices_[i + 1]]),
      (vertices_[triangle_indices_[i + 2]])
    };

    Eigen::Vector3d c = closesPointOnTriangle(t, source);
    double d = (source - c).norm();

    if (d < min_dist)
    {
      closest_triangle = i / 3;
      min_dist = d;
      closest_point = c;
    }
  }

  TrianglePosition result;
  result.index = closest_triangle;
  result.position = closest_point;
  return result;
}

teleop_tracking::TrianglePose teleop_tracking::Mesh::closestPose(const Eigen::Vector3d &source) const
{
  TrianglePosition p = closestPoint(source);
  TrianglePose tpose;
  tpose.pose = makeArbitraryPose(p.position, face_normals_[p.index]);
  tpose.index = p.index;
  return tpose;
}

void teleop_tracking::Mesh::debugInfo() const
{
  std::cout << "Vertices: " << vertices_.size() << '\n';
  std::cout << "Normals: " << face_normals_.size() << '\n';
  std::cout << "Triangle Indices: " << triangle_indices_.size() << '\n';
  std::cout << "Triangles: " << triangle_indices_.size() / 3 << '\n';
}

Eigen::Vector3d teleop_tracking::Mesh::walkTriangles(const Eigen::Vector3d &start, Eigen::Vector3d &dir, double d) const
{
  typedef Eigen::ParametrizedLine<double, 3> Line3d;

  TrianglePosition close_pt = closestPoint(start);

  // Triangle edges
  Eigen::Vector3d a = vertices_[triangle_indices_[close_pt.index*3 + 0]];
  Eigen::Vector3d b = vertices_[triangle_indices_[close_pt.index*3 + 1]];
  Eigen::Vector3d c = vertices_[triangle_indices_[close_pt.index*3 + 2]];

  std::cout << "INDX: " << (close_pt.index * 3 + 0) << '\n';

  std::cout << "Triangle:\n" << a << "\n-\n" << b << "-\n-" << c << '\n';
  std::cout << "Normal: \n" << face_normals_[close_pt.index] << '\n';

  Eigen::Hyperplane<double, 3> pp (face_normals_[close_pt.index], close_pt.position);

  Eigen::Affine3d p = makeArbitraryPose(close_pt.position, face_normals_[close_pt.index]);
  Eigen::Vector3d p_x_dir = p.matrix().col(0).head<3>();
  std::cout << "p x dir:\n" << p_x_dir << '\n';
  // Define walk dir
  Eigen::ParametrizedLine<double, 3> walk_dir (close_pt.position,
                                               p_x_dir);

  std::cout << "Walk dir:\n" << walk_dir.direction() << '\n';

  // Triangle edges
  Line3d ab (a, b - a);
  Line3d ac (a, c - a);
  Line3d cb (c, b - c);

  double d1, d2, d3;
  intersectPlanes(walk_dir, ab, pp, d1);
  intersectPlanes(walk_dir, ac, pp, d2);
  intersectPlanes(walk_dir, cb, pp, d3);

  if (d1 < 0.0) d1 = std::numeric_limits<double>::max();
  if (d2 < 0.0) d2 = std::numeric_limits<double>::max();
  if (d3 < 0.0) d3 = std::numeric_limits<double>::max();

  Eigen::Vector3d intersect_at;
  std::vector<unsigned> neighbors;
  if (d1 < d2)
  {
    if (d3 < d1)
    {
      intersect_at = walk_dir.pointAt(d3);
      findTriangleNeighbors(triangle_indices_[close_pt.index*3 + 1], triangle_indices_[close_pt.index*3 + 2], neighbors);
    }
    else
    {
      intersect_at = walk_dir.pointAt(d1);
      findTriangleNeighbors(triangle_indices_[close_pt.index*3 + 0], triangle_indices_[close_pt.index*3 + 1], neighbors);
    }
    std::cout << "Neighbors: " << neighbors.size() << "\n";
  }
  else
  {
    if (d3 < d2)
    {
      intersect_at = walk_dir.pointAt(d3);
      findTriangleNeighbors(triangle_indices_[close_pt.index*3 + 1], triangle_indices_[close_pt.index*3 + 2], neighbors);
    }
    else
    {
      intersect_at = walk_dir.pointAt(d2);
      findTriangleNeighbors(triangle_indices_[close_pt.index*3 + 0], triangle_indices_[close_pt.index*3 + 2], neighbors);
    }
  }
  dir = intersect_at;
  // calculate position on new triangle
  Eigen::Vector3d cut_plane_normal = walk_dir.direction().cross(face_normals_[close_pt.index]);

  std::cout << "Cut dir n: " << cut_plane_normal << '\n';

  // Find neighbor
  if (neighbors.size() != 2) return Eigen::Vector3d(0,0,1);

  unsigned new_t = (neighbors[0] == close_pt.index) ? neighbors[1] : neighbors[0];

  Eigen::Vector3d new_n = face_normals_[new_t];

  std::cout << "new t n: " << new_n << '\n';

  Line3d new_t_line (intersect_at, new_n.cross(cut_plane_normal).normalized());

  std::cout << "New walk dir: " << new_n.cross(cut_plane_normal).normalized() << '\n';

  return new_t_line.pointAt(0.25);
}

bool teleop_tracking::Mesh::findTriangleNeighbors(unsigned idx1, unsigned idx2, std::vector<unsigned> &neighbors) const
{
  neighbors.clear();

  for (unsigned i = 0; i < triangle_indices_.size(); i += 3)
  {
    if (triangle_indices_[i+0] == idx1 ||
        triangle_indices_[i+1] == idx1 ||
        triangle_indices_[i+2] == idx1)
    {
      if (triangle_indices_[i+0] == idx2 ||
          triangle_indices_[i+1] == idx2 ||
          triangle_indices_[i+2] == idx2)
      {
        neighbors.push_back(i / 3);
      }
    }
  }

  return !neighbors.empty();
}

