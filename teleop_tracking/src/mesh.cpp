#include "teleop_tracking/mesh.h"
#include "teleop_tracking/point_to_triangle.h"

#include "teleop_tracking/stl_loader.h"
#include "combine_vertices.h"

#include <ros/console.h>
#include <cmath>

typedef Eigen::ParametrizedLine<double, 3> Line3d;
typedef Eigen::Hyperplane<double, 3> Plane3d;

inline Eigen::Affine3d makePose(const Eigen::Vector3d& origin,
                                const Eigen::Vector3d& x,
                                const Eigen::Vector3d& y,
                                const Eigen::Vector3d& z)
{
  Eigen::Affine3d p = Eigen::Affine3d::Identity();
  p.translation() = origin;
  p.matrix().col(0).head<3>() = x;
  p.matrix().col(1).head<3>() = -y;
  p.matrix().col(2).head<3>() = z;
  return p;
}

inline Eigen::Affine3d makeArbitraryPose(const Eigen::Vector3d& origin,
                                         const Eigen::Vector3d& normal)
{
  Eigen::Hyperplane<double, 3> plane (normal, origin);
  Eigen::Vector3d x = Eigen::Vector3d::UnitX();
  if (x.dot(normal) > 0.99)
  {
    x = Eigen::Vector3d::UnitY();
  }

  x = plane.projection(origin + x) - origin;
  x.normalize();

  Eigen::Vector3d y = x.cross(normal);
  y.normalize();

  return makePose(origin, x, y, normal.normalized());
}

// Helper to find triangle intersection
struct IntersectionResult
{
  unsigned v1, v2;
  double dist;
};

IntersectionResult findIntersection(const Line3d& walk_dir, const teleop_tracking::TriangleRef& triangle, bool ignore_close = false)
{
  // Define sides
  // edge 0 : a->b
  // edge 1 : b->c
  // edge 2 : c->a
  Eigen::Vector3d edges[3] = {
    (triangle.b - triangle.a),
    (triangle.c - triangle.b),
    (triangle.a - triangle.c)
  };

  Plane3d planes[3] = {
    Plane3d(triangle.normal.cross(edges[0]), triangle.a),
    Plane3d(triangle.normal.cross(edges[1]), triangle.b),
    Plane3d(triangle.normal.cross(edges[2]), triangle.c)
  };

  double dists[3] = {
    walk_dir.intersection(planes[0]),
    walk_dir.intersection(planes[1]),
    walk_dir.intersection(planes[2])
  };

  std::cout << "DISTS: " << dists[0] << " " << dists[1] << " " << dists[2] << '\n';

  // Post process
  for (int i = 0; i < 3; ++i)
    if (dists[i] < (ignore_close ? 0.001 : 0.0)) dists[i] = std::numeric_limits<double>::max();

  // Find closest intersection in the forward direction
  int min_idx = 0;
  double min_d = dists[0];
  for (int i = 1; i < 3; ++i)
  {
    if (dists[i] < min_d)
    {
      min_d = dists[i];
      min_idx = i;
    }
  }

  // Populate result
  IntersectionResult result;
  if (min_idx == 0)
  {
    result.v1 = triangle.index*3 + 0;
    result.v2 = triangle.index*3 + 1;
  }
  else if (min_idx == 1)
  {
    result.v1 = triangle.index*3 + 1;
    result.v2 = triangle.index*3 + 2;
  }
  else
  {
    result.v1 = triangle.index*3 + 2;
    result.v2 = triangle.index*3 + 0;
  }

  result.dist = dists[min_idx];

  return result;
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

Eigen::Affine3d teleop_tracking::Mesh::walkTriangle2(const Eigen::Affine3d &start, unsigned start_triangle_idx, unsigned &new_triangle_idx, const Eigen::Vector2d &travel) const
{
  Eigen::Affine3d current_pose = start;
  double distance_to_travel = travel.norm();
  Eigen::Vector3d direction_of_travel = Eigen::Vector3d(travel(0), travel(1), 0).normalized();

  unsigned previous_triangle_idx = start_triangle_idx;
  unsigned current_triangle_idx = start_triangle_idx;


    // from a previous triangle to new one -> only have to check two components
  while (!walkTriangleFrom(current_pose,
                           current_triangle_idx,
                           previous_triangle_idx,
                           distance_to_travel,
                           direction_of_travel))
  {
    // Rotate to the new orientation
    std::cout << "STEPPING TRIANGLE " << distance_to_travel << " to go\n";
  }

  new_triangle_idx = current_triangle_idx;
  return current_pose;





  /*TriangleRef t = triangle(start_triangle_idx);

  // Calculate walk direction
  Eigen::Affine3d delta_pose = start * Eigen::Translation3d(travel(0), travel(1), 0);
  Line3d walk_dir (start.translation(), (delta_pose.translation() - start.translation()).normalized());
  // Find the closest intersection
  IntersectionResult int_result = findIntersection(walk_dir, t);
  Eigen::Vector3d intersect_point = walk_dir.pointAt(int_result.dist);
  // Check magnitude
  Eigen::Affine3d result_pose = start;
  double to_go = travel.norm();
  if (int_result.dist > to_go)
  {
    // No collision
    result_pose.translation() = walk_dir.pointAt(to_go);
    return result_pose;
  }
  else
  {
    // hit edge
    result_pose.translation() = intersect_point;
    // Find neighbor
    std::cout << "Edge encountered\n";
    unsigned next_triangle_idx;
    if (findNeighbor(triangle_indices_[int_result.v1],
                     triangle_indices_[int_result.v2],
                     start_triangle_idx, next_triangle_idx))
    {
      new_triangle_idx = next_triangle_idx;
      // there is a neighbor
      // Rotate to new pose
      std::cout << "Neighbor: " << next_triangle_idx << '\n';
      Eigen::Vector3d rot_axis = vertices_[triangle_indices_[int_result.v2]] - vertices_[triangle_indices_[int_result.v1]];
      std::cout << "Axis:\n" << rot_axis << '\n';
      double rot_amt = std::acos(face_normals_[start_triangle_idx].dot(face_normals_[next_triangle_idx]));
      std::cout << "Amt: " << rot_amt << '\n';
      Eigen::AngleAxisd rotation (rot_amt, rot_axis.normalized());
      Eigen::Affine3d rot_in_local_frame = result_pose.inverse() * rotation * result_pose;
      rot_in_local_frame.matrix().col(3) = Eigen::Vector4d::Zero();
      std::cout << "local rot:\n" << rot_in_local_frame.matrix() << '\n';
      std::cout << "AGH\n" << result_pose.matrix() << '\n';
      result_pose = result_pose * rot_in_local_frame;
      std::cout << "AGH2\n" << result_pose.matrix() << '\n';
      std::cout << "normals:\n" << face_normals_[next_triangle_idx] << "\n\n" << result_pose.matrix().col(2).head<3>() <<'\n';
      std::cout << "traveling " << to_go - int_result.dist << '\n';

      if (!result_pose.matrix().col(2).head<3>().isApprox(face_normals_[next_triangle_idx], 1e-4))
      {
        std::cout << "ROTATING AXES\n";
        Eigen::AngleAxisd back_the_other_way (-2.0 * rot_amt, rotation.axis());
        result_pose *= back_the_other_way;
      }

      result_pose *= Eigen::Translation3d((to_go - int_result.dist) * Eigen::Vector3d(travel(0), travel(1), 0).normalized());
      return result_pose;
    }
    return result_pose;
  }
  */
}

bool teleop_tracking::Mesh::walkTriangleFrom(Eigen::Affine3d &pose, unsigned &triangle_idx,
                                             unsigned &prev_triangle_idx,
                                             double &distance, const Eigen::Vector3d &direction) const
{
  TriangleRef t = triangle(triangle_idx);

  // Calculate walk direction
  Eigen::Affine3d delta_pose = pose * Eigen::Translation3d(distance * direction);
  Line3d walk_dir (pose.translation(),
                   (delta_pose.translation() - pose.translation()).normalized());

  // Find the closest intersection
  IntersectionResult intersection;
  if (triangle_idx == prev_triangle_idx)
  {
    // This is the first triangle and we need to test all the sides
    intersection = findIntersection(walk_dir, t);
  }
  else
  {
    intersection = findIntersection(walk_dir, t, true);
  }

  if (intersection.dist > distance)
  {
    // Does not collide with edge. Walk terminates in this triangle.
    pose.translation() = walk_dir.pointAt(distance);
    distance = 0.0;
    return true;
  }

  // We do collide. Necessary to look up next triangle and prepare for the transition.
  pose.translation() = walk_dir.pointAt(intersection.dist);
  distance -= intersection.dist;

  unsigned next_triangle_idx;
  if (findNeighbor(triangle_indices_[intersection.v1],
                   triangle_indices_[intersection.v2],
                   triangle_idx, next_triangle_idx))
  {
    // Calculate rotation
    Eigen::Vector3d rot_edge = vertices_[triangle_indices_[intersection.v2]] -
                               vertices_[triangle_indices_[intersection.v1]];

    double rot_amt = std::acos(face_normals_[triangle_idx].dot(face_normals_[next_triangle_idx]));

    Eigen::AngleAxisd rotation (rot_amt, rot_edge.normalized());
    Eigen::Affine3d rot_in_local_frame = pose.inverse() * rotation * pose;
    rot_in_local_frame.matrix().col(3) = Eigen::Vector4d::Zero();

    pose = pose * rot_in_local_frame;
    if (!pose.matrix().col(2).head<3>().isApprox(face_normals_[next_triangle_idx], 1e-4))
    {
      Eigen::AngleAxisd back_the_other_way (-2.0 * rot_amt, rotation.axis());
      pose *= back_the_other_way;
    }

    // Continue
    // There is a valid neighbor
    prev_triangle_idx = triangle_idx;
    triangle_idx = next_triangle_idx;
    return false;
  }
  else
  {
    // There is not a valid neighbor
    std::cerr << "No valid neighbor for triangle " << triangle_idx << '\n';
    return true;
  }
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

bool teleop_tracking::Mesh::findNeighbor(unsigned idx1, unsigned idx2,
                                         unsigned current_triangle_idx,
                                         unsigned &triangle_idx_out) const
{
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
        if (i/3 != current_triangle_idx)
        {
          triangle_idx_out = i/3;
          return true;
        }
      }
    }
  }

  return false;
}

