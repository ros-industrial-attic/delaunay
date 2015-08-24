#include "teleop_tracking/mesh.h"
#include "teleop_tracking/point_to_triangle.h"

#include "teleop_tracking/stl_loader.h"
#include "combine_vertices.h"

#include <iostream>
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

    Eigen::Vector3d c = closestPointOnTriangle(t, source);
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
  std::cout << "Close point: " << p.position.transpose() << '\n';
  std::cout << "Index: " << p.index << '\n';
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


Eigen::Affine3d teleop_tracking::Mesh::walkTriangle2(const Eigen::Affine3d &start, unsigned start_triangle_idx, unsigned &new_triangle_idx, const Eigen::Vector2d &travel) const
{
  Eigen::Affine3d current_pose = start;
  double distance_to_travel = travel.norm();
  Eigen::Vector3d direction_of_travel = Eigen::Vector3d(travel(0), travel(1), 0).normalized();

  unsigned previous_triangle_idx = start_triangle_idx;
  unsigned current_triangle_idx = start_triangle_idx;
  Edge last_edge;


    // from a previous triangle to new one -> only have to check two components
  while (!walkTriangleFrom(current_pose,
                           current_triangle_idx,
                           previous_triangle_idx,
                           last_edge,
                           distance_to_travel,
                           direction_of_travel))
  {
    if (distance_to_travel != distance_to_travel)
    {
      std::cerr << "Nan detected. Returning immediately\n";
      return start;
    }
  }

  new_triangle_idx = current_triangle_idx;
  return current_pose;
}


bool teleop_tracking::Mesh::walkTriangleFrom(Eigen::Affine3d &pose,
                                             unsigned &triangle_idx,
                                             unsigned &prev_triangle_idx,
                                             Edge &last_edge,
                                             double &distance,
                                             const Eigen::Vector3d &direction) const
{
  TriangleRef t = triangle(triangle_idx);

  // Calculate walk direction
  Eigen::Affine3d delta_pose = pose * Eigen::Translation3d(distance * direction);
  Line3d walk_dir (pose.translation(),
                   (delta_pose.translation() - pose.translation()).normalized());

  // Find the closest intersection
  IntersectInput intersect_query;
  intersect_query.walk = walk_dir;
  IntersectOutput intersect_result;
  intersect_query.normal = face_normals_[triangle_idx];

  // If triangle_idx equals prev_triangle_idx, then we're on the first
  // iteration
  if (triangle_idx == prev_triangle_idx)
  {
    intersect_query.addEdge(triangle_indices_[triangle_idx*3+0],
                            triangle_indices_[triangle_idx*3+1]);
    intersect_query.addEdge(triangle_indices_[triangle_idx*3+1],
                            triangle_indices_[triangle_idx*3+2]);
    intersect_query.addEdge(triangle_indices_[triangle_idx*3+2],
                            triangle_indices_[triangle_idx*3+0]);
    intersect_result = findIntersect(intersect_query);
  }
  else
  {
    // Calculate the vertex of the current triangle that was NOT
    // part of the last edge intersection
    unsigned new_vertex = last_edge.v1 ^
        last_edge.v2 ^
        triangle_indices_[triangle_idx*3 + 0] ^
        triangle_indices_[triangle_idx*3 + 1] ^
        triangle_indices_[triangle_idx*3 + 2];

    intersect_query.addEdge(last_edge.v1, new_vertex);
    intersect_query.addEdge(last_edge.v2, new_vertex);
    intersect_result = findIntersect(intersect_query);
  }

  // Update the last edge
  last_edge = intersect_query.edges[intersect_result.edge_index];

  // If dist is set to it's numeric max, then the walk dir does not intersect
  // with ANY of the triangle edges and we're probably outside it - ERROR
  if (intersect_result.dist == std::numeric_limits<double>::max())
  {
    distance = 0.0;
    return true;
  }

  Plane3d home_plane (t.normal, t.a);
  if (intersect_result.dist > distance)
  {
    // Does not collide with edge. Walk terminates in this triangle.
    pose.translation() = home_plane.projection(walk_dir.pointAt(distance));
    distance = 0.0;
    return true;
  }

  // We do collide. Necessary to look up next triangle and prepare for the transition.
  pose.translation() = home_plane.projection(walk_dir.pointAt(intersect_result.dist));
  distance -= intersect_result.dist;

  unsigned next_triangle_idx;
  if (findNeighbor(last_edge.v1,
                   last_edge.v2,
                   triangle_idx,
                   next_triangle_idx))
  {
    // If the triangle we're moving onto has the same normal as the one we're coming
    // from, there's no need to do anything else
    if (face_normals_[triangle_idx].isApprox(face_normals_[next_triangle_idx]))
    {
      prev_triangle_idx = triangle_idx;
      triangle_idx = next_triangle_idx;
      return false;
    }

    // Transition logic -> could keep same relative orientation or could transition based
    // on world coordinates
    Eigen::Affine3d new_pose = transitionRelative(pose, triangle_idx, next_triangle_idx);

    // Optional post transition test to make sure our normals look reasonable
    if (!face_normals_[next_triangle_idx].isApprox(new_pose.matrix().col(2).head<3>(), 1e-4))
    {
      std::cerr << "Something went wrong while transitioning. Normals don't match.\n";
      return true;
    }

    pose = new_pose;
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

bool teleop_tracking::Mesh::intersectRay(const Eigen::Vector3d &origin,
                                         const Eigen::Vector3d &direction,
                                         teleop_tracking::TrianglePosition &output,
                                         double &dist) const
{
  unsigned best_idx = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (unsigned i = 0; i < triangle_indices_.size(); i += 3)
  {
    double dist;
    if (intersectRayTriangle(origin, direction,
                             vertices_[triangle_indices_[i+0]],
                             vertices_[triangle_indices_[i+1]],
                             vertices_[triangle_indices_[i+2]],
                             dist))
    {
      // The ray does intersect
      if (dist < min_dist)
      {
        min_dist = dist;
        best_idx = i / 3;
      }
    }
  }

  if (min_dist == std::numeric_limits<double>::max()) return false;
  else
  {
    output.index = best_idx;
    output.position = origin + direction * min_dist;
    dist = min_dist;
    return true;
  }
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

teleop_tracking::Mesh::IntersectOutput
teleop_tracking::Mesh::findIntersect(const teleop_tracking::Mesh::IntersectInput &input) const
{
  std::vector<Plane3d> planes;
  std::vector<double> distances;

  planes.reserve(input.edges.size());
  distances.reserve(input.edges.size());

  // std::cout << "Walk: " << input.walk.direction().transpose() << " " << input.walk.origin().transpose() << "\n";
  for (unsigned i = 0; i < input.edges.size(); ++i)
  {
    const Eigen::Vector3d& v2 = vertices_[input.edges[i].v2];
    const Eigen::Vector3d& v1 = vertices_[input.edges[i].v1];

    Eigen::Vector3d norm_edge = (v2 - v1).normalized();

    Plane3d p (input.normal.cross(norm_edge), v1);
    double d = input.walk.intersection(p);

    if (d != d)
    {
      // std::cout << "Dist edge: " << norm_edge.transpose() << " " << d << "\n";
      // std::cout << "v1: " << v1.transpose() << '\n';
      // std::cout << "v2: " << v2.transpose() << '\n';
    }

    planes.push_back(p);
    distances.push_back(d);
  }

  // Find closest non-negative edge
  double min_distance = std::numeric_limits<double>::max();
  unsigned min_edge = 0;

  for (unsigned i = 0; i < distances.size(); ++i)
  {
    // std::cout << "edge " << i << " " << distances[i] << '\n';
    if (distances[i] >= 0.0 && distances[i] < min_distance)
    {
      min_distance = distances[i];
      min_edge = i;
    }
  }

  // std::cout << "min edge: " << min_edge << " " << min_distance << '\n';

  IntersectOutput output;
  output.dist = min_distance;
  output.edge_index = min_edge;
  return output;
}

Eigen::Affine3d teleop_tracking::Mesh::transitionRelative(const Eigen::Affine3d &start,
                                                          unsigned from_idx,
                                                          unsigned to_idx) const
{
  const Eigen::Vector3d& from_normal = face_normals_[from_idx];
  const Eigen::Vector3d& to_normal = face_normals_[to_idx];

  Eigen::Vector3d rot_edge = from_normal.cross(to_normal).normalized();
  double rot_amt = std::acos(from_normal.dot(to_normal));
  if (rot_amt != rot_amt) rot_amt = 0.0; // test for NaN

  Eigen::AngleAxisd rotation (rot_amt, rot_edge);
  // Compute similarity transform
  Eigen::Affine3d rot_in_local_frame = Eigen::Affine3d::Identity();
  rot_in_local_frame.matrix().block<3,3>(0,0) = start.linear().inverse() * rotation.toRotationMatrix() * start.linear();
  rot_in_local_frame.matrix().col(3) = Eigen::Vector4d(0, 0, 0, 1);

  Eigen::Affine3d rotated_frame = start * rot_in_local_frame;

  if (!rotated_frame.matrix().col(2).head<3>().isApprox(to_normal, 1e-4))
  {
    Eigen::AngleAxisd other_rotation (-rot_amt, rot_edge);
    rot_in_local_frame.matrix().block<3,3>(0,0) = start.linear().inverse() *
                                                  other_rotation.toRotationMatrix() * start.linear();
    rot_in_local_frame.matrix().col(3) = Eigen::Vector4d(0, 0, 0, 1);

    rotated_frame = start * rot_in_local_frame;
    std::cout << "CORRECTION\n";
  }

  return rotated_frame;
}

