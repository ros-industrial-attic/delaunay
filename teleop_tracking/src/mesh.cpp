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
    (triangle.b - triangle.a).normalized(),
    (triangle.c - triangle.b).normalized(),
    (triangle.a - triangle.c).normalized()
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
    // Rotate to the new orientation
    std::cout << "STEPPING TRIANGLE " << distance_to_travel << " to go\n";
    if (distance_to_travel != distance_to_travel)
    {
      ROS_WARN("NAN");
      return start;
    }
  }

  new_triangle_idx = current_triangle_idx;
  return current_pose;
}


//static Eigen::Affine3d toDifferentFrame(const )

bool teleop_tracking::Mesh::walkTriangleFrom(Eigen::Affine3d &pose, unsigned &triangle_idx,
                                             unsigned &prev_triangle_idx, Edge &last_edge,
                                             double &distance,
                                             const Eigen::Vector3d &direction) const
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
    last_edge.v1 = triangle_indices_[intersection.v1];
    last_edge.v2 = triangle_indices_[intersection.v2];
  }
  else
  {
    // Formulate edge query
    unsigned new_vertex = last_edge.v1 ^ last_edge.v2 ^
        triangle_indices_[triangle_idx*3 + 0] ^
        triangle_indices_[triangle_idx*3 + 1] ^
        triangle_indices_[triangle_idx*3 + 2];

    IntersectInput in;
    in.walk = walk_dir;
    in.normal = face_normals_[triangle_idx];
    Edge e1;
    e1.v1 = last_edge.v1;
    e1.v2 = new_vertex;
    in.edges.push_back(e1);
    e1.v1 = last_edge.v2;
    in.edges.push_back(e1);

    IntersectOutput out = findIntersect(in);
    intersection.dist = out.dist;

    last_edge.v1 = in.edges[out.edge_index].v1;
    last_edge.v2 = in.edges[out.edge_index].v2;

    if (triangle_indices_[triangle_idx*3+0] == last_edge.v1)
    {
      intersection.v1 = triangle_idx*3+0;
    } else if (triangle_indices_[triangle_idx*3+1] == last_edge.v1)
    {
      intersection.v1 = triangle_idx*3+1;
    } else
    {
      intersection.v1 = triangle_idx*3+2;
    }

    if (triangle_indices_[triangle_idx*3+0] == last_edge.v2)
    {
      intersection.v2 = triangle_idx*3+0;
    } else if (triangle_indices_[triangle_idx*3+1] == last_edge.v2)
    {
      intersection.v2 = triangle_idx*3+1;
    } else
    {
      intersection.v2 = triangle_idx*3+2;
    }

    assert(last_edge.v1 != last_edge.v2);
    assert(intersection.v1 != intersection.v2);
  }

  Plane3d home_plane (t.normal, t.a);

  if (intersection.dist == std::numeric_limits<double>::max())
  {
    distance = 0.0;
    return true;
  }

  if (intersection.dist > distance)
  {
    // Does not collide with edge. Walk terminates in this triangle.
    pose.translation() = home_plane.projection(walk_dir.pointAt(distance));
    distance = 0.0;
    return true;
  }

  // We do collide. Necessary to look up next triangle and prepare for the transition.
  pose.translation() = home_plane.projection(walk_dir.pointAt(intersection.dist));
  distance -= intersection.dist;

  unsigned next_triangle_idx;
  if (findNeighbor(triangle_indices_[intersection.v1],
                   triangle_indices_[intersection.v2],
                   triangle_idx, next_triangle_idx))
  {
    // Calculate rotation
    Eigen::Vector3d rot_edge = vertices_[triangle_indices_[intersection.v2]] -
                               vertices_[triangle_indices_[intersection.v1]];

    std::cout << "V1: " << vertices_[triangle_indices_[intersection.v1]].transpose() << '\n';
    std::cout << "V2: " << vertices_[triangle_indices_[intersection.v2]].transpose() << '\n';

    if (face_normals_[triangle_idx].isApprox(face_normals_[next_triangle_idx]))
    {
      prev_triangle_idx = triangle_idx;
      triangle_idx = next_triangle_idx;
      return false;
    }
    rot_edge = face_normals_[triangle_idx].cross(face_normals_[next_triangle_idx]);

    double rot_amt = std::acos(face_normals_[triangle_idx].dot(face_normals_[next_triangle_idx]));
    if (rot_amt != rot_amt) rot_amt = 0.0;
    std::cout << "From Idx: " << triangle_idx << " to " << next_triangle_idx << '\n';
    std::cout << "From N: " << face_normals_[triangle_idx].transpose() << "\nTo N: " << face_normals_[next_triangle_idx].transpose() <<'\n';
    std::cout << "Rot amt: " << rot_amt << " about " << rot_edge.normalized().transpose() << '\n';

    Eigen::AngleAxisd rotation (rot_amt, rot_edge.normalized());
    Eigen::Affine3d rot_in_local_frame = Eigen::Affine3d::Identity();
    rot_in_local_frame.matrix().block<3,3>(0,0) = pose.linear().inverse() * rotation.toRotationMatrix() * pose.linear();
    rot_in_local_frame.matrix().col(3) = Eigen::Vector4d(0, 0, 0, 1);

    std::cout << "Local rot:\n" << rot_in_local_frame.matrix() << '\n';
    std::cout << "Prev pose:\n" << pose.matrix() << '\n';

    Eigen::Affine3d rotated_frame = pose * rot_in_local_frame;
    std::cout << "After initial:\n" << rotated_frame.matrix() << '\n';
    if (!rotated_frame.matrix().col(2).head<3>().isApprox(face_normals_[next_triangle_idx], 1e-4))
    {
      Eigen::AngleAxisd other_rotation (-rot_amt, rot_edge.normalized());
      rot_in_local_frame.matrix().block<3,3>(0,0) = pose.linear().inverse() *
                                                    other_rotation.toRotationMatrix() * pose.linear();
      rot_in_local_frame.matrix().col(3) = Eigen::Vector4d(0, 0, 0, 1);

      rotated_frame = pose * rot_in_local_frame;
      std::cout << "Corrected:\n" << rotated_frame.matrix() << "\n";
    }


    if (!rotated_frame.matrix().col(2).head<3>().isApprox(face_normals_[next_triangle_idx], 1e-4))
    {
      ROS_WARN("Not right");
      return true;
    }
    else
      pose = rotated_frame;

    std::cout << "Final pose:\n" << pose.matrix() << '\n';

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

  std::cout << "Walk: " << input.walk.direction().transpose() << " " << input.walk.origin().transpose() << "\n";
  for (unsigned i = 0; i < input.edges.size(); ++i)
  {
    const Eigen::Vector3d& v2 = vertices_[input.edges[i].v2];
    const Eigen::Vector3d& v1 = vertices_[input.edges[i].v1];

    Eigen::Vector3d norm_edge = (v2 - v1).normalized();

    Plane3d p (input.normal.cross(norm_edge), v1);
    double d = input.walk.intersection(p);

    if (d != d)
    {
      std::cout << "Dist edge: " << norm_edge.transpose() << " " << d << "\n";
      std::cout << "v1: " << v1.transpose() << '\n';
      std::cout << "v2: " << v2.transpose() << '\n';
    }

    planes.push_back(p);
    distances.push_back(d);
  }

  // Find closest non-negative edge
  double min_distance = std::numeric_limits<double>::max();
  unsigned min_edge = 0;

  for (unsigned i = 0; i < distances.size(); ++i)
  {
    std::cout << "edge " << i << " " << distances[i] << '\n';
    if (distances[i] >= 0.0 && distances[i] < min_distance)
    {
      min_distance = distances[i];
      min_edge = i;
    }
  }

  std::cout << "min edge: " << min_edge << " " << min_distance << '\n';

  IntersectOutput output;
  output.dist = min_distance;
  output.edge_index = min_edge;
  return output;
}

