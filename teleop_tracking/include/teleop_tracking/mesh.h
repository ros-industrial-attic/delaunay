#ifndef TELEOP_TRACKING_MESH_H
#define TELEOP_TRACKING_MESH_H

#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen_stl_containers/eigen_stl_vector_container.h>

namespace teleop_tracking
{

struct TrackPoint
{
  Eigen::Vector3d position;
  Eigen::Vector3d normal;
};

struct TrianglePosition
{
  Eigen::Vector3d position;
  unsigned int index;
};

struct TrianglePose
{
  Eigen::Affine3d pose;
  unsigned int index;
};

struct TriangleRef
{
  TriangleRef(unsigned index,
              const Eigen::Vector3d& a,
              const Eigen::Vector3d& b,
              const Eigen::Vector3d& c,
              const Eigen::Vector3d& normal)
    : a(a), b(b), c(c)
    , normal(normal)
    , index(index)
   {}

  const Eigen::Vector3d& a;
  const Eigen::Vector3d& b;
  const Eigen::Vector3d& c;
  const Eigen::Vector3d& normal;
  unsigned index;
};

class Mesh
{
public:
  // Can throw if mesh could not be found or is not valid
  Mesh(const std::string& meshfile);

  // Finds the physically closest position on mesh from given
  // source location
  TrianglePosition closestPoint(const Eigen::Vector3d& source) const;
  // Source should probably be a pose
  TrianglePose closestPose(const Eigen::Vector3d& source) const;

  // Debug
  void debugInfo() const;

  Eigen::Vector3d walkTriangles(const Eigen::Vector3d& start, Eigen::Vector3d& dir, double d) const;

  Eigen::Affine3d walkTriangle2(const Eigen::Affine3d& start, unsigned start_triangle_idx, unsigned &new_triangle_idx,
                                const Eigen::Vector2d& travel) const;

  bool walkTriangleFrom(Eigen::Affine3d& pose, unsigned& triangle_idx, unsigned& prev_triangle_idx,
                        double& distance, const Eigen::Vector3d& direction) const;

  TriangleRef triangle(unsigned idx) const
  {
    unsigned idx_base = idx*3;
    return TriangleRef(idx, vertices_[triangle_indices_[idx_base + 0]],
                            vertices_[triangle_indices_[idx_base + 1]],
                            vertices_[triangle_indices_[idx_base + 2]],
                            face_normals_[idx]);

  }

protected:
  bool findTriangleNeighbors(unsigned idx1, unsigned idx2, std::vector<unsigned>& neighbors) const;

  bool findNeighbor(unsigned vertex_idx1, unsigned vertex_idx2,
                    unsigned current_triangle_idx,
                    unsigned& triangle_idx_out) const;

private:
  EigenSTL::vector_Vector3d vertices_;
  EigenSTL::vector_Vector3d face_normals_;
  std::vector<unsigned int> triangle_indices_;
};

}

#endif
