#ifndef TELEOP_TRACKING_MESH_H
#define TELEOP_TRACKING_MESH_H

#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <assimp/scene.h>

namespace teleop_tracking
{
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > VecVec3d;

struct TrackPoint
{
  Eigen::Vector3d position;
  Eigen::Vector3d normal;
};

struct ClosestResult
{
  Eigen::Vector3d position;
  unsigned int index;
};

class Mesh
{
public:
  // Can throw if mesh could not be found or is not valid
  Mesh(const std::string& meshfile);

  // Finds the physically closest position on mesh from given
  // source location
  ClosestResult closestPoint(const Eigen::Vector3d& source) const;

  // Debug
  void debugInfo() const;

  // Deprecated numerical methods
  Eigen::Affine3d closestPose(const Eigen::Vector3d& source) const;
  Eigen::Affine3d closestPose2(const Eigen::Vector3d& source, double r, const Eigen::Vector3d& norm) const;

  Eigen::Vector3d walkTriangles(const Eigen::Vector3d& start, const Eigen::Vector3d& dir, double d) const;

  Eigen::Vector3d sphereNormalSearch(const Eigen::Vector3d& source,const Eigen::Vector3d& nom, double r) const;

  const VecVec3d& normals() const { return normals_; }
  const VecVec3d& vertices() const { return vertices_; }

protected:

  bool findTriangleNeighbors(unsigned idx1, unsigned idx2, std::vector<unsigned>& neighbors) const;

  void generateVerticeMap();

private:
  VecVec3d vertices_;
  VecVec3d normals_;
  std::vector<unsigned int> triangles_;
  std::vector<std::vector<unsigned> > vertex_map_;
};

}

#endif
