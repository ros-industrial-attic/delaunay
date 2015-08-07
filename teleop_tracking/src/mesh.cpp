#include "teleop_tracking/mesh.h"
#include "teleop_tracking/point_to_triangle.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

#include <ros/console.h>
#include <cmath>

static void errorOut(const std::string& msg, const std::string& name)
{
  std::ostringstream ss;
  ss << msg << ": " << name << std::endl;
  throw std::runtime_error(ss.str());
}

static double distSquared(const aiVector3D& m, const Eigen::Vector3d& v)
{
  return std::pow(v.x() - m.x , 2) + std::pow(v.y() - m.y , 2) + std::pow(v.z() - m.z , 2);
}

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

Eigen::Vector3d fromAiVector(const aiVector3D& a)
{
  return Eigen::Vector3d(a.x, a.y, a.z);
}

teleop_tracking::Mesh::Mesh(const std::string& meshfile)
{
  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(meshfile, aiProcess_JoinIdenticalVertices |
                                                     aiProcess_GenNormals |
                                                     aiProcess_GenSmoothNormals |
                                                     aiProcess_Triangulate);

  if (!scene)
  {
    errorOut("Could not load scene from file", meshfile);
  }

  if (!scene->HasMeshes())
  {
    errorOut("File does not contain meshes", meshfile);
  }

  if (scene->mNumMeshes > 1)
  {
    errorOut("File contains too many meshes", meshfile);
  }

  if (!scene->mMeshes[0]->mNormals)
  {
    errorOut("Mesh does not contain surface normals", meshfile);
  }


  const aiMesh* mesh = scene->mMeshes[0];

  if (mesh->mNumFaces == 0)
  {
    errorOut("Mesh has no faces", meshfile);
  }

  // Copy the vertex indices for triangles into local storage
  for (unsigned i = 0; i < mesh->mNumFaces; ++i)
  {
    if (mesh->mFaces[i].mNumIndices != 3)
    {
      errorOut("Face inside mesh was not a triangle", meshfile);
    }
    else
    {
      triangles_.insert(triangles_.end(), mesh->mFaces[i].mIndices, mesh->mFaces[i].mIndices + 3);
    }
  }

  // Copy/convert the vertices into local storage
  vertices_.reserve(mesh->mNumVertices);
  for (unsigned i = 0; i < mesh->mNumVertices; ++i)
  {
    vertices_.push_back(fromAiVector(mesh->mVertices[i]));
  }

  // Copy/conver the normals into local storage
  normals_.reserve(mesh->mNumVertices);
  for (unsigned i = 0; i < mesh->mNumVertices; ++i)
  {
    normals_.push_back(fromAiVector(mesh->mNormals[i]));
  }

  this->generateVerticeMap();
}

teleop_tracking::ClosestResult teleop_tracking::Mesh::closestPoint(const Eigen::Vector3d &source) const
{
  double min_dist = std::numeric_limits<double>::max();
  Eigen::Vector3d closest_point (0, 0, 0);
  unsigned int closest_triangle = 0;

  for (unsigned i = 0; i < triangles_.size(); i += 3)
  {
    // triangle built from 3 vertexes
    Eigen::Vector3d t[3] = {
      (vertices_[triangles_[i + 0]]),
      (vertices_[triangles_[i + 1]]),
      (vertices_[triangles_[i + 2]])
    };

    Eigen::Vector3d c = closesPointOnTriangle(t, source);
    double d = (source - c).norm();

    if (d < min_dist)
    {
      closest_triangle = i;
      min_dist = d;
      closest_point = c;
    }
  }

  ClosestResult result;
  result.index = closest_triangle;
  result.position = closest_point;
  return result;
}

void teleop_tracking::Mesh::debugInfo() const
{
  std::cout << "Vertices: " << vertices_.size() << '\n';
  std::cout << "Normals: " << normals_.size() << '\n';
  std::cout << "Triangle Indices: " << triangles_.size() << '\n';
  std::cout << "Triangles: " << triangles_.size() / 3 << '\n';
}

Eigen::Affine3d teleop_tracking::Mesh::closestPose(const Eigen::Vector3d &source) const
{
  ClosestResult r = closestPoint(source);
  Eigen::Vector3d normal = (normals_[triangles_[r.index]]);

  Eigen::Hyperplane<double, 3> plane (normal, r.position);
  // Unit X
  Eigen::Vector3d unit_x = Eigen::Vector3d::UnitX();
  Eigen::Vector3d x = plane.projection(unit_x).normalized();
  // Y Dir
  Eigen::Vector3d y = normal.cross(x);

  Eigen::Affine3d pose = makePose(r.position, x, y, normal);
  return pose;
}

Eigen::Affine3d teleop_tracking::Mesh::closestPose2(const Eigen::Vector3d &source, double r, const Eigen::Vector3d &norm) const
{
  ClosestResult res = closestPoint(source);
  Eigen::Vector3d nom_norm = (normals_[triangles_[res.index]]);
  Eigen::Vector3d normal = sphereNormalSearch(source, norm, r);
  std::cout << "NORMAL: " << normal << '\n';
  if (normal(0) == 0.0 && normal(1) == 0.0 && normal(2) == 0.0)
  {
    std::cout << "BACKUP";
    normal = nom_norm;
  }

  Eigen::Hyperplane<double, 3> plane (normal, res.position);
  // Unit X
  Eigen::Vector3d unit_x = Eigen::Vector3d::UnitX();
  Eigen::Vector3d x = plane.projection(unit_x).normalized();
  // Y Dir
  Eigen::Vector3d y = normal.cross(x);

  Eigen::Affine3d pose = makePose(res.position, x, y, normal);
  return pose;
}

/**
 * @brief teleop_tracking::Mesh::walkTriangles
 *
 * Walks the local triangles to follow a closed form solution
 *
 * @param start
 * @param dir
 * @param d
 * @return
 */
Eigen::Vector3d teleop_tracking::Mesh::walkTriangles(const Eigen::Vector3d &start, const Eigen::Vector3d &dir, double d) const
{
  typedef Eigen::ParametrizedLine<double, 3> Line3d;

  ClosestResult close_pt = closestPoint(start);
  Eigen::ParametrizedLine<double, 3> l (close_pt.position, normals_[triangles_[close_pt.index]]);

  // Triangle edges
  Eigen::Vector3d a = vertices_[triangles_[close_pt.index]];
  Eigen::Vector3d b = vertices_[triangles_[close_pt.index+1]];
  Eigen::Vector3d c = vertices_[triangles_[close_pt.index+2]];

  Eigen::Hyperplane<double, 3> pp (normals_[triangles_[close_pt.index]], a);

  // a-b
  Line3d ab (a, b - a);
  Line3d ac (a, c - a);
  Line3d cb (c, b - c);

  double d1, d2, d3;
  intersectPlanes(l, ab, pp, d1);
  intersectPlanes(l, ac, pp, d2);
  intersectPlanes(l, cb, pp, d3);

  if (d1 < 0.0) d1 = std::numeric_limits<double>::max();
  if (d2 < 0.0) d2 = std::numeric_limits<double>::max();
  if (d3 < 0.0) d3 = std::numeric_limits<double>::max();

  std::vector<unsigned> neighbors;
  findTriangleNeighbors(close_pt.index, close_pt.index+1, neighbors);
  std::cout << "NEIGHBORS: " << neighbors.size() << "\n";

  if (d1 < d2)
  {
    if (d3 < d1) return l.pointAt(d3);
    else return l.pointAt(d1);
  }
  else
  {
    if (d3 < d2) return l.pointAt(d3);
    else return l.pointAt(d2);
  }
}


Eigen::Vector3d teleop_tracking::Mesh::sphereNormalSearch(const Eigen::Vector3d &source, const Eigen::Vector3d& nom, double r) const
{
  Eigen::Vector3d final_normal (0,0,0);
  double total_weight = 0.0;

  for (unsigned i = 0; i < triangles_.size(); i += 3)
  {
    // triangle built from 3 vertexes
    Eigen::Vector3d t[3] = {
      (vertices_[triangles_[i + 0]]),
      (vertices_[triangles_[i + 1]]),
      (vertices_[triangles_[i + 2]])
    };

    Eigen::Vector3d c = closesPointOnTriangle(t, source);
    double d = (source - c).norm();

    const double MAX_WEIGHT = 100000.0;
    const double MIN_WEIGHT = 0.0;

    if (d <= r)
    {
      Eigen::Vector3d n = (normals_[triangles_[i]]);
      n.normalize();
      double dot = n.dot(nom);
      double theta = std::acos(dot);
      if (std::abs(theta) > 1.7)
      {
        std::cout << "Skipping " << theta << '\n';
        continue;
      }

      double inv_d;
      if (d < 0.000001)
      {
        inv_d = MAX_WEIGHT;
      }
      else
      {
        inv_d = 1.0 / d;
        inv_d = (inv_d > MAX_WEIGHT ? MAX_WEIGHT : inv_d);
      }


      total_weight += inv_d;
      final_normal += inv_d * n;
    }
  }

  if (total_weight == 0.0) return Eigen::Vector3d(0,0,0);

  final_normal /= total_weight;
  return final_normal.normalized();
}

bool hasIndex(const std::vector<unsigned>& indexes, unsigned idx)
{
  for (unsigned i = 0; i < indexes.size(); ++i)
    if (indexes[i] == idx) return true;
  return false;
}

bool teleop_tracking::Mesh::findTriangleNeighbors(unsigned idx1, unsigned idx2, std::vector<unsigned> &neighbors) const
{
  neighbors.clear();
  for (std::vector<unsigned>::size_type i = 0 ; i < triangles_.size(); i += 3)
  {
    // check for idx1
    if (triangles_[i] == idx1 || triangles_[i+1] == idx1 || triangles_[i+2] == idx1)
    {
      // check for idx2
      if (triangles_[i] == idx2 || triangles_[i+1] == idx2 || triangles_[i+2] == idx2)
      {
        neighbors.push_back(idx1 ^ idx2 ^ triangles_[i] ^ triangles_[i+1] ^ triangles_[i+2]);
      }
    }
  }

  return !neighbors.empty();
}

void teleop_tracking::Mesh::generateVerticeMap()
{
  std::vector<std::vector<unsigned> > vertex_map;
  vertex_map.resize(vertices_.size());

  for (unsigned i = 0; i < vertices_.size(); ++i)
  {
    for (unsigned j = 0; j < vertices_.size(); ++j)
    {
      if (vertices_[i].isApprox(vertices_[j]))
      {
        vertex_map[i].push_back(j);
      }
    }
  }

  vertex_map_ = vertex_map;
}
