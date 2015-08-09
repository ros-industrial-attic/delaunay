#include "combine_vertices.h"
#include <cassert>

unsigned appendUnique(EigenSTL::vector_Vector3d& vertices, const Eigen::Vector3d& other)
{
  for (unsigned i = 0; i < vertices.size(); i++)
  {
    if (vertices[i].isApprox(other)) return i;
  }

  vertices.push_back(other);
  return vertices.size() - 1;
}

Eigen::Vector3d toEigen(const teleop_tracking::StlLoader::Vector& v)
{
  return Eigen::Vector3d(v.x, v.y, v.z);
}

void teleop_tracking::combineVertices(const std::vector<teleop_tracking::StlLoader::Facet> &facets,
                                      EigenSTL::vector_Vector3d &vertices,
                                      EigenSTL::vector_Vector3d &face_normals,
                                      std::vector<unsigned> &face_indices)
{
  // The assumption is that these source are empty
  assert(vertices.empty());
  assert(face_normals.empty());
  assert(face_indices.empty());

  for (std::size_t i = 0; i < facets.size(); ++i)
  {
    const StlLoader::Facet& f = facets[i];

    face_normals.push_back(toEigen(f.normal).normalized());

    unsigned v0 = appendUnique(vertices, toEigen(f.vertices[0]));
    unsigned v1 = appendUnique(vertices, toEigen(f.vertices[1]));
    unsigned v2 = appendUnique(vertices, toEigen(f.vertices[2]));

    face_indices.push_back(v0);
    face_indices.push_back(v1);
    face_indices.push_back(v2);
  }
}
