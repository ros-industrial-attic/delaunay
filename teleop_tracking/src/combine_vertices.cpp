#include "combine_vertices.h"
#include <cassert>

unsigned appendUnique(EigenSTL::vector_Vector3f& vertices, const Eigen::Vector3f& other)
{
  for (unsigned i = 0; i < vertices.size(); i++)
  {
    if (vertices[i].isApprox(other)) return i;
  }

  vertices.push_back(other);
  return vertices.size() - 1;
}

Eigen::Vector3f toEigenf(const teleop_tracking::StlLoader::Vector& v)
{
  return Eigen::Vector3f(v.x, v.y, v.z);
}

Eigen::Vector3d toEigend(const teleop_tracking::StlLoader::Vector& v)
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

  EigenSTL::vector_Vector3f float_vector;

  for (std::size_t i = 0; i < facets.size(); ++i)
  {
    const StlLoader::Facet& f = facets[i];

    face_normals.push_back(toEigend(f.normal).normalized());

    unsigned v0 = appendUnique(float_vector, toEigenf(f.vertices[0]));
    unsigned v1 = appendUnique(float_vector, toEigenf(f.vertices[1]));
    unsigned v2 = appendUnique(float_vector, toEigenf(f.vertices[2]));

    face_indices.push_back(v0);
    face_indices.push_back(v1);
    face_indices.push_back(v2);
  }

  // copy the vector of single precision floats to double precision output
  for (std::size_t i = 0; i < float_vector.size(); ++i)
  {
    Eigen::Vector3d v = float_vector[i].cast<double>();
    vertices.push_back(v);
  }
}
