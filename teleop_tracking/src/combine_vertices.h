#ifndef COMBINE_VERTICES_H
#define COMBINE_VERTICES_H

#include "teleop_tracking/stl_loader.h"
#include "Eigen/Dense"
#include "eigen_stl_containers/eigen_stl_vector_container.h"

namespace teleop_tracking
{

void combineVertices(const std::vector<StlLoader::Facet>& facets,
                     EigenSTL::vector_Vector3d& vertices,
                     EigenSTL::vector_Vector3d& face_normals,
                     std::vector<unsigned>& face_indices);

}

#endif

