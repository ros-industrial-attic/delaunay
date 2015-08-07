#include <iostream>
#include "teleop_tracking/stl_loader.h"

std::ostream& operator<<(std::ostream& os, const teleop_tracking::StlLoader::Facet& f)
{
  os << f.normal.x << " " <<f.normal.y << " " << f.normal.z << "\n";
  os << f.vertices[0].x << " " <<f.vertices[0].y << " " << f.vertices[0].z << "\n";
  os << f.vertices[1].x << " " <<f.vertices[1].y << " " << f.vertices[1].z << "\n";
  os << f.vertices[2].x << " " <<f.vertices[2].y << " " << f.vertices[2].z << "\n";
  return os;
}

int main(int argc, char** argv)
{
  using teleop_tracking::StlLoader;

  if (argc <= 1) return -1;

  StlLoader loader (argv[1]);

  std::cout << loader.facets().size() << "\n";

  for (std::size_t i = 0; i < loader.facets().size(); ++i)
  {
    const StlLoader::Facet& f = loader.facets()[i];
    std::cout << f << "\n";
  }
}
