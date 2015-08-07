#ifndef STL_LOADER_H
#define STL_LOADER_H

#include <vector>
#include <string>

namespace teleop_tracking
{

  class StlLoader
  {
  public:
    struct Vector
    {
      float x,y,z;
    };

    struct Facet
    {
      Vector normal;
      Vector vertices[3];
    };

    StlLoader(const std::string& file);

    const std::vector<Facet>& facets() const { return facets_; }

  private:
    void parseFile(const std::string& file);

    std::vector<Facet> facets_;
  };

} // end namespace teleop_tracking

#endif // STL_LOADER_H

