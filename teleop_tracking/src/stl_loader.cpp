#include "teleop_tracking/stl_loader.h"

#include <stdexcept>
#include <fstream>
#include <stdint.h>
#include <cstring>

void readFloat(std::istream& is, float& f)
{
  is.read(reinterpret_cast<char*>(&f), 4);
}

void readVec(std::istream& is, teleop_tracking::StlLoader::Vector& v)
{
  readFloat(is, v.x);
  readFloat(is, v.y);
  readFloat(is, v.z);
}

teleop_tracking::StlLoader::StlLoader(const std::string &file)
{
  parseFile(file);
}

void teleop_tracking::StlLoader::parseFile(const std::string &file)
{
  std::ifstream ifile (file.c_str(), std::ios::binary);
  if (!ifile)
    throw std::runtime_error(file);

  ifile.exceptions ( std::ifstream::failbit | std::ifstream::badbit );

  // Read header
  char header_buff[80];
  ifile.read(header_buff, 80);

  // Read n triangles
  char n_buff[4];
  ifile.read(n_buff, 4);

  // copy into local var
  uint32_t n_triangles;
  std::memcpy(&n_triangles, n_buff, 4);

  // Iterate over triangles
  facets_.reserve(n_triangles);
  for (uint32_t i = 0; i < n_triangles; ++i)
  {
    Facet f;
    // read normal
    readVec(ifile, f.normal);
    readVec(ifile, f.vertices[0]);
    readVec(ifile, f.vertices[1]);
    readVec(ifile, f.vertices[2]);
    char endbuff[2];
    ifile.read(endbuff, 2);

    facets_.push_back(f);
  }

}
