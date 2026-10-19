#ifndef SSOS_GNC__NODES__THRUSTER_GEOMETRY_LOADER_HPP_
#define SSOS_GNC__NODES__THRUSTER_GEOMETRY_LOADER_HPP_

#include <string>

#include "ssos_gnc/flight/actuators/thruster_allocation.hpp"

namespace ssos_gnc
{

namespace nodes
{

struct LoadResult
{
  bool success{false};
  std::string message;
  std::size_t thruster_count{0};
};

class ThrusterGeometryLoader
{
public:
  ThrusterGeometryLoader();

  static bool is_thruster_link(const std::string & name);

  LoadResult load_urdf(const std::string & urdf_xml);

  LoadResult load_table(const std::string & yaml_path, const std::string & table_name);

  const flight::ThrusterGeometry & geometry() const {return geometry_;}
  bool ready() const {return geometry_.is_valid();}
  void reset();

private:
  flight::ThrusterGeometry geometry_;
};
}  // namespace nodes
}  // namespace ssos_gnc

#endif  // SSOS_GNC__NODES__THRUSTER_GEOMETRY_LOADER_HPP_
