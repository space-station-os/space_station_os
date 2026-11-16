#include "ssos_eps/mbsu/mbsu_parameters.hpp"

namespace ssos_eps
{
namespace mbsu
{

MbsuParams default_mbsu_params()
{
  MbsuParams p{};
  // Same channel acceptance as the legacy MBSU (> 120 V, two healthy sources).
  p.min_source_voltage_v = 120.0;
  p.max_source_voltage_v = 180.0;
  p.min_sources = 1;
  p.redundant_sources = 2;
  return p;
}

}  // namespace mbsu
}  // namespace ssos_eps
