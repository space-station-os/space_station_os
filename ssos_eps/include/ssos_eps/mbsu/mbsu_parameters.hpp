#ifndef SSOS_EPS__MBSU__MBSU_PARAMETERS_HPP_
#define SSOS_EPS__MBSU__MBSU_PARAMETERS_HPP_

// Parameters for one main bus switching unit (MBSU) routing primary power from
// its source channels to its DDCUs.

namespace ssos_eps
{
namespace mbsu
{

struct MbsuParams
{
  double min_source_voltage_v;    // channel voltage below which a source is unusable [V]
  double max_source_voltage_v;    // channel voltage above which a source is unusable [V]
  int min_sources;                // sources needed for a usable bus [-]
  int redundant_sources;          // sources needed before redundancy is considered lost [-]
};

MbsuParams default_mbsu_params();

}  // namespace mbsu
}  // namespace ssos_eps

#endif  // SSOS_EPS__MBSU__MBSU_PARAMETERS_HPP_
