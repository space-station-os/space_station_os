#ifndef SSOS_EPS__EPS_MODEL_HPP_
#define SSOS_EPS__EPS_MODEL_HPP_

namespace ssos_eps
{

struct EpsConfig
{
  double solar_array_area_m2{100.0};
  double solar_array_efficiency{0.30};
  double load_demand_w{20000.0};
  double battery_capacity_wh{50000.0};
  double initial_battery_soc{0.80};
  double low_soc_threshold{0.20};
};

struct EpsInput
{
  double solar_flux_w_m2{1361.0};
  bool in_eclipse{false};
  double dt_s{1.0};
};

struct EpsState
{
  double generation_w{0.0};
  double load_demand_w{0.0};
  double battery_soc{0.0};
  double power_balance_w{0.0};
  bool healthy{true};
};

class EpsModel
{
public:
  explicit EpsModel(const EpsConfig & config = EpsConfig());

  EpsState update(const EpsInput & input);

  const EpsState & state() const;
  const EpsConfig & config() const;

private:
  EpsConfig config_;
  EpsState state_;
};

}  // namespace ssos_eps

#endif  // SSOS_EPS__EPS_MODEL_HPP_
