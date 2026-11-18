#ifndef SSOS_ECLSS__NODES__EPS_POWER_CLIENT_HPP_
#define SSOS_ECLSS__NODES__EPS_POWER_CLIENT_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "space_station_interfaces/msg/load_status_array.hpp"
#include "space_station_interfaces/srv/load.hpp"

// Power supply for one ECLSS unit from ssos_eps. The unit reports the power it
// needs each step; this requests (and updates) a named load through
// /ssos/eps/load_request and follows /ssos/eps/loads to tell the unit how much
// of that power is actually delivered. When EPS is not running, or coupling is
// disabled, the unit is treated as fully powered so ECLSS still runs on its own.

namespace ssos_eclss
{
namespace nodes
{

class EpsPowerClient
{
public:
  EpsPowerClient(rclcpp_lifecycle::LifecycleNode * node, std::string load_id, int priority);

  /// Create the service client and status subscription (on_configure).
  void start(bool enabled);
  /// Release the load and drop the client and subscription (on_cleanup).
  void stop();

  /// Report the power the unit needs at full function this step [W].
  void update_demand(double power_w);

  /// Share of the demand EPS delivers [0, 1].
  double powered_fraction() const;
  bool powered() const {return powered_fraction() > 0.999;}
  /// True if EPS is running and answering for this load.
  bool eps_present() const;
  /// Short description of the supply state for diagnostics and heartbeats.
  std::string status() const;

  const std::string & load_id() const {return load_id_;}
  double demand_w() const {return demand_w_;}

private:
  void on_status(const space_station_interfaces::msg::LoadStatusArray::SharedPtr msg);
  void send_request(double power_w);

  rclcpp_lifecycle::LifecycleNode * node_;
  std::string load_id_;
  int priority_;
  bool enabled_{false};

  rclcpp::Client<space_station_interfaces::srv::Load>::SharedPtr client_;
  rclcpp::Subscription<space_station_interfaces::msg::LoadStatusArray>::SharedPtr sub_;
  rclcpp::Clock steady_{RCL_STEADY_TIME};

  double demand_w_{0.0};
  double requested_w_{-1.0};
  bool request_pending_{false};
  rclcpp::Time last_request_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_status_{0, 0, RCL_STEADY_TIME};
  bool have_status_{false};

  bool listed_{false};
  bool shed_{false};
  double served_w_{0.0};
  double status_requested_w_{0.0};
  bool rejected_{false};
  std::string reject_reason_;
};

}  // namespace nodes
}  // namespace ssos_eclss

#endif  // SSOS_ECLSS__NODES__EPS_POWER_CLIENT_HPP_
