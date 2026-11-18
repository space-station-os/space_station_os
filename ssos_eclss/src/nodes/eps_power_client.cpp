#include "ssos_eclss/nodes/eps_power_client.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <utility>

namespace ssos_eclss
{
namespace nodes
{

namespace
{
using Load = space_station_interfaces::srv::Load;

constexpr double kStatusTimeoutS = 5.0;     // EPS considered gone after this
constexpr double kRefreshS = 30.0;          // re-assert the request this often
constexpr double kRelistS = 5.0;            // re-request if EPS no longer lists us
constexpr double kMinChangeW = 50.0;
constexpr double kMinChangeFraction = 0.05;
constexpr double kMinRequestW = 1.0;        // EPS rejects non-positive requests
}  // namespace

EpsPowerClient::EpsPowerClient(rclcpp_lifecycle::LifecycleNode * node, std::string load_id,
                               int priority)
: node_(node), load_id_(std::move(load_id)), priority_(priority)
{}

void EpsPowerClient::start(bool enabled)
{
  enabled_ = enabled;
  requested_w_ = -1.0;
  request_pending_ = false;
  have_status_ = false;
  listed_ = false;
  shed_ = false;
  rejected_ = false;
  if (!enabled_) {
    return;
  }
  client_ = node_->create_client<Load>("/ssos/eps/load_request");
  sub_ = node_->create_subscription<space_station_interfaces::msg::LoadStatusArray>(
    "/ssos/eps/loads", 10, std::bind(&EpsPowerClient::on_status, this, std::placeholders::_1));
}

void EpsPowerClient::stop()
{
  // Hand the power back so EPS does not keep it allocated to a unit that is gone.
  if (client_ && requested_w_ > 0.0 && client_->service_is_ready()) {
    auto req = std::make_shared<Load::Request>();
    req->load_id = load_id_;
    req->action = Load::Request::ACTION_RELEASE;
    client_->async_send_request(req);
  }
  requested_w_ = -1.0;
  client_.reset();
  sub_.reset();
}

bool EpsPowerClient::eps_present() const
{
  return enabled_ && have_status_ &&
         (steady_.now() - last_status_).seconds() < kStatusTimeoutS;
}

void EpsPowerClient::on_status(
  const space_station_interfaces::msg::LoadStatusArray::SharedPtr msg)
{
  have_status_ = true;
  last_status_ = steady_.now();
  listed_ = false;
  for (const auto & load : msg->loads) {
    if (load.load_id == load_id_) {
      listed_ = true;
      shed_ = load.shed;
      served_w_ = load.served_power;
      status_requested_w_ = load.requested_power;
      break;
    }
  }
}

void EpsPowerClient::update_demand(double power_w)
{
  demand_w_ = std::max(power_w, 0.0);
  if (!enabled_ || !client_ || request_pending_ || !client_->service_is_ready()) {
    return;
  }
  const double target = std::max(demand_w_, kMinRequestW);
  const double since = (steady_.now() - last_request_).seconds();
  const bool changed = requested_w_ < 0.0 ||
    std::abs(target - requested_w_) > std::max(kMinChangeW, kMinChangeFraction * requested_w_);
  const bool refresh = since > kRefreshS;
  const bool relist = eps_present() && !listed_ && since > kRelistS;
  if (changed || refresh || relist) {
    send_request(target);
  }
}

void EpsPowerClient::send_request(double power_w)
{
  auto req = std::make_shared<Load::Request>();
  req->load_id = load_id_;
  req->action = Load::Request::ACTION_REQUEST;
  req->power_w = power_w;
  req->priority = static_cast<uint8_t>(std::clamp(priority_, 0, 255));
  req->ddcu_index = -1;
  request_pending_ = true;
  requested_w_ = power_w;
  last_request_ = steady_.now();
  client_->async_send_request(
    req, [this](rclcpp::Client<Load>::SharedFuture future) {
      request_pending_ = false;
      const auto res = future.get();
      rejected_ = !res->success;
      reject_reason_ = res->success ? "" : res->message;
      if (rejected_) {
        RCLCPP_WARN(node_->get_logger(), "EPS rejected load '%s': %s",
                    load_id_.c_str(), res->message.c_str());
      }
    });
}

double EpsPowerClient::powered_fraction() const
{
  if (!eps_present()) {
    return 1.0;
  }
  if (listed_) {
    if (shed_) {
      return 0.0;
    }
    return status_requested_w_ > 0.0 ?
           std::clamp(served_w_ / status_requested_w_, 0.0, 1.0) : 1.0;
  }
  // Not (yet) registered: unpowered only if EPS turned the request down.
  return rejected_ ? 0.0 : 1.0;
}

std::string EpsPowerClient::status() const
{
  if (!enabled_) {
    return "EPS coupling off";
  }
  if (!eps_present()) {
    return "EPS not running";
  }
  if (listed_ && shed_) {
    return "shed by EPS";
  }
  if (!listed_ && rejected_) {
    return "EPS rejected load: " + reject_reason_;
  }
  const double f = powered_fraction();
  return f > 0.999 ? "powered" : "brown-out (" + std::to_string(static_cast<int>(f * 100)) +
         "% power)";
}

}  // namespace nodes
}  // namespace ssos_eclss
