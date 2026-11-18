#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "space_station_interfaces/msg/load_status_array.hpp"
#include "space_station_interfaces/srv/load.hpp"

#include "ssos_eclss/nodes/eps_power_client.hpp"

// EpsPowerClient against a stand-in for eps_node: a /ssos/eps/load_request
// service that records requests and a /ssos/eps/loads publisher the test
// drives by hand.

using namespace std::chrono_literals;
using space_station_interfaces::msg::LoadStatus;
using space_station_interfaces::msg::LoadStatusArray;
using space_station_interfaces::srv::Load;
using ssos_eclss::nodes::EpsPowerClient;

class EpsPowerClientTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    host_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("power_client_host");
    client_ = std::make_unique<EpsPowerClient>(host_.get(), "eclss_ogs", 1);
    eps_ = std::make_shared<rclcpp::Node>("fake_eps");
    executor_.add_node(host_->get_node_base_interface());
    executor_.add_node(eps_);
  }

  void TearDown() override
  {
    client_->stop();
    executor_.remove_node(eps_);
    executor_.remove_node(host_->get_node_base_interface());
  }

  void start_eps(bool accept = true)
  {
    status_pub_ = eps_->create_publisher<LoadStatusArray>("/ssos/eps/loads", 10);
    service_ = eps_->create_service<Load>(
      "/ssos/eps/load_request",
      [this, accept](const Load::Request::SharedPtr req, Load::Response::SharedPtr res) {
        requests_.push_back(*req);
        res->success = accept;
        res->message = accept ? "ok" : "no DDCU headroom";
        res->granted_power_w = accept ? req->power_w : 0.0;
        res->ddcu_index = accept ? 2 : -1;
      });
  }

  void publish_status(bool listed, double requested, double served, bool shed)
  {
    LoadStatusArray msg;
    LoadStatus other;
    other.load_id = "comms";
    other.requested_power = 1800.0;
    other.served_power = 1800.0;
    msg.loads.push_back(other);
    if (listed) {
      LoadStatus mine;
      mine.load_id = "eclss_ogs";
      mine.priority = 1;
      mine.requested_power = requested;
      mine.served_power = served;
      mine.shed = shed;
      msg.loads.push_back(mine);
    }
    status_pub_->publish(msg);
  }

  // Spin until pred holds or the timeout runs out.
  template<typename Pred>
  bool spin_until(Pred pred, std::chrono::milliseconds timeout = 3000ms)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some();
      if (pred()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  void wait_for_service()
  {
    ASSERT_TRUE(spin_until([this] {
        client_->update_demand(demand_);
        return !requests_.empty();
      }));
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr host_;
  std::unique_ptr<EpsPowerClient> client_;
  rclcpp::Node::SharedPtr eps_;
  rclcpp::Publisher<LoadStatusArray>::SharedPtr status_pub_;
  rclcpp::Service<Load>::SharedPtr service_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::vector<Load::Request> requests_;
  double demand_{1200.0};
};

TEST_F(EpsPowerClientTest, FullyPoweredWithoutEps)
{
  client_->start(true);
  client_->update_demand(1200.0);
  EXPECT_FALSE(client_->eps_present());
  EXPECT_DOUBLE_EQ(client_->powered_fraction(), 1.0);
  EXPECT_TRUE(client_->powered());
  EXPECT_EQ(client_->status(), "EPS not running");
}

TEST_F(EpsPowerClientTest, DisabledNeverRequests)
{
  start_eps();
  client_->start(false);
  spin_until([this] {client_->update_demand(1200.0); return false;}, 300ms);
  EXPECT_TRUE(requests_.empty());
  publish_status(true, 1200.0, 0.0, true);
  spin_until([] {return false;}, 200ms);
  EXPECT_DOUBLE_EQ(client_->powered_fraction(), 1.0);
  EXPECT_EQ(client_->status(), "EPS coupling off");
}

TEST_F(EpsPowerClientTest, RequestsItsLoadWithPriority)
{
  start_eps();
  client_->start(true);
  wait_for_service();
  ASSERT_EQ(requests_.size(), 1u);
  EXPECT_EQ(requests_[0].load_id, "eclss_ogs");
  EXPECT_EQ(requests_[0].action, Load::Request::ACTION_REQUEST);
  EXPECT_EQ(requests_[0].priority, 1);
  EXPECT_EQ(requests_[0].ddcu_index, -1);
  EXPECT_DOUBLE_EQ(requests_[0].power_w, 1200.0);
}

TEST_F(EpsPowerClientTest, SmallChangesDoNotResendLargeOnesDo)
{
  start_eps();
  client_->start(true);
  wait_for_service();
  publish_status(true, 1200.0, 1200.0, false);
  spin_until([this] {client_->update_demand(1230.0); return false;}, 300ms);
  EXPECT_EQ(requests_.size(), 1u);

  ASSERT_TRUE(spin_until([this] {
      client_->update_demand(1500.0);
      return requests_.size() == 2u;
    }));
  EXPECT_DOUBLE_EQ(requests_.back().power_w, 1500.0);
}

TEST_F(EpsPowerClientTest, ZeroDemandStillHoldsAMinimalLoad)
{
  start_eps();
  client_->start(true);
  demand_ = 0.0;
  wait_for_service();
  EXPECT_GT(requests_[0].power_w, 0.0);
}

TEST_F(EpsPowerClientTest, FollowsServedShareFromStatus)
{
  start_eps();
  client_->start(true);
  wait_for_service();

  publish_status(true, 1200.0, 1200.0, false);
  ASSERT_TRUE(spin_until([this] {return client_->eps_present();}));
  EXPECT_TRUE(client_->powered());
  EXPECT_EQ(client_->status(), "powered");

  publish_status(true, 1200.0, 600.0, false);
  ASSERT_TRUE(spin_until([this] {return client_->powered_fraction() < 0.9;}));
  EXPECT_NEAR(client_->powered_fraction(), 0.5, 1e-9);
  EXPECT_FALSE(client_->powered());
  EXPECT_EQ(client_->status().rfind("brown-out", 0), 0u);

  publish_status(true, 1200.0, 0.0, true);
  ASSERT_TRUE(spin_until([this] {return client_->powered_fraction() == 0.0;}));
  EXPECT_EQ(client_->status(), "shed by EPS");

  publish_status(true, 1200.0, 1200.0, false);
  ASSERT_TRUE(spin_until([this] {return client_->powered();}));
}

TEST_F(EpsPowerClientTest, RejectedRequestLeavesUnitUnpowered)
{
  start_eps(false);
  client_->start(true);
  wait_for_service();
  publish_status(false, 0.0, 0.0, false);
  ASSERT_TRUE(spin_until([this] {return client_->powered_fraction() == 0.0;}));
  EXPECT_EQ(client_->status(), "EPS rejected load: no DDCU headroom");
}

TEST_F(EpsPowerClientTest, StopReleasesTheLoad)
{
  start_eps();
  client_->start(true);
  wait_for_service();
  client_->stop();
  ASSERT_TRUE(spin_until([this] {return requests_.size() == 2u;}));
  EXPECT_EQ(requests_.back().load_id, "eclss_ogs");
  EXPECT_EQ(requests_.back().action, Load::Request::ACTION_RELEASE);
}

TEST_F(EpsPowerClientTest, ReRequestsWhenEpsForgetsTheLoad)
{
  start_eps();
  client_->start(true);
  wait_for_service();
  // EPS restarted and lists only its own loads: the client asks again.
  ASSERT_TRUE(spin_until([this] {
      publish_status(false, 0.0, 0.0, false);
      client_->update_demand(1200.0);
      return requests_.size() >= 2u;
    }, 8000ms));
  EXPECT_EQ(requests_.back().load_id, "eclss_ogs");
}
