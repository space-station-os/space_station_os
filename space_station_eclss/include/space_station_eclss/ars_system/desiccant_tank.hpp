#ifndef space_station_eclss_DESICCANT_BEDS_HPP_
#define space_station_eclss_DESICCANT_BEDS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <space_station_eclss/srv/crew_quarters.hpp>
#include <space_station_eclss/msg/air_data.hpp>
#include <space_station_eclss/msg/cdra_status.hpp>
#include <mutex>
#include <memory>

class DesiccantBed1 : public rclcpp::Node
{
public:
  DesiccantBed1();

private:
  // --- Callback for service ---
  void handle_request(
    const std::shared_ptr<space_station_eclss::srv::CrewQuarters::Request> request,
    std::shared_ptr<space_station_eclss::srv::CrewQuarters::Response> response);

  // --- Internal processing ---
  void process_air();

  // --- Internal state ---
  bool is_active_;
  double co2_;
  double moisture_;
  double contaminants_;
  int retry_count_;
  const int max_retries_;
  
  // --- ROS Interfaces ---
  rclcpp::Service<space_station_eclss::srv::CrewQuarters>::SharedPtr bed1_service_;
  rclcpp::Client<space_station_eclss::srv::CrewQuarters>::SharedPtr adsorbent_client_;
  rclcpp::Publisher<space_station_eclss::msg::AirData>::SharedPtr air_quality_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- Mutex for safe data access ---
  std::mutex data_mutex_;
};


class DesiccantBed2 : public rclcpp::Node {
public:
  DesiccantBed2();

private:
  void handle_request(
    const std::shared_ptr<space_station_eclss::srv::CrewQuarters::Request> request,
    std::shared_ptr<space_station_eclss::srv::CrewQuarters::Response> response);

  void humidify_air();

  double co2_, contaminants_;
  double moisture_;
  bool is_active_;
  double humidification_rate_;
  std::mutex data_mutex_;

  rclcpp::Service<space_station_eclss::srv::CrewQuarters>::SharedPtr bed2_service_;
  rclcpp::Publisher<space_station_eclss::msg::CdraStatus>::SharedPtr cdra_status_pub_;
  rclcpp::Publisher<space_station_eclss::msg::AirData>::SharedPtr air_quality_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  space_station_eclss::msg::CdraStatus cdra_;
};

#endif  // space_station_eclss_DESICCANT_BEDS_HPP_
