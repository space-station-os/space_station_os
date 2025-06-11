#ifndef WHC_WASTE_TANK_HPP
#define WHC_WASTE_TANK_HPP

#include <rclcpp/rclcpp.hpp>
#include "space_station_eclss/srv/upa.hpp"
#include "std_msgs/msg/float64.hpp"
#include "space_station_eclss/msg/water_crew.hpp"

class WHCWasteTank : public rclcpp::Node {
public:
  WHCWasteTank();

private:
  
  rclcpp::Client<space_station_eclss::srv::Upa>::SharedPtr upa_client_;


  rclcpp::TimerBase::SharedPtr retry_timer_;
  rclcpp::TimerBase::SharedPtr urine_collection_timer_;
  rclcpp::TimerBase::SharedPtr dashboard_timer_;

  
  rclcpp::Publisher<space_station_eclss::msg::WaterCrew>::SharedPtr waste_status_pub_;


  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr grey_water_sub_;

 
  double urine_volume_;
  double pretreatment_volume_;
  double flush_volume_;
  double total_water_volume_;
  double tank_capacity_;
  double processing_threshold_;
  bool upa_available_;


  void simulate_urine_collection();
  void receive_grey_water(const std_msgs::msg::Float64::SharedPtr msg);
  void process_waste_transfer();
  void process_urine_response(rclcpp::Client<space_station_eclss::srv::Upa>::SharedFuture future);
  void retry_process_waste_transfer();
  void publish_status();
};

#endif  // WHC_WASTE_TANK_HPP
