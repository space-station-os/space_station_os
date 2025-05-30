#ifndef space_station_eclss_SABATIER_PUBLISHER_HPP_
#define space_station_eclss_SABATIER_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include "space_station_eclss/msg/sabatier.hpp"
#include "space_station_eclss/msg/air_data.hpp"
#include "space_station_eclss/msg/electrolysis.hpp"
#include "std_msgs/msg/float64.hpp" 
#include <chrono>
#include <memory>
#include <string>
#include <vector>

class Sabatier : public rclcpp::Node {
public:
  Sabatier();

private:
  void process_air_data(const space_station_eclss::msg::AirData &msg);
  void process_hydrogen_data(const space_station_eclss::msg::Electrolysis &msg);
  void compute_reaction();
  void run_reactor();
  void publish_sabatier_outputs();
  bool safety_check();
  double pid_temperature(double desired, double current);
  double pid_pressure(double desired, double current);
  double pd_ghsv(double desired, double current); 
  
  // Air mixture parameters
  double co2_mass_;
  double h2_mass_;
  double moisture_content_;
  double contaminants_;
  double dew_point_;
  double total_air_mass_;

  // Sabatier reaction parameters
  double reactor_temp_;
  double desired_temp_;
  double reactor_pressure_;
  double desired_pressure_;
  double h2_co2_ratio_;
  double ghsv_;
  double desired_ghsv_;
  double total_inlet_flow_;
  double methane_yield_;
  double water_yield_;
  double reaction_efficiency_;

  // PID control variables
  double temp_integral_;
  double temp_previous_error_;
  double pressure_integral_;
  double pressure_previous_error_;
  double ghsv_previous_error_;
  double integral_temp_error_;
  double integral_pressure_error_;
  double last_temp_error_;
  double last_pressure_error_;
  double last_ghsv_error_;

  double temp_kp_;
  double temp_ki_;
  double temp_kd_;
  double pressure_kp_;
  double pressure_ki_;
  double pressure_kd_;
  double ghsv_kp_;
  double ghsv_kd_;

  // System states
  bool reactor_state_;
  bool valve_state_;

  // ROS publishers, subscribers, and timers
  rclcpp::Publisher<space_station_eclss::msg::Sabatier>::SharedPtr sabatier_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr water_output_pub_;  // ✅ NEW
  rclcpp::Subscription<space_station_eclss::msg::Electrolysis>::SharedPtr hydrogen_subscriber_;
  rclcpp::Subscription<space_station_eclss::msg::AirData>::SharedPtr ars_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // space_station_eclss_SABATIER_PUBLISHER_HPP_
