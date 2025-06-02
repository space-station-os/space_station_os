#ifndef space_station_eclss_ELECTROLYSIS_H
#define space_station_eclss_ELECTROLYSIS_H

#include <rclcpp/rclcpp.hpp>
#include "space_station_eclss/srv/water.hpp"
#include "space_station_eclss/msg/electrolysis.hpp"
#include <std_msgs/msg/float64.hpp>
class ElectrolysisNode : public rclcpp::Node {
public:
    ElectrolysisNode();

private:
    void handle_electrolysis_request(
        const std::shared_ptr<space_station_eclss::srv::Water::Request> request,
        std::shared_ptr<space_station_eclss::srv::Water::Response> response);

    void performElectrolysis();

    void publishElectrolysisData(double hydrogen_moles, double oxygen_moles, double energy_used);

    void resetElectrolysisState(bool water_status);

    bool water_available_;
    double water_level_;
    double pressure_;
    double temperature_;
    double efficiency_factor_;
    double required_pressure_;
    double depletion_factor_;
    bool water_status;
    rclcpp::Service<space_station_eclss::srv::Water>::SharedPtr electrolysis_server_;
    rclcpp::Publisher<space_station_eclss::msg::Electrolysis>::SharedPtr gas_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr h2_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr o2_pub_;
    rclcpp::TimerBase::SharedPtr electrolysis_timer_;
};

#endif // space_station_eclss_ELECTROLYSIS_H
