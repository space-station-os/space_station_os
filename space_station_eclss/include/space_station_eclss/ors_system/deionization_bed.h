#ifndef IONIZATION_BED_HPP
#define IONIZATION_BED_HPP

#include "rclcpp/rclcpp.hpp"
#include "space_station_eclss/srv/water.hpp"

class IonizationBed : public rclcpp::Node {
public:
    IonizationBed();

private:
    void handle_deionization_request(
        const std::shared_ptr<space_station_eclss::srv::Water::Request> request,
        std::shared_ptr<space_station_eclss::srv::Water::Response> response);
    
    void contamination_removal_pipeline();
    void deionization();
    void contamination_removal();
    void gas_sensor();
    void open_three_way_valve();
    void send_to_electrolysis();

    // ROS Components
    rclcpp::Service<space_station_eclss::srv::Water>::SharedPtr deionization_server_;
    rclcpp::Client<space_station_eclss::srv::Water>::SharedPtr electrolysis_client_;
    rclcpp::TimerBase::SharedPtr processing_timer_;

    // Water Properties
    double water_;
    double contaminants_;
    double gas_bubbles_;
    double iodine_level_;
    bool is_active;
};

#endif // IONIZATION_BED_HPP
