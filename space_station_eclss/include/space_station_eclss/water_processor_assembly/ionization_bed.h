#ifndef CATALYTIC_REACTOR_PROCESSOR_HPP
#define CATALYTIC_REACTOR_PROCESSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include "space_station_eclss/srv/ion_bed.hpp"
#include "space_station_eclss/srv/water.hpp"
#include "space_station_eclss/msg/water_crew.hpp"

class CatalyticReactorProcessor : public rclcpp::Node {
public:
    CatalyticReactorProcessor();

private:
    rclcpp::Service<space_station_eclss::srv::IonBed>::SharedPtr catalytic_reactor_service_;
    rclcpp::Client<space_station_eclss::srv::Water>::SharedPtr product_water_tank_client_;
    rclcpp::Publisher<space_station_eclss::msg::WaterCrew>::SharedPtr waste_status_publisher_;
    rclcpp::TimerBase::SharedPtr dashboard_timer_;
    double unprocessed_water_;  // Store water if Product Water Tank is unavailable

    void handle_catalytic_reactor(
        const std::shared_ptr<space_station_eclss::srv::IonBed::Request> request,
        std::shared_ptr<space_station_eclss::srv::IonBed::Response> response);
   void publish_total_water(double filtered_water, double contaminants, double iodine_level, double gas_bubbles);

    
    void process_ion_exchange(double &filtered_water, double &contaminants, double &iodine_level, double &gas_bubbles);
    void send_to_product_water_tank(double water, double gas_bubbles, double contaminants, double iodine_level, double pressure, double temperature);
    void handle_product_water_tank_response(rclcpp::Client<space_station_eclss::srv::Water>::SharedFuture future);
};

#endif // CATALYTIC_REACTOR_PROCESSOR_HPP
