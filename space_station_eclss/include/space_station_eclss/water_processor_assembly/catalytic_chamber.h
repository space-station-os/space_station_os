#ifndef MULTI_FILTRATION_PROCESSOR_HPP
#define MULTI_FILTRATION_PROCESSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include "space_station_eclss/srv/filteration.hpp"
#include "space_station_eclss/srv/ion_bed.hpp"
#include "space_station_eclss/msg/water_crew.hpp"

class MultiFiltrationProcessor : public rclcpp::Node {
public:
    MultiFiltrationProcessor();

private:
    rclcpp::Service<space_station_eclss::srv::Filteration>::SharedPtr multi_filtration_service_;
    rclcpp::Client<space_station_eclss::srv::IonBed>::SharedPtr catalytic_reactor_client_;
    rclcpp::Publisher<space_station_eclss::msg::WaterCrew>::SharedPtr waste_status_publisher_;
   
    double unprocessed_water_;  // Stores water if Catalytic Reactor is unavailable

    void handle_multi_filtration(
        const std::shared_ptr<space_station_eclss::srv::Filteration::Request> request,
        std::shared_ptr<space_station_eclss::srv::Filteration::Response> response);
    void publish_total_water(double water, double gas_bubbles, double contaminants, double iodine_level, double pressure, double temperature);

    void remove_ammonia_and_organics(double &filtered_water, double &contaminants, double &organics, double &ammonia);
    void send_to_catalytic_reactor(double filtered_water, double remaining_contaminants, double organics);
    void handle_catalytic_reactor_response(rclcpp::Client<space_station_eclss::srv::IonBed>::SharedFuture future);
};

#endif // MULTI_FILTRATION_PROCESSOR_HPP
