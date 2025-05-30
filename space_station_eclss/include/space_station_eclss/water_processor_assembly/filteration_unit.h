#ifndef WATER_PROCESSOR_ASSEMBLY_HPP
#define WATER_PROCESSOR_ASSEMBLY_HPP

#include <rclcpp/rclcpp.hpp>
#include "space_station_eclss/srv/distillation.hpp"
#include "space_station_eclss/srv/filteration.hpp"
#include "space_station_eclss/msg/water_crew.hpp"
class WaterProcessorAssembly : public rclcpp::Node {
public:
    WaterProcessorAssembly();

private:
    rclcpp::Service<space_station_eclss::srv::Distillation>::SharedPtr wpa_service_;
    rclcpp::Client<space_station_eclss::srv::Filteration>::SharedPtr next_stage_client_;
    rclcpp::Publisher<space_station_eclss::msg::WaterCrew>::SharedPtr waste_status_publisher_;
    rclcpp::TimerBase::SharedPtr dashboard_timer_;
    void handle_water_processing(
        const std::shared_ptr<space_station_eclss::srv::Distillation::Request> request,
        std::shared_ptr<space_station_eclss::srv::Distillation::Response> response);
    void publish_status(double water, double contaminants, double organics, double ammonia);
    void send_to_next_stage(double filtered_water, double remaining_contaminants, double organics, double ammonia);
    void handle_next_stage_response(rclcpp::Client<space_station_eclss::srv::Filteration>::SharedFuture future);
};

#endif // WATER_PROCESSOR_ASSEMBLY_HPP
