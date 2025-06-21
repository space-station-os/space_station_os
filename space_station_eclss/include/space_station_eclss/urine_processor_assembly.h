#ifndef URINE_PROCESSOR_ASSEMBLY_HPP
#define URINE_PROCESSOR_ASSEMBLY_HPP

#include <rclcpp/rclcpp.hpp>
#include "space_station_eclss/srv/upa.hpp"
#include "space_station_eclss/srv/distillation.hpp"
#include "space_station_eclss/msg/water_crew.hpp"

class UrineProcessorAssembly : public rclcpp::Node {
public:
    UrineProcessorAssembly();

private:
    rclcpp::Service<space_station_eclss::srv::Upa>::SharedPtr upa_service_;
    rclcpp::Client<space_station_eclss::srv::Distillation>::SharedPtr wpa_client_;
    rclcpp::Publisher<space_station_eclss::msg::WaterCrew>::SharedPtr waste_status_pub_;
     rclcpp::TimerBase::SharedPtr dashboard_timer_;
    bool is_processing_;       // Tracks if UPA is currently processing a batch
    double unprocessed_water_; // Stores processed water if WPA is unavailable

    void handle_urine_request(
        const std::shared_ptr<space_station_eclss::srv::Upa::Request> request,
        std::shared_ptr<space_station_eclss::srv::Upa::Response> response);
    void publish_status(double water, double contaminants, double temperature);
    void simulate_distillation(double urine_volume, double &distilled_urine, double &contaminants);
    void simulate_purge_pump(double &distilled_urine);
    void send_to_wpa(double processed_water, double contaminants);
    void handle_wpa_response(rclcpp::Client<space_station_eclss::srv::Distillation>::SharedFuture future);
};

#endif // URINE_PROCESSOR_ASSEMBLY_HPP
