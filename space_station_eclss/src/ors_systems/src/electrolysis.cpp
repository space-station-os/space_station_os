#include "space_station_eclss/ors_system/electrolysis.h"
#include <cmath>

ElectrolysisNode::ElectrolysisNode() : Node("electrolysis_node"), water_available_(false) {
    electrolysis_server_ = this->create_service<space_station_eclss::srv::Water>(
        "/electrolysis",
        std::bind(&ElectrolysisNode::handle_electrolysis_request, this, std::placeholders::_1, std::placeholders::_2));

    gas_pub_ = this->create_publisher<space_station_eclss::msg::Electrolysis>("/electrolysis_output", 10);
    h2_pub_ = this->create_publisher<std_msgs::msg::Float64>("/h2_storage", 10);
    o2_pub_ = this->create_publisher<std_msgs::msg::Float64>("/o2_recovery", 10);

    this->declare_parameter("efficiency_factor", 0.95);
    this->declare_parameter("required_pressure", 50.0);
    this->declare_parameter("depletion_factor", 0.3);

    water_status = false;
    RCLCPP_INFO(this->get_logger(), "[INIT] Electrolysis Node Initialized.");
}

void ElectrolysisNode::handle_electrolysis_request(
    const std::shared_ptr<space_station_eclss::srv::Water::Request> request,
    std::shared_ptr<space_station_eclss::srv::Water::Response> response) {

    if (water_available_ && water_level_ > 0) {
        response->success = false;
        response->message = "Electrolysis already in progress.";
        RCLCPP_WARN(this->get_logger(), "[REJECT] Electrolysis in progress.");
        return;
    }

    water_level_ = request->water;
    pressure_ = request->pressure;
    temperature_ = request->temperature;
    water_available_ = true;

    response->success = true;
    response->message = "Electrolysis started.";

    RCLCPP_INFO(this->get_logger(), "[START] Electrolysis started with %.2f L at %.2f°C, %.2f Pa.",
                water_level_, temperature_, pressure_);

    if (!electrolysis_timer_ || electrolysis_timer_->is_canceled()) {
        electrolysis_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ElectrolysisNode::performElectrolysis, this));
    }
}

void ElectrolysisNode::performElectrolysis() {
    if (!water_available_ || water_level_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "[SKIP] No water available. Stopping...");
        resetElectrolysisState(true);
        return;
    }

    efficiency_factor_ = this->get_parameter("efficiency_factor").as_double();
    required_pressure_ = this->get_parameter("required_pressure").as_double();
    depletion_factor_ = this->get_parameter("depletion_factor").as_double();

    if (pressure_ < required_pressure_) {
        RCLCPP_WARN(this->get_logger(), "[ADJUST] Pressure %.2f below threshold %.2f. Adjusting.", pressure_, required_pressure_);
        pressure_ = required_pressure_;
    }

    double energy_required_per_mol = 237.1;
    double water_moles = water_level_ * 55.5;
    double total_energy = water_moles * energy_required_per_mol;
    double actual_energy_used = total_energy * efficiency_factor_;

    double hydrogen_moles = 2.0 * water_moles * efficiency_factor_;
    double oxygen_moles = 1.0 * water_moles * efficiency_factor_;
    water_level_ *= (1.0 - depletion_factor_);

    publishElectrolysisData(hydrogen_moles, oxygen_moles, actual_energy_used);

    if (water_level_ <= 2.0) {
        RCLCPP_INFO(this->get_logger(), "[COMPLETE] Water depleted. Stopping electrolysis.");
        resetElectrolysisState(true);
    }
}

void ElectrolysisNode::publishElectrolysisData(double hydrogen_moles, double oxygen_moles, double energy_used) {
    auto gas_msg = space_station_eclss::msg::Electrolysis();
    gas_msg.header.stamp = this->get_clock()->now();

    const double molar_mass_h2 = 2.016;
    const double molar_mass_o2 = 31.998;

    double hydrogen_grams = hydrogen_moles * molar_mass_h2;
    double oxygen_grams = oxygen_moles * molar_mass_o2;

    gas_msg.h2 = hydrogen_grams;
    gas_msg.o2 = oxygen_grams;

    gas_msg.temperature.temperature = temperature_ + (energy_used / 1000);
    gas_msg.pressure.fluid_pressure = pressure_ + (hydrogen_moles * 0.1);

    gas_pub_->publish(gas_msg);

    std_msgs::msg::Float64 h2_msg;
    h2_msg.data = hydrogen_grams;
    h2_pub_->publish(h2_msg);

    std_msgs::msg::Float64 o2_msg;
    o2_msg.data = oxygen_grams;
    o2_pub_->publish(o2_msg);

    RCLCPP_INFO(this->get_logger(), "[OUTPUT] H₂: %.2f g, O₂: %.2f g | Temp: %.2f°C, Pressure: %.2f Pa",
                hydrogen_grams, oxygen_grams, gas_msg.temperature.temperature, gas_msg.pressure.fluid_pressure);
}

void ElectrolysisNode::resetElectrolysisState(bool water_status) {
    if (water_status) {
        water_available_ = false;
        water_level_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "[RESET] Electrolysis reset. Ready for next batch.");
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ElectrolysisNode>());
    rclcpp::shutdown();
    return 0;
}
