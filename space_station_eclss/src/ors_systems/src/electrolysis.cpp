#include "space_station_eclss/electrolysis.h"
#include <cmath>

ElectrolysisNode::ElectrolysisNode() : Node("electrolysis_node"), water_available_(false) {
    electrolysis_server_ = this->create_service<space_station_eclss::srv::Water>(
        "/electrolysis",
        std::bind(&ElectrolysisNode::handle_electrolysis_request, this, std::placeholders::_1, std::placeholders::_2));

    gas_pub_ = this->create_publisher<space_station_eclss::msg::Electrolysis>("/electrolysis_output", 10);

    this->declare_parameter("efficiency_factor", 0.95);
    this->declare_parameter("required_pressure", 50.0);
    this->declare_parameter("depletion_factor", 0.3);
    water_status = false;
    RCLCPP_INFO(this->get_logger(), "[INIT] Electrolysis Node Initialized. Water Available: %s", water_available_ ? "TRUE" : "FALSE");
}

void ElectrolysisNode::handle_electrolysis_request(
    const std::shared_ptr<space_station_eclss::srv::Water::Request> request,
    std::shared_ptr<space_station_eclss::srv::Water::Response> response) {

    RCLCPP_INFO(this->get_logger(), "[Request] Before Handling: Water Available = %s", water_available_ ? "TRUE" : "FALSE");

    if (water_available_ && water_level_ > 0) {
        response->success = false;
        response->message = "Electrolysis is already processing a batch.";
        RCLCPP_WARN(this->get_logger(), "[Request] Rejecting: Electrolysis is already processing a batch.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "[Request] Accepting New Water for Electrolysis.");

    water_level_ = request->water;
    pressure_ = request->pressure;
    temperature_ = request->temperature;
    water_available_ = true;

    RCLCPP_INFO(this->get_logger(), "[Request] Received: Water: %.2f L, Pressure: %.2f, Temp: %.2f", water_level_, pressure_, temperature_);
    response->success = true;
    response->message = "Electrolysis started successfully.";

   
    if (!electrolysis_timer_ || electrolysis_timer_->is_canceled()) {
        electrolysis_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ElectrolysisNode::performElectrolysis, this));
    }

    RCLCPP_INFO(this->get_logger(), "[Request] After Handling: Water Available = %s", water_available_ ? "TRUE" : "FALSE");
}

void ElectrolysisNode::performElectrolysis() {
    RCLCPP_INFO(this->get_logger(), "[Electrolysis] Before Processing: Water Available = %s", water_available_ ? "TRUE" : "FALSE");

    if (!water_available_ || water_level_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "[Electrolysis] No Water Available! Resetting Electrolysis State...");
        resetElectrolysisState(water_status=true);
        return;
    }

    efficiency_factor_ = this->get_parameter("efficiency_factor").as_double();
    required_pressure_ = this->get_parameter("required_pressure").as_double();
    depletion_factor_ = this->get_parameter("depletion_factor").as_double();

    if (pressure_ < required_pressure_) {
        RCLCPP_WARN(this->get_logger(), "[Electrolysis] Insufficient Pressure (%.2f). Adjusting Pressure...", pressure_);
        pressure_ = required_pressure_;
    }

    double energy_required_per_mol = 237.1;
    double water_moles = water_level_ * 55.5;
    double total_energy = water_moles * energy_required_per_mol;
    double actual_energy_used = total_energy * efficiency_factor_;

    double hydrogen_moles = (2.0 * water_moles * efficiency_factor_);
    double oxygen_moles = (1.0 * water_moles * efficiency_factor_);

    water_level_ *= (1.0 - depletion_factor_);

    if (water_level_ <= 2.0) {
        
        RCLCPP_WARN(this->get_logger(), "[Electrolysis] Water Fully Depleted! Resetting...");
        resetElectrolysisState(water_status=true);
    }

    RCLCPP_INFO(this->get_logger(), "[Electrolysis] Remaining Water After Processing: %.2f L", water_level_);
    publishElectrolysisData(hydrogen_moles, oxygen_moles, actual_energy_used);

    RCLCPP_INFO(this->get_logger(), "[Electrolysis] After Processing: Water Available = %s", water_available_ ? "TRUE" : "FALSE");
}


void ElectrolysisNode::publishElectrolysisData(double hydrogen_moles, double oxygen_moles, double energy_used) {
    auto gas_msg = space_station_eclss::msg::Electrolysis();
    gas_msg.header.stamp = this->get_clock()->now();
    gas_msg.h2 = hydrogen_moles;
    gas_msg.o2 = oxygen_moles;
    gas_msg.temperature.temperature = temperature_ + (energy_used / 1000);
    gas_msg.pressure.fluid_pressure = pressure_ + (hydrogen_moles * 0.1);

    gas_pub_->publish(gas_msg);

    RCLCPP_INFO(this->get_logger(), "===========================");
    RCLCPP_INFO(this->get_logger(), "[Publish] Electrolysis: Produced H₂ = %.2f moles, O₂ = %.2f moles", hydrogen_moles, oxygen_moles);
    RCLCPP_INFO(this->get_logger(), "[Publish] Temperature: %.2f C, Pressure: %.2f Pa", temperature_, pressure_);
    RCLCPP_INFO(this->get_logger(), "===========================");
}

void ElectrolysisNode::resetElectrolysisState(bool water_status) {
    if ( water_status ) {
        RCLCPP_WARN(this->get_logger(), "[Reset] Resetting Electrolysis State...");
        water_available_ = false;
        water_level_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "[Reset] Electrolysis is now available for new water.");
    }
    RCLCPP_INFO(this->get_logger(), "[Reset] After Reset: Water Available = %s", water_available_ ? "TRUE" : "FALSE");
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ElectrolysisNode>());
    rclcpp::shutdown();
    return 0;
}
