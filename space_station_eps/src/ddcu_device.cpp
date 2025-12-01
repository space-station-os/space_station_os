#include "space_station_eps/ddcu_device.hpp"


using namespace space_station_eps;

DdcuNode::DdcuNode(const rclcpp::NodeOptions &options)
: Node("ddcu_node", options), ddcu_temperature_(25.0)  // Initial temp: 25°C
{
  this->declare_parameter<std::string>("ddcu_type", "DDCU-I");
  this->declare_parameter<double>("regulation_nominal", 124.5);
  this->declare_parameter<double>("regulation_tolerance", 1.5);

  this->get_parameter("ddcu_type", ddcu_type_);
  this->get_parameter("regulation_nominal", nominal_voltage_);
  this->get_parameter("regulation_tolerance", regulation_tolerance_);

  primary_voltage_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/ddcu/input_voltage", 10,
    std::bind(&DdcuNode::primaryVoltageCallback, this, std::placeholders::_1)
  );
  load_srv_ = this->create_service<space_station_interfaces::srv::Load>(
    "/ddcu/load_request",
    std::bind(&DdcuNode::handleLoadRequest, this,
              std::placeholders::_1, std::placeholders::_2)
  );
  output_voltage_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ddcu/output_voltage", 10);
  temperature_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ddcu/temperature", 10);
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/eps/diagnostics", 10);

  coolant_client_ = rclcpp_action::create_client<Coolant>(
    this, "/coolant_heat_transfer");

  RCLCPP_INFO(this->get_logger(), "DDCU (%s) Node initialized", ddcu_type_.c_str());
}

void DdcuNode::primaryVoltageCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  input_voltage_ = msg->data;
  double output_voltage = nominal_voltage_;

  // Fault conditions for input voltage
  if (input_voltage_ < 115.0 || input_voltage_ > 173.0)
  {
    output_voltage = 0.0;
    publishOutputVoltage(output_voltage);
    publishDiagnostics(input_voltage_, output_voltage);
    return;
  }

  // Add small fluctuations to simulate regulation behavior
  output_voltage += ((rand() % 300) - 150) / 100.0; // ±1.5V
  publishOutputVoltage(output_voltage);
  publishDiagnostics(input_voltage_, output_voltage);

  // Simulate output power and heat loss
  const double output_current = 40.0;  // Amps
  const double output_power_kw = (output_voltage * output_current) / 1000.0;

  const double efficiency = 0.96;
  const double input_power_kw = output_power_kw / efficiency;
  const double heat_loss_kw = input_power_kw - output_power_kw;

  const double heat_loss_joules = heat_loss_kw * 1000.0;

  // Increase temperature with runtime if voltage is valid
  if (input_voltage_ >= 115.0 && input_voltage_ <= 173.0) {
    ddcu_temperature_ += 0.2;
    if (ddcu_temperature_ > 85.0) ddcu_temperature_ = 85.0;
  }

  // Publish DDCU temperature
  std_msgs::msg::Float64 temp_msg;
  temp_msg.data = ddcu_temperature_;
  temperature_pub_->publish(temp_msg);

  callInternalCooling(heat_loss_joules);
}

void DdcuNode::publishOutputVoltage(double voltage)
{
  std_msgs::msg::Float64 msg;
  msg.data = voltage;
  output_voltage_pub_->publish(msg);
}

void DdcuNode::handleLoadRequest(
  const std::shared_ptr<space_station_interfaces::srv::Load::Request> request,
  std::shared_ptr<space_station_interfaces::srv::Load::Response> response)
{
  std::lock_guard<std::mutex> lock(voltage_mutex_);

  double load_voltage = request->load_voltage;
  RCLCPP_INFO(this->get_logger(), "[DDCU] Voltage request received: %.2f V", load_voltage);

  if (input_voltage_ < 115.0 || input_voltage_ > 173.0) {
    response->success = false;
    response->message = "Primary bus voltage unstable — unable to provide load voltage.";
    RCLCPP_WARN(this->get_logger(), "[DDCU] Request rejected: primary bus unstable.");
    return;
  }
  if (load_voltage < (nominal_voltage_ - regulation_tolerance_) ||
      load_voltage > (nominal_voltage_ + regulation_tolerance_)) {
    response->success = false;
    response->message = "Requested voltage outside regulation range.";
    RCLCPP_WARN(this->get_logger(), "[DDCU] Request rejected: voltage out of range.");
    return;
  }
  double output_voltage = nominal_voltage_;
  output_voltage -= (load_voltage * 0.005);
  if (output_voltage < nominal_voltage_ - regulation_tolerance_) {
    output_voltage = nominal_voltage_ - regulation_tolerance_;
  }

  // Publish updated voltage
  publishOutputVoltage(output_voltage);

  // Return success response
  response->success = true;
  response->message = "Voltage supply successful at " + std::to_string(output_voltage) + " V.";

  RCLCPP_INFO(this->get_logger(),
              "[DDCU] Voltage supplied: %.2f V (requested %.2f V)",
              output_voltage, load_voltage);
}
void DdcuNode::callInternalCooling(double heat_j)
{
  constexpr double HEAT_THRESHOLD_J = 10.0;

  if (heat_j < HEAT_THRESHOLD_J) {
    RCLCPP_DEBUG(this->get_logger(), "[COOLING] Heat too small: %.2f J, skipping.", heat_j);
    return;
  }

  if (!coolant_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "[COOLING] Coolant action server not available.");
        
    return;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                       "[DDCU] Temperature=%.2f°C, Heat=%.2f J",
                       ddcu_temperature_, heat_j);

  if (ddcu_temperature_ > 40.0){
    RCLCPP_DEBUG(this->get_logger(), "[COOLING] Temperature high: %.2f °C, activating cooling.", ddcu_temperature_);
 
  
    Coolant::Goal goal;
    goal.input_temperature_c = ddcu_temperature_;   // send current temp as cooling request
    goal.component_id = "ddcu_" + ddcu_type_;

    RCLCPP_INFO(this->get_logger(),
                "[COOLING] Sending goal to coolant server: temp=%.2f °C, heat=%.2f J",
                ddcu_temperature_, heat_j);

    auto send_goal_options = rclcpp_action::Client<Coolant>::SendGoalOptions();

    send_goal_options.feedback_callback =
      [this](GoalHandleCoolant::SharedPtr,
            const std::shared_ptr<const Coolant::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(),
                    "[COOLING][Feedback] Internal=%.2f°C | Ammonia=%.2f°C | Vented=%.2f kJ",
                    feedback->internal_temp_c, feedback->ammonia_temp_c, feedback->vented_heat_kj);

        // Update local DDCU temperature with feedback from coolant loop
        ddcu_temperature_ = feedback->internal_temp_c;

        // Publish updated temperature
        std_msgs::msg::Float64 temp_msg;
        temp_msg.data = ddcu_temperature_;
        temperature_pub_->publish(temp_msg);
      };

    send_goal_options.result_callback =
      [this](const GoalHandleCoolant::WrappedResult &result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(),
                      "[COOLING] Goal succeeded: %s", result.result->message.c_str());
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "[COOLING] Goal failed or canceled.");
        }
      };

    coolant_client_->async_send_goal(goal, send_goal_options);
  }
}

void DdcuNode::publishDiagnostics(double input_voltage, double output_voltage)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "DDCU_" + ddcu_type_;
  status.hardware_id = "DDCU_Generic";

  if (input_voltage < 115.0 || input_voltage > 173.0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "Primary input voltage out of range!";
  }
  else if (output_voltage < (nominal_voltage_ - regulation_tolerance_) ||
           output_voltage > (nominal_voltage_ + regulation_tolerance_)) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "Secondary voltage outside regulation range!";
  }
  else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "DDCU operating within limits.";
  }

  diag_pub_->publish(status);
}

int main (int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DdcuNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
