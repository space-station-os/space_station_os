#include "space_station_eclss/ars_system/air_collector_tank.hpp"
#include <cmath>

using namespace std::chrono_literals;

AirCollector::AirCollector()
    : Node("air_collector"), previous_mode_("idle"), service_unavailable_count_(0)  // Initialize previous_mode_
{
  RCLCPP_INFO(this->get_logger(), "Initializing Air Collector System...");

  // Load parameters dynamically
  this->declare_parameter("crew_onboard", 4);
  this->declare_parameter("cabin_pressure", 14.7);
  this->declare_parameter("temperature_cutoff", 450.0);
  this->declare_parameter("max_crew_limit", 6);
  this->declare_parameter("power_consumption", 1.0);
  this->declare_parameter("tank_capacity", 1000.0);
  this->declare_parameter("system_name", "space_station_eclss");
  this->declare_parameter("mode_of_operation", "idle");  

  this->declare_parameter("co2_threshold", 100.0);
  this->declare_parameter("moisture_threshold", 70.0);
  this->declare_parameter("contaminants_threshold", 30.0);

  this->declare_parameter("temp_kp", 0.1);
  this->declare_parameter("temp_ki", 0.01);
  this->declare_parameter("temp_kd", 0.005);
  this->declare_parameter("press_kp", 0.1);
  this->declare_parameter("press_ki", 0.01);
  this->declare_parameter("press_kd", 0.005);

  this->declare_parameter("C_activity", 1.04);  

  temperature_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("/temperature", 10);
  pressure_publisher_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("/pipe_pressure", 10);
  desiccant_bed_client_ = this->create_client<space_station_eclss::srv::CrewQuarters>("/desiccant_bed1");
  air_quality_publisher_ = this->create_publisher<space_station_eclss::msg::AirData>("/collector_air_quality", 10);

  timer_ = this->create_wall_timer(1s, std::bind(&AirCollector::timer_callback, this));


  cdra_status_publisher_ = this->create_publisher<space_station_eclss::msg::CdraStatus>("/cdra_status", 10);
  cdra=space_station_eclss::msg::CdraStatus();  
  RCLCPP_INFO(this->get_logger(), "\033[1;34mAir Collector System successfully initialized.\033[0m");
}

void AirCollector::timer_callback()
{
  this->get_parameter("crew_onboard", crew_onboard_);
  this->get_parameter("cabin_pressure", cabin_pressure_);
  this->get_parameter("temperature_cutoff", temperature_cutoff_);
  this->get_parameter("power_consumption", power_consumption_);
  this->get_parameter("tank_capacity", tank_capacity_);

  this->get_parameter("co2_threshold", co2_threshold_);
  this->get_parameter("moisture_threshold", moisture_threshold_);
  this->get_parameter("contaminants_threshold", contaminants_threshold_);
  this->get_parameter("mode_of_operation", mode_of_operation_);

  double delta_t = 1.0;  

  double C_activity;
  this->get_parameter("C_activity", C_activity);
  simulate_temperature_pressure();
  if (mode_of_operation_ == "idle") {
      C_activity = 0.8;
  } else if (mode_of_operation_ == "exercise") {
      C_activity = 2.5;
  } else if (mode_of_operation_ == "emergency") {
      C_activity = 3.0;
  } else if (mode_of_operation_ == "biological_research") {
      C_activity = 1.2;
  } else if (mode_of_operation_ == "eva_repair") {
      C_activity = 2.8;
  }
  cdra.co2_processing_state = 0;
  cdra.system_health = 1;
  cdra.data="IDLE";
  cdra_status_publisher_->publish(cdra);
  if (mode_of_operation_ != previous_mode_) {
      RCLCPP_INFO(this->get_logger(), "[Mode Change] %s -> %s | Adjusted CO₂ Activity Rate: %.2f g/min per astronaut", 
                  previous_mode_.c_str(), mode_of_operation_.c_str(), C_activity);
      previous_mode_ = mode_of_operation_;
      update_other_nodes_parameters(mode_of_operation_); 
  }

  double delta_co2 = (crew_onboard_ * C_activity * (cabin_pressure_ / 14.7) * (temperature_cutoff_ / 450.0)) * delta_t;
  co2_mass_ += delta_co2;

  double moisture_factor = (mode_of_operation_ == "exercise") ? 1.5 :
                           (mode_of_operation_ == "emergency") ? 2.0 :
                           (mode_of_operation_ == "biological_research") ? 0.7 :
                           (mode_of_operation_ == "eva_repair") ? 0.2 : 0.5;

  double delta_moisture = (crew_onboard_ * moisture_factor + (temperature_cutoff_ / 450.0) * 2.5) * delta_t;
  moisture_content_ += delta_moisture;

  double contaminants_factor = (mode_of_operation_ == "exercise") ? 0.5 :
                               (mode_of_operation_ == "emergency") ? 0.7 :
                               (mode_of_operation_ == "biological_research") ? 0.4 :
                               (mode_of_operation_ == "eva_repair") ? 0.8 : 0.3;

  double delta_contaminants = (crew_onboard_ * contaminants_factor - 0.1 * power_consumption_) * delta_t;
  contaminants_ += delta_contaminants;

  RCLCPP_INFO(this->get_logger(), "\033[1;34mCO₂ Mass: %.2f g, Moisture: %.2f %%, Contaminants: %.2f %%\033[0m",
              co2_mass_, moisture_content_, contaminants_);
  RCLCPP_INFO(this->get_logger(),"==============================================");
  cdra.co2_processing_state = 1;
  cdra.system_health = 1;
  cdra.data="AIR_COLLECTION";
  cdra_status_publisher_->publish(cdra);

  auto msg = space_station_eclss::msg::AirData();
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id="collector";
  msg.co2_mass = co2_mass_;
  msg.moisture_content = moisture_content_;
  msg.contaminants = contaminants_;

  air_quality_publisher_->publish(msg);

  double total=co2_mass_+moisture_content_+contaminants_;
  if (total<tank_capacity_) 
  {
    RCLCPP_INFO(this->get_logger(), "\033[1;34mTank Capacity: %.2f g\033[0m", total);
   
  

      if (co2_mass_ > co2_threshold_ ) 
      {
        if (!desiccant_bed_client_->wait_for_service(3s))  
        {
          service_unavailable_count_++;
          RCLCPP_WARN(this->get_logger(), "Desiccant Bed Service unavailable. Holding air... (%d attempts)", service_unavailable_count_);
          
          if (service_unavailable_count_ >= 5)  
          {
            cdra.co2_processing_state = 1;
            cdra.system_health = 2;
            cdra.data="FAILURE";
            cdra_status_publisher_->publish(cdra);
            RCLCPP_ERROR(this->get_logger(), "EMERGENCY: Desiccant Bed Service unavailable for too long! Immediate action required!");
          }
        } 
        else 
        {
          service_unavailable_count_ = 0;  
          send_air_to_desiccant_bed();
        }
      }
    }
    else 
    {
      RCLCPP_ERROR(this->get_logger(), "\033[1;31mTank Capacity Exceeded! Current Total: %.2f g\033[0m", total);
      cdra.co2_processing_state = 1;
      cdra.system_health = 2;
      cdra.data="FAILURE";
      cdra_status_publisher_->publish(cdra);
    }
    
}
void AirCollector::send_air_to_desiccant_bed() {
  if (!desiccant_bed_client_->wait_for_service(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Desiccant Bed service is not available!");
      return;
  }

  cdra.co2_processing_state = 2;
  cdra.system_health = 1;
  cdra.data = "SENDING_TO_DESICCANT";
  cdra_status_publisher_->publish(cdra);
  auto request = std::make_shared<space_station_eclss::srv::CrewQuarters::Request>();
  request->co2_mass = co2_mass_;
  request->moisture_content = moisture_content_;
  request->contaminants = contaminants_;

  RCLCPP_INFO(this->get_logger(), "\033[1;34mRequesting Desiccant Bed to process air batch...\033[0m");
  RCLCPP_INFO(this->get_logger(),"==============================================");
 
  auto future = desiccant_bed_client_->async_send_request(request,
    std::bind(&AirCollector::process_desiccant_response, this, std::placeholders::_1));
}

void AirCollector::process_desiccant_response(rclcpp::Client<space_station_eclss::srv::CrewQuarters>::SharedFuture future) {
  try {
      auto response = future.get();
      if (response->success) {
          RCLCPP_INFO(this->get_logger(), "\033[1;34mDesiccant Bed accepted air for processing. Opening valve...");
          
          co2_mass_ = 0.0;
          moisture_content_ = 0.0;
          contaminants_ = 0.0;

          cdra.co2_processing_state = 2;
          cdra.system_health = 1;
          cdra.data = "DESICCANT_PROCESSING";
          cdra_status_publisher_->publish(cdra);


          RCLCPP_INFO(this->get_logger(), "\033[1;34mAir Collector reset. Continuing collection...\033[0m");
      } else {
          RCLCPP_WARN(this->get_logger(), "Desiccant Bed is still processing. Holding air...");
      }
  } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception while calling Desiccant Bed: %s", e.what());
  }
}


void AirCollector::simulate_temperature_pressure() {
  // Retrieve PID parameters dynamically
  double temp_kp, temp_ki, temp_kd, press_kp, press_ki, press_kd;
  this->get_parameter("temp_kp", temp_kp);
  this->get_parameter("temp_ki", temp_ki);
  this->get_parameter("temp_kd", temp_kd);
  this->get_parameter("press_kp", press_kp);
  this->get_parameter("press_ki", press_ki);
  this->get_parameter("press_kd", press_kd);
  
  double desired_temperature = 70.0;  // **Target temp set to 70°C**
  double desired_pressure = cabin_pressure_;  // **Maintain cabin pressure**

  // **PD control logic for temperature adjustment**
  double temp_error = desired_temperature - current_temperature_;
  double temp_derivative = temp_error - prev_temp_error_;
  double temp_adjustment = (temp_kp * temp_error) + (temp_kd * temp_derivative);
  prev_temp_error_ = temp_error;

  // **Limit the temperature increase to prevent excessive oscillation**
  if (std::abs(temp_error) > 1.0) {  // Only adjust if deviation is significant
    current_temperature_ += temp_adjustment;
  }

  // **Prevent overshoot**
  if (current_temperature_ > desired_temperature + 1.0) {
    current_temperature_ = desired_temperature + 1.0;
  } else if (current_temperature_ < desired_temperature - 1.0) {
    current_temperature_ = desired_temperature - 1.0;
  }

  // **PD control logic for pressure adjustment**
  double press_error = desired_pressure - current_pressure_;
  double press_derivative = press_error - prev_press_error_;
  double press_adjustment = (press_kp * press_error) + (press_kd * press_derivative);
  prev_press_error_ = press_error;

  current_pressure_ += press_adjustment;

  // **Publish Temperature**
  sensor_msgs::msg::Temperature temp_msg;
  temp_msg.temperature = current_temperature_;
  temp_msg.variance = 0.5;  // Simulated variance
  temperature_publisher_->publish(temp_msg);

  // **Publish Pressure**
  sensor_msgs::msg::FluidPressure press_msg;
  press_msg.fluid_pressure = current_pressure_;
  press_msg.variance = 0.1;
  pressure_publisher_->publish(press_msg);

  // **Log Temperature and Pressure**
  RCLCPP_INFO(this->get_logger(), "Simulated Temperature: %.2f°C (Target: 70°C), Pressure: %.2f psi", 
              current_temperature_, current_pressure_);
}


void AirCollector::update_other_nodes_parameters(const std::string &mode) {
  using rclcpp::AsyncParametersClient;

  std::map<std::string, double> desiccant_rates {
    {"idle", 0.9}, {"standby", 1.0}, {"exercise", 1.2},
    {"emergency", 1.3}, {"biological_research", 1.1}, {"eva_repair", 1.25}
  };
  std::map<std::string, double> adsorbent_eff {
    {"idle", 0.90}, {"standby", 0.95}, {"exercise", 1.05},
    {"emergency", 1.15}, {"biological_research", 1.00}, {"eva_repair", 1.1}
  };

  auto desiccant_client = std::make_shared<AsyncParametersClient>(this, "desiccant_server");
  auto adsorbent_client = std::make_shared<AsyncParametersClient>(this, "adsorbent_bed");

  if (desiccant_client->wait_for_service(1s)) {
    desiccant_client->set_parameters({
      rclcpp::Parameter("moisture_removal_rate", desiccant_rates[mode]),
      rclcpp::Parameter("contaminant_removal_rate", desiccant_rates[mode])
    });
  }

  if (adsorbent_client->wait_for_service(1s)) {
    adsorbent_client->set_parameters({
      rclcpp::Parameter("co2_removal_efficiency", adsorbent_eff[mode])
    });
  }
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AirCollector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
