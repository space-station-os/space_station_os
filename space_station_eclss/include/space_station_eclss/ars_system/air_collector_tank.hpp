#ifndef space_station_eclss_AIR_COLLECTOR_TANK_HPP_
#define space_station_eclss_AIR_COLLECTOR_TANK_HPP_

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "space_station_eclss/srv/crew_quarters.hpp"
#include "space_station_eclss/msg/cdra_status.hpp"
#include "space_station_eclss/msg/air_data.hpp"

/**
 * @brief A simple PD Controller for temperature and pressure regulation.
 */
struct PIDController
{
  double kp, ki, kd;
  double prev_error = 0.0;
  double integral = 0.0;

  /**
   * @brief Computes the PD output given the setpoint and measured value.
   * 
   * @param setpoint The desired target value.
   * @param measured The current measured value.
   * @param dt The time interval between control updates.
   * @return The computed control output.
   */
  double compute(double setpoint, double measured, double dt)
  {
    double error = setpoint - measured;
    integral += error * dt;
    double derivative = (error - prev_error) / dt;
    double output = (kp * error) + (kd * derivative);
    prev_error = error;
    return output;
  }
};

/**
 * @class AirCollector
 * @brief Simulates air mixture properties and interacts with a CO₂ service.
 * 
 * - **Simulates CO₂, moisture, and contaminant accumulation** based on crew activity.
 * - **Publishes temperature & pressure using `sensor_msgs`**.
 * - **Sends air batches to the Desiccant Bed via a ROS 2 service-client interaction**.
 * - **Prevents overloading the Desiccant Bed by checking its availability before sending air**.
 * - **Handles emergency situations if the Desiccant Bed service remains unavailable**.
 * - **Supports multiple system modes**: `idle`, `exercise`, `emergency`, `biological_research`, `eva_repair`.
 */
class AirCollector : public rclcpp::Node
{
public:
  /**
   * @brief Constructor initializes parameters, timers, and service clients.
   */
  AirCollector();

private:
  /**
   * @brief Periodic update function handling simulation and service calls.
   */
  void timer_callback();

  /**
   * @brief Simulates temperature and pressure changes based on system activity.
   */
  void simulate_temperature_pressure();

  /**
   * @brief Sends CO₂, moisture, and contaminants data to the Desiccant Bed.
   * - Ensures the Desiccant Bed is available before sending.
   * - Holds air if the service is unavailable.
   * - Triggers emergency if service remains offline for too long.
   */
  void send_air_to_desiccant_bed();

  /**
   * @brief Handles the response from the Desiccant Bed service.
   * @param future Future object containing the service response.
   */
  void process_desiccant_response(rclcpp::Client<space_station_eclss::srv::CrewQuarters>::SharedFuture future);

  /**
   * @brief Updates `C_activity_` dynamically based on the mode of operation.
   */
  void update_c_activity();

  void update_other_nodes_parameters(const std::string &mode);

  /*** SYSTEM PARAMETERS ***/
  double flow_rate_;          ///< Flow rate in SCFM
  double co2_intake_;         ///< CO₂ intake in mmHg
  int crew_onboard_;          ///< Number of astronauts onboard
  double cabin_pressure_;     ///< Cabin pressure in PSI
  double temperature_cutoff_; ///< Maximum temperature cutoff in Celsius
  int max_crew_limit_;        ///< Maximum crew capacity
  double power_consumption_;  ///< Power consumption in kW
  double tank_capacity_;      ///< Maximum tank capacity for air mass in grams
  std::string system_name_;   ///< System name
  std::string mode_of_operation_; ///< Current mode of operation

  /*** SERVICE CALL THRESHOLDS ***/
  double co2_threshold_;         ///< CO₂ limit before triggering service call
  double moisture_threshold_;    ///< Moisture limit before triggering service call
  double contaminants_threshold_;///< Contaminant limit before triggering service call

  /*** SIMULATED AIR MIXTURE VALUES ***/
  double co2_mass_;         ///< Current CO₂ mass in grams
  double moisture_content_; ///< Moisture content in percentage
  double contaminants_;     ///< Contaminants in percentage
  double total_air_mass_;   ///< Total air mass in grams

  /*** TEMPERATURE & PRESSURE CONTROL USING PD ***/
  PIDController temp_pid_;  ///< PD Controller for temperature regulation
  PIDController press_pid_; ///< PD Controller for pressure regulation
  double current_temperature_; ///< Current simulated temperature
  double current_pressure_; ///< Current simulated pressure
  double prev_temp_error_;  ///< Previous error for temperature regulation
  double prev_press_error_; ///< Previous error for pressure regulation

  /*** DYNAMIC C_ACTIVITY BASED ON SYSTEM MODE ***/
  double C_activity_;  ///< CO₂ production rate per astronaut based on mode

  /*** MODE CHANGE DETECTION ***/
  std::string previous_mode_; ///< Stores the last mode to detect changes

  /*** SERVICE AVAILABILITY TRACKING ***/
  int service_unavailable_count_; ///< Tracks how many times the service was unavailable

  /*** STATUS MESSAGE ***/
  space_station_eclss::msg::CdraStatus cdra; ///< CDRA status message

  /*** ROS INTERFACES ***/
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_publisher_; ///< Publishes /temperature
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_publisher_;  ///< Publishes /pipe_pressure
  rclcpp::Client<space_station_eclss::srv::CrewQuarters>::SharedPtr desiccant_bed_client_; ///< Calls Desiccant Bed service when needed
  rclcpp::TimerBase::SharedPtr timer_; ///< Timer for periodic updates
  rclcpp::Publisher<space_station_eclss::msg::CdraStatus>::SharedPtr cdra_status_publisher_; ///< Publishes CDRA status
  rclcpp::Publisher<space_station_eclss::msg::AirData>::SharedPtr air_quality_publisher_;

};


#endif // space_station_eclss_AIR_COLLECTOR_TANK_HPP_
