
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <cmath>

using namespace std::chrono_literals;

class MockSolarController : public rclcpp::Node
{
public:
  MockSolarController()
  : Node("mock_solar_controller")
  {
    // ---- Parameters ----
    orbit_period_s_          = this->declare_parameter<double>("orbit_period_s", 90.0);     // 90 s == 90 min
    eclipse_duration_s_      = this->declare_parameter<double>("eclipse_duration_s", 35.0);
    ramp_window_s_           = this->declare_parameter<double>("ramp_window_s", 8.0);
    p_max_kw_                = this->declare_parameter<double>("p_max_kw", 84.0);           // ISS-class
    derate_with_beta_cosine_ = this->declare_parameter<bool>("derate_with_beta_cosine", true);

    temp_ambient_c_          = this->declare_parameter<double>("temp_ambient_c", 20.0);
    temp_per_kw_c_           = this->declare_parameter<double>("temp_per_kw_c", 0.4);
    temp_time_constant_s_    = this->declare_parameter<double>("temp_time_constant_s", 20.0);

    beta_max_deg_            = this->declare_parameter<double>("beta_max_deg", 75.0);
    beta_lock_limit_deg_     = this->declare_parameter<double>("beta_lock_limit_deg", 72.0);
    precession_orbits_       = this->declare_parameter<int>("beta_precession_orbits", 10);

    // SSU parameters (regulation before EPS)
    ssu_v_setpoint_v_        = this->declare_parameter<double>("ssu_v_setpoint_v", 160.0);   // ~160 Vdc
    ssu_v_ripple_v_          = this->declare_parameter<double>("ssu_v_ripple_v", 0.6);       // small ripple
    ssu_efficiency_          = this->declare_parameter<double>("ssu_efficiency", 0.985);     // SSU conversion loss

    // ---- Publishers ----
    
    v_out_pub_  = this->create_publisher<std_msgs::msg::Float64>("/solar_controller/ssu_voltage_v", 10); // V
    p_out_pub_  = this->create_publisher<std_msgs::msg::Float64>("/solar_controller/ssu_power_w", 10);  // W
    i_out_pub_  = this->create_publisher<std_msgs::msg::Float64>("/solar_controller/ssu_current_a", 10);// A

    temp_pub_   = this->create_publisher<std_msgs::msg::Float64>("/solar_controller/panel_temperature", 10);
    beta_pub_   = this->create_publisher<std_msgs::msg::Float64>("/solar_controller/sun_beta_deg", 10);
    diag_pub_   = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/eps/diagnostics", 10);

    // 1 Hz: 1 s ≙ 1 min
    timer_ = this->create_wall_timer(1000ms, std::bind(&MockSolarController::tick, this));

    RCLCPP_INFO(this->get_logger(),
      "MockSolarController: orbit=%.0fs, eclipse=%.0fs, Pmax=%.1f kW, SSU=~%.0f V",
      orbit_period_s_, eclipse_duration_s_, p_max_kw_, ssu_v_setpoint_v_);
  }

private:
  
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr v_out_pub_;   // regulated voltage (SSU) [V]
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr p_out_pub_;   // available power after SSU [W]
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr i_out_pub_;   // available current after SSU [A]
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temp_pub_;    // panel temperature [°C]
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr beta_pub_;    // debug beta angle [deg]
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  
  double orbit_period_s_{90.0};
  double eclipse_duration_s_{35.0};
  double ramp_window_s_{8.0};
  double p_max_kw_{84.0};
  bool   derate_with_beta_cosine_{true};

  double temp_ambient_c_{20.0};
  double temp_per_kw_c_{0.4};
  double temp_time_constant_s_{20.0};

  double beta_max_deg_{75.0};
  double beta_lock_limit_deg_{72.0};
  int    precession_orbits_{10};

  
  double ssu_v_setpoint_v_{160.0};
  double ssu_v_ripple_v_{0.6};
  double ssu_efficiency_{0.985};

 
  int    time_step_{0};
  int    orbit_count_{0};
  double panel_temp_c_{20.0};

  
  static double deg2rad(double d) { return d * M_PI / 180.0; }

  static double cosine_rise(double x_0_to_1) {
    if (x_0_to_1 <= 0.0) return 0.0;
    if (x_0_to_1 >= 1.0) return 1.0;
    return 0.5 * (1.0 - std::cos(M_PI * x_0_to_1));
  }
  static double cosine_fall(double x_0_to_1) {
    if (x_0_to_1 <= 0.0) return 1.0;
    if (x_0_to_1 >= 1.0) return 0.0;
    return 0.5 * (1.0 + std::cos(M_PI * x_0_to_1));
  }

  double illumination_factor(double t) 
  {
    const double T = orbit_period_s_;
    const double Te = eclipse_duration_s_;
    const double Tr = std::max(0.0, std::min(ramp_window_s_, (T - Te) * 0.45));

    if (t < Te) return 0.0;

    double f = 1.0;
    if (t < Te + Tr) {
      double x = (t - Te) / Tr;
      f = cosine_rise(x);
    }
    if (t > T - Tr) {
      double x = (T - t) / Tr;     
      f *= cosine_fall(1.0 - x);  
    }
    return std::clamp(f, 0.0, 1.0);
  }

  double sun_beta_deg() const
  {
    if (precession_orbits_ <= 0) return 0.0;
    double phase = 2.0 * M_PI * (static_cast<double>(orbit_count_) / static_cast<double>(precession_orbits_));
    return beta_max_deg_ * std::sin(phase);
  }

  void publish_diag(const std::string& name, uint8_t level, const std::string& msg)
  {
    diagnostic_msgs::msg::DiagnosticStatus d;
    d.name = name;
    d.level = level;
    d.message = msg;
    diag_pub_->publish(d);
  }

  void tick()
  {
    
    const double T = orbit_period_s_;
    const double t = std::fmod(static_cast<double>(time_step_), T);
    if (t < 1e-9 && time_step_ != 0) { orbit_count_++; }

   
    double illum = illumination_factor(t);
    double beta_deg = sun_beta_deg();
    double beta_derate = 1.0;
    if (derate_with_beta_cosine_) {
      beta_derate = std::cos(deg2rad(std::min(89.9, std::abs(beta_deg))));
      beta_derate = std::max(0.0, beta_derate);
    }

    bool gimbal_lock = std::abs(beta_deg) >= beta_lock_limit_deg_;
    if (gimbal_lock) {
      // extra hit if in gimbal lock
      beta_derate *= 0.6;
      publish_diag("SolarArrayGimbal", diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                   "Gimbal lock detected: |beta| high");
    } else {
      publish_diag("SolarArrayGimbal", diagnostic_msgs::msg::DiagnosticStatus::OK,
                   "Gimbal nominal");
    }

  
    double p_array_kw = p_max_kw_ * illum * beta_derate;   // kW
    double p_array_w  = p_array_kw * 1000.0;               // W

   
    
    double v_ssu_v = 0.0;
    double p_ssu_w = 0.0;
    double i_ssu_a = 0.0;

    if (p_array_w > 1.0) {
      
      double ripple = ssu_v_ripple_v_ * std::sin(2.0 * M_PI * (t / T) * 4.0); 
      v_ssu_v = ssu_v_setpoint_v_ + ripple;

   
      p_ssu_w = p_array_w * ssu_efficiency_;
      
      i_ssu_a = p_ssu_w / std::max(1e-3, v_ssu_v);

      publish_diag("SSU", diagnostic_msgs::msg::DiagnosticStatus::OK,
                   "SSU regulating to setpoint");
    } else {
     
      v_ssu_v = 0.0;
      p_ssu_w = 0.0;
      i_ssu_a = 0.0;

      publish_diag("SSU", diagnostic_msgs::msg::DiagnosticStatus::WARN,
                   "No solar input (eclipse)");
    }

    
    const double dt = 1.0; // s
    double target_temp_c = temp_ambient_c_ + temp_per_kw_c_ * (p_array_w / 1000.0);
    double alpha = dt / std::max(1e-6, temp_time_constant_s_);
    panel_temp_c_ += alpha * (target_temp_c - panel_temp_c_);


    std_msgs::msg::Float64 vmsg; vmsg.data = v_ssu_v;  v_out_pub_->publish(vmsg);
    std_msgs::msg::Float64 pmsg; pmsg.data = p_ssu_w;  p_out_pub_->publish(pmsg);
    std_msgs::msg::Float64 imsg; imsg.data = i_ssu_a;  i_out_pub_->publish(imsg);

    std_msgs::msg::Float64 tmsg; tmsg.data = panel_temp_c_; temp_pub_->publish(tmsg);
    std_msgs::msg::Float64 bmsg; bmsg.data = beta_deg;      beta_pub_->publish(bmsg);

  
    if (illum <= 1e-6) {
      RCLCPP_DEBUG(this->get_logger(),
        "[ECLIPSE] P_arr=%.1f kW, V_ssu=%.1f V, I_ssu=%.1f A, T=%.1f C, beta=%.1f°, orbit=%d",
        p_array_kw, v_ssu_v, i_ssu_a, panel_temp_c_, beta_deg, orbit_count_);
    } else {
      RCLCPP_DEBUG(this->get_logger(),
        "[SUN]     P_arr=%.1f kW (derate=%.2f) -> SSU: V=%.1f V, I=%.1f A, P=%.0f W, T=%.1f C, beta=%.1f°, orbit=%d",
        p_array_kw, beta_derate, v_ssu_v, i_ssu_a, p_ssu_w, panel_temp_c_, beta_deg, orbit_count_);
    }
    time_step_++;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockSolarController>());
  rclcpp::shutdown();
  return 0;
}
