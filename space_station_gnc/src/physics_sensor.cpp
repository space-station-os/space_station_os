#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <random>

class PhysicsSensor : public rclcpp::Node {
public:
    PhysicsSensor()
        : Node("physics_sensor") {
        
        // Parameters for sensor noise
        this->declare_parameter("imu_bias_x", 0.0);
        this->declare_parameter("imu_bias_y", 0.0);
        this->declare_parameter("imu_bias_z", 0.0);
        this->declare_parameter("imu_noise_sigma", 0.002);
        
        this->declare_parameter("startracker_bias_x", 0.0);
        this->declare_parameter("startracker_bias_y", 0.0);
        this->declare_parameter("startracker_bias_z", 0.0);
        this->declare_parameter("startracker_bias_w", 0.0);
        this->declare_parameter("startracker_noise_sigma", 0.001);
        
        this->declare_parameter("gps_bias_x", 0.0);
        this->declare_parameter("gps_bias_y", 0.0);
        this->declare_parameter("gps_bias_z", 0.0);
        this->declare_parameter("gps_noise_sigma", 0.1);

        // 100Hz subscriber for /gnc/angvel_body
        imu_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/gnc/angvel_body", rclcpp::QoS(10),
            std::bind(&PhysicsSensor::callback_angvel, this, std::placeholders::_1));

        // 0.1Hz subscriber for /gnc/attitude_LVLH
        startracker_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
            "/gnc/attitude_LVLH", rclcpp::QoS(10),
            std::bind(&PhysicsSensor::callback_attitude, this, std::placeholders::_1));

        // 1Hz subscriber for /gnc/position_ECEF
        gps_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/gnc/position_ECEF", rclcpp::QoS(10),
            std::bind(&PhysicsSensor::callback_position, this, std::placeholders::_1));

        // Publishers for raw data
        imu_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/gnc/imu_raw", 10);
        startracker_pub_ = this->create_publisher<geometry_msgs::msg::Quaternion>("/gnc/startracker_raw", 10);
        gps_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/gnc/gps_raw", 10);

        // Timers to simulate the data processing rates
        imu_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), // 100Hz = 10ms interval
            std::bind(&PhysicsSensor::timer_callback_process_imu, this));

        startracker_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 0.1Hz = 10 seconds interval TODO: at least this is too fast
            std::bind(&PhysicsSensor::timer_callback_process_startracker, this));

        gps_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), // 1Hz = 1 second interval
            std::bind(&PhysicsSensor::timer_callback_process_gps, this));
    }

private:
    std::default_random_engine generator_;

    double generate_noise(double sigma) {
        std::normal_distribution<double> distribution(0.0, sigma);
        return distribution(generator_);
    }

    void callback_angvel(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        latest_imu_ = *msg;
    }

    void callback_attitude(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
        latest_startracker_ = *msg;
    }

    void callback_position(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        latest_gps_ = *msg;
    }

    void timer_callback_process_imu() {
        latest_imu_.x += this->get_parameter("imu_bias_x").as_double() + generate_noise(this->get_parameter("imu_noise_sigma").as_double());
        latest_imu_.y += this->get_parameter("imu_bias_y").as_double() + generate_noise(this->get_parameter("imu_noise_sigma").as_double());
        latest_imu_.z += this->get_parameter("imu_bias_z").as_double() + generate_noise(this->get_parameter("imu_noise_sigma").as_double());
        RCLCPP_INFO(this->get_logger(), "Processing IMU Data: x=%.3f, y=%.3f, z=%.3f", latest_imu_.x, latest_imu_.y, latest_imu_.z);
        imu_pub_->publish(latest_imu_);
    }

    void timer_callback_process_startracker() {
        latest_startracker_.x += this->get_parameter("startracker_bias_x").as_double() + generate_noise(this->get_parameter("startracker_noise_sigma").as_double());
        latest_startracker_.y += this->get_parameter("startracker_bias_y").as_double() + generate_noise(this->get_parameter("startracker_noise_sigma").as_double());
        latest_startracker_.z += this->get_parameter("startracker_bias_z").as_double() + generate_noise(this->get_parameter("startracker_noise_sigma").as_double());
        latest_startracker_.w += this->get_parameter("startracker_bias_w").as_double() + generate_noise(this->get_parameter("startracker_noise_sigma").as_double());
        RCLCPP_INFO(this->get_logger(), "Processing Star Tracker Data: x=%.3f, y=%.3f, z=%.3f, w=%.3f", latest_startracker_.x, latest_startracker_.y, latest_startracker_.z, latest_startracker_.w);
        startracker_pub_->publish(latest_startracker_);
    }

    void timer_callback_process_gps() {
        latest_gps_.x += this->get_parameter("gps_bias_x").as_double() + generate_noise(this->get_parameter("gps_noise_sigma").as_double());
        latest_gps_.y += this->get_parameter("gps_bias_y").as_double() + generate_noise(this->get_parameter("gps_noise_sigma").as_double());
        latest_gps_.z += this->get_parameter("gps_bias_z").as_double() + generate_noise(this->get_parameter("gps_noise_sigma").as_double());
        RCLCPP_INFO(this->get_logger(), "Processing GPS Data: x=%.3f, y=%.3f, z=%.3f", latest_gps_.x, latest_gps_.y, latest_gps_.z);
        gps_pub_->publish(latest_gps_);
    }

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr startracker_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr gps_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr startracker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gps_pub_;

    rclcpp::TimerBase::SharedPtr imu_timer_;
    rclcpp::TimerBase::SharedPtr startracker_timer_;
    rclcpp::TimerBase::SharedPtr gps_timer_;

    geometry_msgs::msg::Vector3 latest_imu_;
    geometry_msgs::msg::Quaternion latest_startracker_;
    geometry_msgs::msg::Vector3 latest_gps_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PhysicsSensor>());
    rclcpp::shutdown();
    return 0;
}

