#pragma once

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <bullet/LinearMath/btVector3.h>
#include <bullet/LinearMath/btQuaternion.h>

namespace sun {

// Compute Julian Date from ROS time
inline double computeJulianDate(const rclcpp::Time &now_ros) {
    const double unix_sec = now_ros.seconds();
    return (unix_sec / 86400.0) + 2440587.5;
}

// Compute Julian centuries since J2000.0
inline double computeJulianCenturies(double jd) {
    return (jd - 2451545.0) / 36525.0;
}

// Obliquity of the ecliptic (radians)
inline double obliquityEcliptic(double T) {
    double eps_arcsec = 23.0 * 3600.0 + 27.0 * 60.0 + 8.26
                      - 46.845 * T - 0.0059 * T * T
                      + 0.00181 * T * T * T;
    return (eps_arcsec / 3600.0) * M_PI / 180.0;
}

// Mean solar longitude (radians)
inline double meanSolarLongitude(double T) {
    double L = 280.46 + 36000.77 * T;
    return fmod(L, 360.0) * M_PI / 180.0;
}

// Mean anomaly (radians)
inline double solarMeanAnomaly(double T) {
    double g = 357.528 + 35999.05 * T;
    return fmod(g, 360.0) * M_PI / 180.0;
}

// Ecliptic longitude (radians)
inline double eclipticLongitude(double L, double g) {
    return L + (1.915 * sin(g) + 0.020 * sin(2 * g)) * M_PI / 180.0;
}

// Unit sun vector in ECI frame
inline btVector3 sunVectorECI(double T) {
    double L = meanSolarLongitude(T);
    double g = solarMeanAnomaly(T);
    double lambda = eclipticLongitude(L, g);
    double epsilon = obliquityEcliptic(T);

    double x = cos(lambda);
    double y = cos(epsilon) * sin(lambda);
    double z = sin(epsilon) * sin(lambda);

    btVector3 s(x, y, z);
    s.normalize();
    return s;
}

} // namespace sun


namespace space_station_thermal_control {

class SunVectorProvider : public rclcpp::Node
{
public:
    SunVectorProvider();

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void tryComputeSunVector();

    geometry_msgs::msg::Point latest_position_;
    geometry_msgs::msg::Quaternion latest_quat_;
    bool got_pose_ = false;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr sun_vec_pub_;
};

} // namespace space_station_thermal_control
