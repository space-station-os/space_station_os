#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <string>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/msg/vector3.hpp>


//std::string mode_demo;


class TorqueCollectorNode : public rclcpp::Node
{
public:
    TorqueCollectorNode() : Node("torque_collector_node")
    {
        //registor PropulsionDevice
        prop.push_back(PropulsionDevice("nauka_mainengine",3900,       -13,-1.5,-10,0,0,+1));
        prop.push_back(PropulsionDevice("dragon_mainengine",8000,        2,0,13,    0,0,-1));
        prop.push_back(PropulsionDevice("progress_mainengine",2950,    -13,-1,10,   0,0,-1));
        prop.push_back(PropulsionDevice("progress_thruster_zenith",260,-13,-1,10,   0,0,-1));
        prop.push_back(PropulsionDevice("progress_thruster_nadir",260, -13,-1,10,   0,0,+1));
        prop.push_back(PropulsionDevice("soyuz_mainengine",2950,        -8,0,-8,    0,0,+1));
        prop.push_back(PropulsionDevice("zvezda_mainengine",3000,     -26,0,0,    +1,0,0));
        prop.push_back(PropulsionDevice("zvezda_thruster_zenith",300,  -26,0,0,     0,0,-1));
        prop.push_back(PropulsionDevice("zvezda_thruster_nadir",300,   -26,0,0,     0,0,+1));
        //prop.push_back(PropulsionDevice("nauka_thruster_zenith",800,       -13,-1.5,-10,0,0,+1,1.0,"stuck-on"));
        prop.push_back(PropulsionDevice("nauka_thruster_zenith",800,   -13,-1.5,-10,0,0,-1));

        pub_torque_control = this->create_publisher<geometry_msgs::msg::Vector3>("gnc/thr_torque_cmd", 1);

        did_receive_thrust = false;

        sub_str_target_thrust = this->create_subscription<std_msgs::msg::String>(
            "gnc/str_target_thrust", 10, 
            std::bind(&TorqueCollectorNode::callback_target_thrust, this, std::placeholders::_1));
        T_callback = 0.1;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(T_callback*1000.0)), 
            std::bind(&TorqueCollectorNode::callback_timer_torque_collctor, this));
    }

    class PropulsionDevice
    {
        public: 
            PropulsionDevice(std::string name_,double Fthrust_,
                    double px,double py,double pz,
                    double dirx,double diry,double dirz,
                    double throttle_=0.0,std::string status_ = "nominal"){
                this->name = name_;
                this->Fthrust = Fthrust_;
                this->throttle = throttle_;
                this->status = status_;
                this->p = tf2::Vector3(px,py,pz);
                double norm = sqrtf(dirx*dirx+diry*diry+dirz*dirz); 
                this->Fdir = tf2::Vector3(dirx/norm,diry/norm,dirz/norm);

            }
            std::string name; 
            double throttle; // 0.0 <= rate <= 1.0 for thruster, 0.0 or 1.0 for engine (for the most of time)
            std::string status; //"nominal" "stuck-on" "stuck-off"
            double Fthrust; //
            tf2::Vector3 p; //position
            tf2::Vector3 Fdir; //e.g., (1,0,0):=thrust to obtain force in +x direction
                                // (0,0,1):= engine installed in **nadir** direction is fired
    };

private:

    //defines PropulsionDevice (engine, thruster)
    std::vector<PropulsionDevice> prop;
    
    //ros2 stuff
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_torque_control;
    geometry_msgs::msg::Vector3 torque_control;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_str_target_thrust;
    rclcpp::TimerBase::SharedPtr timer_;

    
    //define parameters for dynamics
    double T_callback; //in seconds
    double T_step; // in seconds
    bool did_receive_thrust; // in seconds

    //utility function 
    void parsing_target_thrust(std::string input,std::string& which,double& throttle_) {
        std::stringstream ss(input);
        std::string item;
        std::vector<std::string> strs;
        while (std::getline(ss, item, ',')) {
            //strs.push_back(std::stoi(item));
            strs.push_back(item);
        }
        which = strs[0];
        if(strs[1] == "on") throttle_ = 1.0;
        else if(strs[1] == "off") throttle_ = 0.0;
        else{
            throttle_ = std::stoi(strs[1]);
        }        
    }


    void callback_target_thrust(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received input: %s", msg->data.c_str());
        did_receive_thrust = true;
        std::string which;
        double throttle_;
        parsing_target_thrust(msg->data,which,throttle_);
        for (unsigned int ith = 0; ith < prop.size(); ith++){
            if(which == prop[ith].name){
                prop[ith].throttle = throttle_;
                printf("throttle updated: which %s, throttle %f\n",which.c_str(), throttle_);
                break;
            }
        }
    }

    void callback_timer_torque_collctor() 
    {
        //RCLCPP_INFO(this->get_logger(), "Timer callback triggered");

        if(!did_receive_thrust)
            return; // wait till thrust is forwarded. 
                    //TODO: simulation initialization order must be considered 


        tf2::Vector3 tausum(0.0,0.0,0.0); 
        for (unsigned int ith = 0; ith < prop.size(); ith++){
            tf2::Vector3 Fcur =  prop[ith].throttle * prop[ith].Fthrust * prop[ith].Fdir;
            //tau = r (cross) F
            tf2::Vector3 taucur = prop[ith].p.cross(Fcur);
            tausum = tausum + taucur;
            //RCLCPP_INFO(this->get_logger(), "i=%d, name %s, taucur %f %f %f",ith,prop[ith].name.c_str(),
            //    taucur.x(),taucur.y(),taucur.z());
        }
        torque_control.x = tausum.x();
        torque_control.y = tausum.y();
        torque_control.z = tausum.z();
        pub_torque_control->publish(torque_control);        
        //RCLCPP_INFO(this->get_logger(), "got through?");
    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");  
    printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    auto node = std::make_shared<TorqueCollectorNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();


    return 0;
}

