#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <string>
#include <vector>
#include <SDL2/SDL.h>


std::string mode_demo;
tf2::Vector3 angvel_body;

std::vector<std::string> msg_propulsion;
std::vector<std::string> print_propulsion;
std::vector<bool> is_propulsion_on;

const double angvel_th = 0.2 / 180.0 * M_PI; //rad/sec


std::mutex sdl_mutex;
SDL_Window* window = nullptr;
SDL_Renderer* renderer = nullptr;
SDL_Texture* texture = nullptr;
bool update_texture = false;

void display_image_with_sdl(const cv::Mat &image) {
    std::lock_guard<std::mutex> lock(sdl_mutex);

    if (window == nullptr) {
        SDL_Init(SDL_INIT_VIDEO);
        window = SDL_CreateWindow("Space Station GNC Demo", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, image.cols, image.rows, SDL_WINDOW_SHOWN);
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    }

    if (texture) {
        SDL_DestroyTexture(texture);
    }

    texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, image.cols, image.rows);

    SDL_UpdateTexture(texture, nullptr, image.data, image.step);
    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, nullptr, nullptr);
    SDL_RenderPresent(renderer);
    update_texture = true;
}


void sdl_event_loop() {
    SDL_Event event;
    while (true) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                SDL_DestroyTexture(texture);
                SDL_DestroyRenderer(renderer);
                SDL_DestroyWindow(window);
                SDL_Quit();
                return;
            }
        }

        if (update_texture) {
            std::lock_guard<std::mutex> lock(sdl_mutex);
            SDL_RenderClear(renderer);
            SDL_RenderCopy(renderer, texture, nullptr, nullptr);
            SDL_RenderPresent(renderer);
            update_texture = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}


class DemoCrisisNode : public rclcpp::Node
{
public:
    DemoCrisisNode() : Node("demo_crisis_node"){
        sub_topic_control_media = this->create_subscription<std_msgs::msg::String>(
            "gnc/topic_control_media", 10, std::bind(&DemoCrisisNode::callback_control_media, this, std::placeholders::_1));
        sub_angvel_body = this->create_subscription<geometry_msgs::msg::Vector3>(
            "gnc/angvel_body", 1, std::bind(&DemoCrisisNode::callback_angvel_body, this, std::placeholders::_1));


        std::thread sdl_thread(sdl_event_loop);
        sdl_thread.detach();

    }

private:

    void callback_control_media(const std_msgs::msg::String::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "Received input: %s", msg->data.c_str());
        std::string input = msg->data;
        std::string img_path;

        if (input == "000_init")
        {
            img_path = "src/space_station_os/space_station_gnc/media/001_start.png";            
        }
        else if (input == "100_emergency_begin")
        {
            img_path = "src/space_station_os/space_station_gnc/media/100_emergency_begin.png";
        }
        else if (input == "102_emergency_reaction")
        {
            img_path = "src/space_station_os/space_station_gnc/media/102_emergency_reaction.png";
        }
        else
        {
            cv::destroyAllWindows();
            return;
        }

        cv::Mat image = cv::imread(img_path, cv::IMREAD_COLOR);
        if (image.empty()) {
            std::cerr << "Error: Unable to load image: " << img_path << std::endl;
            return;
        }
	RCLCPP_INFO(this->get_logger(), "Image %s loaded\n", img_path.c_str());
        display_image_with_sdl(image);
    }

    void callback_angvel_body(const geometry_msgs::msg::Vector3::SharedPtr msg){
        angvel_body.setValue(msg->x,msg->y,msg->z);
    }


    void display_image(const std::string &image_path)
    {
        cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
        if (image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Image %s not found or unable to load.", image_path.c_str());
            return;
        }
        cv::imshow("Display window", image);
        cv::waitKey(100); // non-blocking millisec
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_topic_control_media;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_angvel_body;

};

double get_time_double(rclcpp::Node::SharedPtr node) {
    rclcpp::Clock::SharedPtr clock = node->get_clock();
    rclcpp::Time tclock = clock->now();
    return (double)(tclock.seconds()) + (double)(tclock.nanoseconds()) * 0.001 * 0.001 * 0.001;
}

bool did_find_num(std::string str, int num) {
    std::stringstream ss(str);
    std::string item;
    std::vector<int> numbers;
    while (std::getline(ss, item, ',')) {
        numbers.push_back(std::stoi(item));
    }

    if (std::find(numbers.begin(), numbers.end(), num) != numbers.end()) {
        return true;
    } 
    return false;
}

void user_input_thread(rclcpp::Node::SharedPtr node)
{
    auto pub_control_media = node->create_publisher<std_msgs::msg::String>("gnc/topic_control_media", 1);
    auto pub_target_thrust = node->create_publisher<std_msgs::msg::String>("gnc/str_target_thrust", 10);
    auto pub_t_fwd_sim = node->create_publisher<std_msgs::msg::Float64>("gnc/t_fwd_sim", 10);

    auto pub_angvel_overwrite = node->create_publisher<geometry_msgs::msg::Vector3>("gnc/angvel_overwrite", 1);
    auto pub_attitude_overwrite = node->create_publisher<geometry_msgs::msg::Quaternion>("gnc/attitude_overwrite", 1);

    auto msg = std::make_shared<std_msgs::msg::String>();
    std::string userinput;

    rclcpp::Rate loop_rate(10.0); // 10 Hz

    double tinit = get_time_double(node);
    double tcur = get_time_double(node);


    msg_propulsion.push_back("nauka_mainengine"); //0
    print_propulsion.push_back("Nauka main engine");
    is_propulsion_on.push_back(false);
    msg_propulsion.push_back("dragon_mainengine"); //1
    print_propulsion.push_back("Crew-2 Dragon main engine");
    is_propulsion_on.push_back(false);
    msg_propulsion.push_back("progress_mainengine"); //2
    print_propulsion.push_back("Progress main engine");
    is_propulsion_on.push_back(false);
    msg_propulsion.push_back("progress_thruster_zenith"); //3
    print_propulsion.push_back("Progress zenith thruster");
    is_propulsion_on.push_back(false);
    msg_propulsion.push_back("progress_thruster_nadir"); //4
    print_propulsion.push_back("Progress nadir thruster");
    is_propulsion_on.push_back(false); ////Note: when it goes "on", the ISS is contaminated by thruster plumes...
    msg_propulsion.push_back("soyuz_mainengine"); //5
    print_propulsion.push_back("Soyuz main engine");
    is_propulsion_on.push_back(false);
    msg_propulsion.push_back("zvezda_mainengine"); //6
    print_propulsion.push_back("Zvezda main engine");
    is_propulsion_on.push_back(false);////Note: when it goes "on", the iss will die...
    msg_propulsion.push_back("zvezda_thruster_zenith"); //7
    print_propulsion.push_back("Zvezda zenith thruster");
    is_propulsion_on.push_back(false);
    msg_propulsion.push_back("zvezda_thruster_nadir"); //8
    print_propulsion.push_back("Zvezda nadir thruster");
    is_propulsion_on.push_back(false);
    msg_propulsion.push_back("nauka_thruster_zenith"); //8
    print_propulsion.push_back("Nauka zenith thruster");
    is_propulsion_on.push_back(false);
    int nth =  (int)is_propulsion_on.size();


    while (rclcpp::ok())
    {
        RCLCPP_INFO(node->get_logger(), "mode_demo: %s",mode_demo.c_str());
        if(mode_demo == "000_init"){
            msg->data = "000_init";
            pub_control_media->publish(*msg);
            printf("Press `ok` then `enter key` to start demo\n");
            std::cin >> userinput;


            geometry_msgs::msg::Vector3 angvel_init;
            angvel_init.x = 0.0;
            //angvel_init.y = +0.22 / 180.0 * M_PI;
            angvel_init.y = -0.22 / 180.0 * M_PI;
            angvel_init.z = 0.0;
            pub_angvel_overwrite->publish(angvel_init);        

            geometry_msgs::msg::Quaternion attitude_init;
            tf2::Quaternion attinit; // pose
            //attinit.setRPY(0.0,1.72/180.0*M_PI,0.0);
            attinit.setRPY(0.0,-1.72/180.0*M_PI,0.0);
            //attinit.setRPY(0.0,11.97/180.0*M_PI,0.0);
            attitude_init.x = attinit.x();
            attitude_init.y = attinit.y();
            attitude_init.z = attinit.z();
            attitude_init.w = attinit.w();
            pub_attitude_overwrite->publish(attitude_init);        

            msg->data = "nauka_thruster_zenith,on";
            //is_propulsion_on[0] = true;
            is_propulsion_on[9] = true; //Nauka zenith thruster
            //msg->data = "nauka_mainengine,on";
            pub_target_thrust->publish(*msg);
            mode_demo = "100_emergency_begin";
        }
        else if(mode_demo == "100_emergency_begin"){
            printf("`Commander, misinjection from the docked module is detected, \n over...`\n");            
            printf("`Nauka main engine with 3800N is firing unexpectedly.\n It may last for about 15 minutes.\n over...`\n");            
            tinit = get_time_double(node);
            msg->data = "100_emergency_begin";
            pub_control_media->publish(*msg);
            mode_demo = "101_emergency_begin";
            printf("Are you ready to take actions? (Answer `ok` to react)\n");            
            std::cin >> userinput;
        }
        else if(mode_demo == "101_emergency_begin"){
            tcur = get_time_double(node);
            if(5.0 < tcur - tinit){
                printf("Emergency situation.\n");
                printf("\n");
                printf("Nauka is forced to be commanded to bring the body to nadir, causing pitching up.");
                //printf("The main engine of Nauka is experiencing a stuck-on failure.");
                printf("Commander, this resulting acceleration and rotation has to be stopped.\n");
                printf("\n");
                printf("  Can we use the engines of the docked spacecraft to remedy the situation?\n");
                printf("\n");
                printf("Yes... we currently have a Crew Dragon, a Progress MS-17 and a Soyuz MS-18 docked. Service module Zvezda is also available.\n");
                printf("We could also consider using thrusters.\n");
                mode_demo = "102_emergency_reaction";
                msg->data = "102_emergency_reaction";
                pub_control_media->publish(*msg);
            }
        }
        else if(mode_demo == "102_emergency_reaction" || mode_demo == "104_emergency_reaction2"){
            if(mode_demo == "102_emergency_reaction"){
                printf(" Nauka thrusters have 800N of force to bring the body to nadir, causing pitching up.\n");                
                //printf(" The Nauka main engine has 3900N to bring the body to zenith, causing pitching down.\n");                
                printf(" - Which engine should we fire?\n");                
            }
            else if(mode_demo == "104_emergency_reaction2")
                printf(" - Which engine should we fire or stop?\n");
            printf("    1: Crew-2 Dragon, 8000N (now ");
            //printf(is_dragon_mainengine_on ? "on)\n" : "off)\n");
            printf(is_propulsion_on[1] ? "on)\n" : "off)\n");
            printf("    2: Progress MS-17, 2950N (now ");
            //printf(is_progress_mainengine_on ? "on)\n" : "off)\n");
            printf(is_propulsion_on[2] ? "on)\n" : "off)\n");
            printf("    3: Progress MS-17 attitude control thruster to zenith, 130N x 2 (now ");
            //printf(is_progress_thruster_zenith_on ? "on)\n" : "off)\n");
            printf(is_propulsion_on[3] ? "on)\n" : "off)\n");
            printf("    4: Progress MS-17 attitude control thruster to nadir, 130N x 2 (now ");
            //printf(is_progress_thruster_nadir_on ? "on)\n" : "off)\n");
            printf(is_propulsion_on[4] ? "on)\n" : "off)\n");
            printf("    5: Soyuz MS18, 2950N (now ");
            //printf(is_soyuz_mainengine_on ? "on)\n" : "off)\n");
            printf(is_propulsion_on[5] ? "on)\n" : "off)\n");
            printf("    6: Zvezda main engine, 3000N (now ");
            printf(is_propulsion_on[6] ? "on)\n" : "off)\n");
            printf("    7: Zvezda attitude control thruster to zenith, 150N x 2 (now ");
            printf(is_propulsion_on[7] ? "on)\n" : "off)\n");
            printf("    8: Zvezda attitude control thruster to nadir, 150N x 2 (now ");
            printf(is_propulsion_on[8] ? "on)\n" : "off)\n");
            printf("\n");
            if(mode_demo == "102_emergency_reaction")
                printf("Enter engines to fire now... Or enter `0` for do nothing.\n");
            else if(mode_demo == "104_emergency_reaction2")
                printf("Enter engines to fire or stop now...\n");
            std::cin >> userinput;

            for(int ith = 1; ith < nth; ith++){
                if(did_find_num(userinput,ith)){ // toggle dragon main engine
                    is_propulsion_on[ith] = !is_propulsion_on[ith];
                    if(is_propulsion_on[ith]){
                        printf("%s, turned on\n",print_propulsion[ith].c_str());
                        msg->data = msg_propulsion[ith] + ",on";
                    }
                    else{
                        printf("%s, turned off\n",print_propulsion[ith].c_str());
                        msg->data = msg_propulsion[ith] + ",off";
                    }
                    pub_target_thrust->publish(*msg);
                }
            }

            if(mode_demo == "102_emergency_reaction"){
                if(did_find_num(userinput,0)){ //forward simulation
                    mode_demo = "103_forward_sim";
                }
                if(14.0 * 60.0 + 10.0 < tcur - tinit){
                    mode_demo = "103_forward_sim";
                }
            }
            else if(mode_demo == "104_emergency_reaction2"){
                bool is_something_on = false;
                for(int ith = 1; ith < nth; ith++){
                    if(is_propulsion_on[ith])
                        is_something_on = true;
                }
                if(!is_something_on){
                    //if here, everyting is off
                    printf("everything is off\n");
                    if(-angvel_th <= angvel_body.x() && angvel_body.x() <= angvel_th)
                        if(-angvel_th <= angvel_body.y() && angvel_body.y() <= angvel_th)
                            if(-angvel_th <= angvel_body.z() && angvel_body.z() <= angvel_th){
                                printf("Body has stopped! Well done!!!\n");
                                mode_demo = "201_bring_back_orientation";
                            }
                }
            }
        }
        else if(mode_demo == "103_forward_sim"){
            tcur = get_time_double(node);
            auto messagedouble = std_msgs::msg::Float64();
            double t_fwd = 14.0 * 60.0 + 10.0 - (tcur - tinit);
            messagedouble.data = t_fwd; //
            //printf("Forwarding: %f min", messagedouble.data / 60.0);
            pub_t_fwd_sim->publish(messagedouble);
            tinit = tinit - t_fwd;
            msg->data = "nauka_thruster_zenith,off";
            //msg->data = "nauka_mainengine,off";
            pub_target_thrust->publish(*msg);
            is_propulsion_on[9] = false; //Nauka zenith thruster

            printf("Nauka's main zenith thruster has stopped. The propellant has run out by now.\n");
            printf("Now rotation needs to be stopped.\n");

            mode_demo = "104_emergency_reaction2"; //
        } // end of if(mode_demo == "???"){
        else if(mode_demo == "201_bring_back_orientation"){
            printf("...\n");
            std::cin >> userinput;
        }
        loop_rate.sleep();
    }//end of     while (rclcpp::ok())

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DemoCrisisNode>();
    
    mode_demo = "000_init";

    std::thread user_thread(user_input_thread, node);

    //user_thread.join();
    rclcpp::spin(node);
    rclcpp::shutdown();
    user_thread.join();

    return 0;
}

