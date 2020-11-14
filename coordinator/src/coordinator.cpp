#include "coordinator/coordinator.hpp"


Coordinator::Coordinator()
{
    command_rob_1_pub = nh_coord.advertise<std_msgs::String>("/command_rob_1", 1000);
    command_rob_2_pub = nh_coord.advertise<std_msgs::String>("/command_rob_2", 1000);
    send_update_pub = nh_coord.advertise<std_msgs::String>("/coord_to_gui", 1000);

    rob_1_sub = nh_coord.subscribe("/rob1_to_coord", 1000, &Coordinator::rob1_callback, this);
    rob_2_sub = nh_coord.subscribe("/rob2_to_coord", 1000, &Coordinator::rob2_callback, this);
    gui_msgs_sub = nh_coord.subscribe("/gui_to_coord", 1000, &Coordinator::gui_callback, this);
}

void Coordinator::initialize_robots()
{
    // ros::WallDuration sleep_t(5);

    // while((command_rob_1_pub.getNumSubscribers()<1)&&(command_rob_2_pub.getNumSubscribers()<1)){
    //     sleep_t.sleep();
    // }
}

void Coordinator::rob1_callback(const std_msgs::String& str)
{
    if(str.data.compare("robot_1_intialization_complete") == 0)
    {
        std::cout<<"###################################### \n";
        std::cout<<"Robot 1 initialized \n";
        std::cout<<"###################################### \n";
    }
}

void Coordinator::rob2_callback(const std_msgs::String& str)
{
    if(str.data.compare("robot_2_intialization_complete") == 0)
    {
        std::cout<<"###################################### \n";
        std::cout<<"Robot 2 initialized \n";
        std::cout<<"###################################### \n";
        send_update_to_gui("All robots initialized");
    }
}

void Coordinator::gui_callback(const std_msgs::String& str)
{
    if(str.data.compare("initialize_robots") == 0)
    {
        std::cout<<"###################################### \n";
        std::cout<<"Initializing Robots \n";
        std::cout<<"###################################### \n";
        std_msgs::String all_rob_msg;
        all_rob_msg.data = "start_robot_initialization";
        command_rob_1_pub.publish(all_rob_msg);
        command_rob_2_pub.publish(all_rob_msg);
    }

}

void Coordinator::send_update_to_gui(std::string str_msg)
{
    msg_to_gui.data = str_msg;
    send_update_pub.publish(msg_to_gui);
    msg_to_gui.data.clear();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Coordinator");

    Coordinator CoordObj;

    while(ros::ok())
    {
        ros::Rate loop_rate(100);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}