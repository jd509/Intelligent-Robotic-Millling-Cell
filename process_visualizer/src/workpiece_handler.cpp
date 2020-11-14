#include "process_visualizer/workpiece_handler.hpp"


Workpiece_Object::Workpiece_Object()
{
    gazebo_model_state_pub = nh_workpiece.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    joint_states_robot1_sub = nh_workpiece.subscribe("ur5_robot1/joint_states", 1, &Workpiece_Object::rob1_jointstates_callback, this);
    joint_states_robot2_sub = nh_workpiece.subscribe("ur5_robot2/joint_states", 1, &Workpiece_Object::rob2_jointstates_callback, this);
    initial_workpiece_pos_sub = nh_workpiece.subscribe("/initial_workpiece_pos", 100, &Workpiece_Object::load_workpiece_in_gazebo, this);
    spawnClient = nh_workpiece.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
}

std::string Workpiece_Object::intToString (int a) {
    std::stringstream ss;
    ss << a;
    return ss.str();
  }

void Workpiece_Object::load_workpiece_in_gazebo(const geometry_msgs::Pose& p)
{
    gazebo_msgs::SpawnModel::Request spawn_model_req;
    gazebo_msgs::SpawnModel::Response spawn_model_resp;


    spawn_model_req.initial_pose.position.x = p.position.x;
    spawn_model_req.initial_pose.position.y = p.position.y;
    spawn_model_req.initial_pose.position.z = p.position.z;

    spawn_model_req.initial_pose.orientation.w = p.orientation.w;
    spawn_model_req.initial_pose.orientation.x = p.orientation.x;
    spawn_model_req.initial_pose.orientation.y = p.orientation.y;
    spawn_model_req.initial_pose.orientation.z = p.orientation.z;
    spawn_model_req.reference_frame = "world";

    std::string workpiece_file_path;
    bool get_workpiece_path;
    get_workpiece_path = nh_workpiece.getParam("/workpiece_urdf_path", workpiece_file_path);

    //workpiece referred to as 'wp' from here on
    std::ifstream wp_inXml(workpiece_file_path.c_str());
    std::stringstream wp_strStream;
    std::string wp_xmlStr;

    wp_strStream << wp_inXml.rdbuf();
    wp_xmlStr = wp_strStream.str();

    std::string wp_index = intToString(num_of_workpieces);
    std::string model_name;

    model_name = "workpiece_" + wp_index;
    spawn_model_req.model_name = model_name;
    spawn_model_req.robot_namespace = model_name;
    spawn_model_req.model_xml = wp_xmlStr;

    bool call_service = spawnClient.call(spawn_model_req, spawn_model_resp);
    if (call_service) {
        if (spawn_model_resp.success) {
            ROS_INFO_STREAM(model_name << " has been spawned");
        }
        else {
            ROS_INFO_STREAM(model_name << " spawn failed");
        }
    }
    else {
        ROS_INFO("fail in first call");
        ROS_ERROR("fail to connect with gazebo server");
    }
    num_of_workpieces = num_of_workpieces+1;
}

void Workpiece_Object::rob1_jointstates_callback(const sensor_msgs::JointState &joint_states_current)
{

}

void Workpiece_Object::rob2_jointstates_callback(const sensor_msgs::JointState &joint_states_current)
{

}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "workpiece_handler");
    
    Workpiece_Object wp_obj;

    while(ros::ok())
    {
        ros::Rate loop_rate(100);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}