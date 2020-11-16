#include "process_visualizer/workpiece_handler.hpp"

bool robot_1_attached = false;
bool robot_2_attached = false;
Workpiece_Object::Workpiece_Object()
{
    gazebo_model_state_pub = nh_workpiece.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    joint_states_robot1_sub = nh_workpiece.subscribe("/gazebo/link_states", 1, &Workpiece_Object::rob1_jointstates_callback, this);
    joint_states_robot2_sub = nh_workpiece.subscribe("ur5_robot2/joint_states", 1, &Workpiece_Object::rob2_jointstates_callback, this);
    initial_workpiece_pos_sub = nh_workpiece.subscribe("/initial_workpiece_pos", 100, &Workpiece_Object::load_workpiece_in_gazebo, this);
    spawnClient = nh_workpiece.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    attached_to_rob_1 = nh_workpiece.subscribe("/attached_to_rob_1", 1000, &Workpiece_Object::set_rob1_flag, this);
    deleteModelClient = nh_workpiece.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");


    robot_model_loader::RobotModelLoader robot_model_loader("ur5_robot1/robot_description");
    kinematic_model_ur5_1 = robot_model_loader.getModel();
    kinematic_state_ur5_1 = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model_ur5_1));

    joint_model_group = kinematic_model_ur5_1->getJointModelGroup("manipulator");
}

int Workpiece_Object::getIndex(std::vector<std::string> v, std::string K)
{
    std::vector<std::string>::iterator it = std::find(v.begin(), v.end(), K);
    std::cout<<"link_name found \n";
    // std::cout<<"Iterator val: "<<it<<std::endl;
    // If element was found
    if (it != v.end()) 
    {
     
        // calculating the index
        // of K
        int index = std::distance(v.begin(), it);
        std::cout<<"Index inside function = "<<index<<std::endl;
        return index;
    }
    else {
        // If the element is not
        // present in the vector
    }
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

    std::string wp_index = intToString(num_of_workpieces_rob1);
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
    num_of_workpieces_rob1 = num_of_workpieces_rob1+1;
}

void Workpiece_Object::set_rob1_flag(const std_msgs::String& msg)
{
    if(msg.data.compare("attached_rob1")==0)
    {
        robot_1_attached = true;
    }
    else if(msg.data.compare("detached_rob1")==0)
    {
        robot_1_attached = false;
        std::string wp_index = intToString(current_wp_rob1);
        std::string model_name = "workpiece_" + wp_index;
        geometry_msgs::Pose p;
        p.position.x = 1.1;
        p.position.y = -0.08;
        p.position.z = 0.525;
        p.orientation.w = 1.0;
        p.orientation.x = 0.0;
        p.orientation.y = 0.0;
        p.orientation.z = 0.0;
        spawn_model(model_name, p);
        current_wp_rob1 ++;
    }
    else if(msg.data.compare("attached_rob2")==0)
    {
        robot_2_attached = true;
    }
    else if(msg.data.compare("detached_rob2")==0)
    {
        robot_2_attached = false;
        std::string wp_index = intToString(current_wp_rob2);
        std::string model_name = "workpiece_" + wp_index;
        geometry_msgs::Pose p;
        p.position.x = bin_1_pos[0][0];
        p.position.y = bin_1_pos[0][1];
        p.position.z = bin_1_pos[0][2];
        p.orientation.w = 1.0;
        p.orientation.x = 0.0;
        p.orientation.y = 0.0;
        p.orientation.z = 0.0;
        spawn_model(model_name, p);
        current_wp_rob2 ++;
    }
}
void Workpiece_Object::remove(std::string model_name)
{
    gazebo_msgs::DeleteModel deleteModel;
    deleteModel.request.model_name = model_name;
    deleteModelClient.call(deleteModel);    
}

void Workpiece_Object::spawn_model(std::string model_name, geometry_msgs::Pose p)
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
}

void Workpiece_Object::rob1_jointstates_callback(const gazebo_msgs::LinkStates& state)
{
    if(robot_1_attached == true)
    {
        // std::vector<double> joint_states;
        // for (size_t i = 0; i < joint_states_current.position.size() - 2; ++i)
        // {
        //     joint_states.push_back(joint_states_current.position[i + 2]);
        // }
        // kinematic_state_ur5_1->setJointGroupPositions(joint_model_group, joint_states);
        // const Eigen::Affine3d &end_effector_state = kinematic_state_ur5_1->getGlobalLinkTransform("ur5_robot1_tf/tool0");

        // double end_effector_z_offset = 0.0001;
        // Eigen::Affine3d tmp_transform(Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, end_effector_z_offset)));

        // Eigen::Affine3d newState = end_effector_state * tmp_transform;

        // tf::StampedTransform transform;
        // try{
        //     listener.lookupTransform("/ur5_robot1_tf/ee_link", "/ur5_robot1_tf/world",ros::Time(0),transform);
        // }
        // catch(tf::TransformException &ex){
        //     ROS_ERROR("%s", ex.what());
        //     ros::Duration(1.0).sleep();
        // }

        std::string wp_index = intToString(current_wp_rob1);
        std::string model_name = "workpiece_" + wp_index;
        remove(model_name);

        // model_state.model_name = model_name;
        // model_state.pose = pose;




        // std::string link_name = "ur5_robot1::wrist_3_link";
        // std::cout<<link_name<<std::endl;
        // geometry_msgs::Pose pose;

        // std::vector<std::string> link_names;

        // for(auto i:state.name)
        // {
        //     link_names.push_back(i);
        //     std::cout<<i<<std::endl;
        // }
        // int id = getIndex(link_names, link_name);
        // std::cout<<id<<std::endl;
        // pose = state.pose[id];

        // std::cout<<pose<<std::endl;


        // model_state.reference_frame = "world";

        // gazebo_model_state_pub.publish(model_state);
        ros::Duration D(2);
        D.sleep();
    }
}


void Workpiece_Object::rob2_jointstates_callback(const sensor_msgs::JointState &joint_states_current)
{
    if(robot_2_attached == true)
    {
        // std::vector<double> joint_states;
        // for (size_t i = 0; i < joint_states_current.position.size() - 2; ++i)
        // {
        //     joint_states.push_back(joint_states_current.position[i + 2]);
        // }
        // kinematic_state_ur5_1->setJointGroupPositions(joint_model_group, joint_states);
        // const Eigen::Affine3d &end_effector_state = kinematic_state_ur5_1->getGlobalLinkTransform("ur5_robot1_tf/tool0");

        // double end_effector_z_offset = 0.0001;
        // Eigen::Affine3d tmp_transform(Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, end_effector_z_offset)));

        // Eigen::Affine3d newState = end_effector_state * tmp_transform;

        // tf::StampedTransform transform;
        // try{
        //     listener.lookupTransform("/ur5_robot1_tf/ee_link", "/ur5_robot1_tf/world",ros::Time(0),transform);
        // }
        // catch(tf::TransformException &ex){
        //     ROS_ERROR("%s", ex.what());
        //     ros::Duration(1.0).sleep();
        // }

        std::string wp_index = intToString(current_wp_rob2);
        std::string model_name = "workpiece_" + wp_index;
        remove(model_name);

        // model_state.model_name = model_name;
        // model_state.pose = pose;




        // std::string link_name = "ur5_robot1::wrist_3_link";
        // std::cout<<link_name<<std::endl;
        // geometry_msgs::Pose pose;

        // std::vector<std::string> link_names;

        // for(auto i:state.name)
        // {
        //     link_names.push_back(i);
        //     std::cout<<i<<std::endl;
        // }
        // int id = getIndex(link_names, link_name);
        // std::cout<<id<<std::endl;
        // pose = state.pose[id];

        // std::cout<<pose<<std::endl;


        // model_state.reference_frame = "world";

        // gazebo_model_state_pub.publish(model_state);
        ros::Duration D(2);
        D.sleep();
    }
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