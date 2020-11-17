#include "ur5_planning/ur10_robot_move_group.hpp"


Move_Group_Robot_3::Move_Group_Robot_3()
{
    std::cout<<"Loading Move Group for Ur10 Robot  \n";
    moveit::planning_interface::MoveGroupInterface::Options Opt{PLANNING_GROUP, ROBOT_DESCRIPTION, node_handle_rob3};
    
    ur10_robot_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(Opt);
    joint_model_group = ur10_robot_group_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    joint_names = joint_model_group->getVariableNames();
    link_names = joint_model_group->getLinkModelNames();

    // ur5_robot1_group_ptr->setGoalJointTolerance(0.1);
    // ur5_robot1_group_ptr->setGoalPositionTolerance(0.1);
    // ur5_robot1_group_ptr->setGoalOrientationTolerance(0.1);

    ur10_robot_group_ptr->setGoalTolerance(0.01);
    // ur5_robot1_group_ptr->setPlanningTime(5.0);
    ur10_robot_group_ptr->allowReplanning(true);

    for(auto i:joint_names)
    {
      std::cout<<"Joint Name: "<<i<<std::endl;
    }

    //ROS publishers and subscribers
    receive_data_from_coord_sub = node_handle_rob3.subscribe("/command_rob_3", 1000, &Move_Group_Robot_3::perform_actions,this);
    send_update_pub = node_handle_rob3.advertise<std_msgs::String>("/rob3_to_coord", 1000);
    planning_scene_diff_publisher = node_handle_rob3.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    // workpiece_pos_sub = node_handle_rob1.subscribe("/initial_workpiece_pos", 1000, &Move_Group_Robot_1::add_workpieces, this);

    workpiece_position[0]  = 0.3;
    workpiece_position[1]  = -0.7;
    workpiece_position[2]  = 0.62;
    world_to_part_T = hom_T(workpiece_position, rotation);

}

void Move_Group_Robot_3::send_update(std::string msg)
{
  update_msg.data = msg;
  send_update_pub.publish(update_msg);
  update_msg.data.clear();
}

std::string Move_Group_Robot_3::intToString (int a) {
    std::stringstream ss;
    ss << a;
    return ss.str();
  }

void Move_Group_Robot_3::perform_actions(const std_msgs::String& msg)
{
  if(msg.data.compare("start_robot_initialization") == 0)
  {
    std::vector<double> target_joint_angles = {-0.0349066, -1.69297,2.0944 ,-1.98968,-1.5708 ,1.29154};
    move_to_configuration(target_joint_angles);
    moveit_msgs::PlanningScene planning_scene;

    send_update("robot_3_intialization_complete");

    //Adding other collision objects to world after initialization
    // add_robot_table();
    // add_workpiece_table();
    // add_workpiece_table_2();
  }
  if(msg.data.compare("execute_milling")==  0)
  {
      perform_milling();
  }
}

Eigen::Matrix4d Move_Group_Robot_3::hom_T(Eigen::Vector3d t, Eigen::Matrix3d r)
{
    Eigen::Matrix4d T;
    T.block(0,3,3,1) << t;
    T.block(0,0,3,3) << r;
    T.block(3,0,1,4) << 0,0,0,1;
    return T;
}

Eigen::MatrixXd Move_Group_Robot_3::file_read_mat(std::string file_name)
{
    std::string tok;
    std::vector <std::vector <double> > vec;
    std::string line;
    std::string item;
    std::ifstream input_file;
    std::ifstream infile(file_name);
    if (infile.good())
    {
        input_file.open(file_name.c_str(),std::ios::in);
        while(!input_file.eof())
        {
            getline(input_file, line);
            if (!line.empty())
            {
                std::istringstream in(line);
                std::vector <double> row_vec;
                while (getline(in, item, ',')) 
                {
                    row_vec.push_back(atof(item.c_str()));
                }
                vec.push_back(row_vec);
            }
        }
        input_file.close();
    }
    else
    {
        std::cerr << "File/Path does not exist" << std::endl;
    }
    if (vec.size()!=0)
    {   
        Eigen::MatrixXd mat(vec.size(), vec[0].size());
        for (int i = 0; i < vec.size(); ++i)
        {
            mat.row(i) = Eigen::VectorXd::Map(&vec[i][0], vec[0].size());
        }
        return mat;
    }
    else
    {
        Eigen::MatrixXd mat(0,0);
        return mat;
    }
} 

Eigen::MatrixXd Move_Group_Robot_3::apply_transformation(Eigen::MatrixXd data, Eigen::Matrix4d T_mat)
{
    //!NOTE: Homogeneous Tranformation Matrix (4x4)

    //! putting data in [x, y, z, 1]' format
    Eigen::MatrixXd data_with_fourth_row(data.cols()+1,data.rows());
    Eigen::VectorXd ones_vec = Eigen::VectorXd::Constant(data.rows(),1);
    data_with_fourth_row.block(0,0,data.cols(),data.rows()) = data.transpose();
    data_with_fourth_row.block(data.cols(),0,1,data.rows()) = ones_vec.transpose();
    Eigen::MatrixXd transformed_data = T_mat*data_with_fourth_row;
    Eigen::MatrixXd transformed_data_mat(transformed_data.rows()-1,transformed_data.cols());
    transformed_data_mat = transformed_data.block(0,0,transformed_data.rows()-1,transformed_data.cols());
    return transformed_data_mat.transpose();
}

void Move_Group_Robot_3::move_to_pose(geometry_msgs::Pose target_p)
{
  moveit_msgs::PlanningScene planning_scene;
  ur10_robot_group_ptr->setStartStateToCurrentState();
  ur10_robot_group_ptr->setMaxVelocityScalingFactor(0.01);
  geometry_msgs::Pose start_pose;
  start_pose = ur10_robot_group_ptr->getCurrentPose().pose;
  std::cout<<"Start Pose: "<<start_pose;

  ur10_robot_group_ptr->setPoseTarget(target_p);
  std::cout<<"Target Pose: "<<target_p;

  bool success = (ur10_robot_group_ptr->plan(ur10_robot_goal_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ur10_robot_group_ptr->move();
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);

}

void Move_Group_Robot_3::perform_milling()
{
    Eigen::MatrixXd waypoints = file_read_mat("/home/jaineel/Desktop/AME_547_Project/Jaineel/Jaineel_WS_New/src/milling_path_visualizer/data/waypoints.csv");
    Eigen::MatrixXd transformed_pts = apply_transformation(waypoints, world_to_part_T);

    //Go to mill position
    geometry_msgs::Pose target_pose; 
    target_pose = ur10_robot_group_ptr->getCurrentPose().pose;
    target_pose.position.x = transformed_pts(0,0);
    target_pose.position.y = transformed_pts(0,1);
    target_pose.position.z = transformed_pts(0,2);
    std::cout<<"Target Pose: "<<target_pose;
    move_to_pose(target_pose);
    ur10_robot_group_ptr->setStartStateToCurrentState();


    //execute milling
    for(size_t i =1; i<transformed_pts.rows(); i++)
    {
      geometry_msgs::Pose target_pose; 
      target_pose = ur10_robot_group_ptr->getCurrentPose().pose;
      target_pose.position.x = transformed_pts(i,0);
      target_pose.position.y = transformed_pts(i,1);
      target_pose.position.z = transformed_pts(i,2);
      std::cout<<"Target Pose: "<<target_pose;
      move_to_pose(target_pose);
      ur10_robot_group_ptr->setStartStateToCurrentState();
    }

    //retract
    geometry_msgs::Pose target_pose_2; 
    target_pose = ur10_robot_group_ptr->getCurrentPose().pose;
    target_pose.position.z = target_pose.position.z + 0.1;
    std::cout<<"Target Pose: "<<target_pose;
    move_to_pose(target_pose);
    ur10_robot_group_ptr->setStartStateToCurrentState();

    //go to initial state
    std::vector<double> target_joint_angles = {-0.0349066, -1.69297,2.0944 ,-1.98968,-1.5708 ,1.29154};
    move_to_configuration(target_joint_angles);

    send_update("milling_complete");

}

void Move_Group_Robot_3::move_to_configuration(std::vector<double>& joint_angles)
{
  moveit_msgs::PlanningScene planning_scene;
  ur10_robot_group_ptr->setStartStateToCurrentState();

  ur10_robot_group_ptr->setJointValueTarget(joint_angles);

  bool success = (ur10_robot_group_ptr->plan(ur10_robot_goal_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ur10_robot_group_ptr->move();
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);

  std::cout<<ur10_robot_goal_plan.trajectory_.joint_trajectory.points[0]<<std::endl;
  std::cout<<ur10_robot_goal_plan.trajectory_.joint_trajectory.points.back()<<std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur10_robot_planning");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  Move_Group_Robot_3 rob3_obj;
        
  //rob1_obj.add_robot_table();

  std::cout<<"Robot Table added as a Collision Object"<<std::endl;

  

  ros::waitForShutdown();
  return 0;
}