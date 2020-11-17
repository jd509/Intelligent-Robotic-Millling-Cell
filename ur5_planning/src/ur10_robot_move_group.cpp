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
}


void Move_Group_Robot_3::move_to_pose(geometry_msgs::Pose target_p)
{
  moveit_msgs::PlanningScene planning_scene;
  ur10_robot_group_ptr->setStartStateToCurrentState();
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