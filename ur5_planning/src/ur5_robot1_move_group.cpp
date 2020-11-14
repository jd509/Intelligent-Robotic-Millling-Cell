#include "ur5_planning/ur5_robot1_move_group.hpp"


Move_Group_Robot_1::Move_Group_Robot_1()
{
    std::cout<<"Loading Move Group for Ur5 Robot 1 \n";
    moveit::planning_interface::MoveGroupInterface::Options Opt{PLANNING_GROUP, ROBOT_DESCRIPTION, node_handle_rob1};
    
    ur5_robot1_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(Opt);
    joint_model_group = ur5_robot1_group_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    joint_names = joint_model_group->getVariableNames();
    link_names = joint_model_group->getLinkModelNames();

    // ur5_robot1_group_ptr->setGoalJointTolerance(0.1);
    // ur5_robot1_group_ptr->setGoalPositionTolerance(0.1);
    // ur5_robot1_group_ptr->setGoalOrientationTolerance(0.1);

    ur5_robot1_group_ptr->setGoalTolerance(0.1);
    ur5_robot1_group_ptr->setPlanningTime(7.0);

    for(auto i:joint_names)
    {
      std::cout<<"Joint Name: "<<i<<std::endl;
    }

    //ROS publishers and subscribers
    receive_data_from_coord_sub = node_handle_rob1.subscribe("/command_rob_1", 1000, &Move_Group_Robot_1::perform_actions,this);
    send_update_pub = node_handle_rob1.advertise<std_msgs::String>("/rob1_to_coord", 1000);
    planning_scene_diff_publisher = node_handle_rob1.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    workpiece_pos_sub = node_handle_rob1.subscribe("/initial_workpiece_pos", 1000, &Move_Group_Robot_1::add_workpieces, this);
}

void Move_Group_Robot_1::send_update(std::string msg)
{
  update_msg.data = msg;
  send_update_pub.publish(update_msg);
  update_msg.data.clear();
}

std::string Move_Group_Robot_1::intToString (int a) {
    std::stringstream ss;
    ss << a;
    return ss.str();
  }

void Move_Group_Robot_1::perform_actions(const std_msgs::String& msg)
{
  if(msg.data.compare("start_robot_initialization") == 0)
  {
    std::vector<double> target_joint_angles = {0.8157, -1.7009, -1.9787, -2.5341, 1.6315, -0.1041};
    move_to_configuration(target_joint_angles);
    send_update("robot_1_intialization_complete");

    //Adding other collision objects to world after initialization
    add_workpiece_table();
    add_workpiece_table_2();
  }
  else if(msg.data.compare("start_pickup_rob1")==0)
  {
    pick();
  }
}

void Move_Group_Robot_1::add_workpieces(const geometry_msgs::Pose &p)
{
  //Intializing Collision Object for workpiece table
  moveit_msgs::CollisionObject workpiece;
  workpiece.header.frame_id = ur5_robot1_group_ptr->getPlanningFrame();
  std::string workpiece_id_generate;
  workpiece_id_generate = "workpiece_" + intToString(workpiece_id);
  workpiece.id = workpiece_id_generate;

  //Creating a mesh from stl of workpiece
  std::string mesh_file_for_table = "package://process_visualizer/resources/workpiece.stl";
  shapes::Mesh *table_mesh = shapes::createMeshFromResource(mesh_file_for_table);

  //Defining shape message
  shape_msgs::Mesh table_mesh_;
  shapes::ShapeMsg table_mesh_msg;
  shapes::constructMsgFromShape(table_mesh, table_mesh_msg);
  table_mesh_ = boost::get<shape_msgs::Mesh>(table_mesh_msg);

  //Position of workpiece
  geometry_msgs::Pose table_pose;
  table_pose.orientation.w = p.orientation.w;
  table_pose.orientation.x = p.orientation.x;
  table_pose.orientation.y = p.orientation.y;
  table_pose.orientation.z = p.orientation.z;

  table_pose.position.x = p.position.x;
  table_pose.position.y = p.position.y;
  table_pose.position.z = p.position.z;

  //Adding mesh to collision object
  workpiece.meshes.push_back(table_mesh_);
  workpiece.mesh_poses.push_back(table_pose);
  wp_pos.emplace_back(table_pose);
  workpiece.operation = workpiece.ADD;

  add_collision_obj_to_world(workpiece, workpiece_id_generate);
  workpiece_id++;
}

void Move_Group_Robot_1::add_collision_obj_to_world(moveit_msgs::CollisionObject object, std::string object_name)
{
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(object);

  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);

  std::cout<<"Added object: "<<object_name<<" into the world frame \n";
}


void Move_Group_Robot_1::add_robot_table()
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;

  //Intializing Collision Object for robot table
  moveit_msgs::CollisionObject robot_table;
  robot_table.header.frame_id = ur5_robot1_group_ptr->getPlanningFrame();
  robot_table.id = "robot_table_1";

  //Creating a mesh from stl of table
  std::string mesh_file_for_table = "package://process_visualizer/resources/table.stl";
  shapes::Mesh *table_mesh = shapes::createMeshFromResource(mesh_file_for_table);

  //Defining shape message
  shape_msgs::Mesh table_mesh_;
  shapes::ShapeMsg table_mesh_msg;
  shapes::constructMsgFromShape(table_mesh, table_mesh_msg);
  table_mesh_ = boost::get<shape_msgs::Mesh>(table_mesh_msg);

  //Position of table
  geometry_msgs::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.orientation.x = 0.0;
  table_pose.orientation.y = 0.0;
  table_pose.orientation.z = 0.0;

  table_pose.position.x = 0.87;
  table_pose.position.y = 0.57;
  table_pose.position.z = 0.005;

  //Adding mesh to collision object
  robot_table.meshes.push_back(table_mesh_);
  robot_table.mesh_poses.push_back(table_pose);
  robot_table.operation = robot_table.ADD;

  collision_objects.push_back(robot_table);

  planning_scene_interface.addCollisionObjects(collision_objects);
  collision_objects.clear();
}

void Move_Group_Robot_1::add_workpiece_table()
{
  //Intializing Collision Object for workpiece table
  moveit_msgs::CollisionObject workpiece_table;
  workpiece_table.header.frame_id = ur5_robot1_group_ptr->getPlanningFrame();
  workpiece_table.id = "workpiece_table_1";

  //Creating a mesh from stl of table
  std::string mesh_file_for_table = "package://process_visualizer/resources/table_workpiece.stl";
  shapes::Mesh *table_mesh = shapes::createMeshFromResource(mesh_file_for_table);

  //Defining shape message
  shape_msgs::Mesh table_mesh_;
  shapes::ShapeMsg table_mesh_msg;
  shapes::constructMsgFromShape(table_mesh, table_mesh_msg);
  table_mesh_ = boost::get<shape_msgs::Mesh>(table_mesh_msg);

  //Position of table
  geometry_msgs::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.orientation.x = 0.0;
  table_pose.orientation.y = 0.0;
  table_pose.orientation.z = 0.0;

  table_pose.position.x = 1.28;
  table_pose.position.y = 0.53;
  table_pose.position.z = 0.005;

  //Adding mesh to collision object
  workpiece_table.meshes.push_back(table_mesh_);
  workpiece_table.mesh_poses.push_back(table_pose);
  workpiece_table.operation = workpiece_table.ADD;

  add_collision_obj_to_world(workpiece_table, "work_piece_table_for_pickup");
}

void Move_Group_Robot_1::add_workpiece_table_2()
{
  //Intializing Collision Object for workpiece table
  moveit_msgs::CollisionObject workpiece_table;
  workpiece_table.header.frame_id = ur5_robot1_group_ptr->getPlanningFrame();
  workpiece_table.id = "workpiece_table_2";

  //Creating a mesh from stl of table
  std::string mesh_file_for_table = "package://process_visualizer/resources/table_workpiece.stl";
  shapes::Mesh *table_mesh = shapes::createMeshFromResource(mesh_file_for_table);

  //Defining shape message
  shape_msgs::Mesh table_mesh_;
  shapes::ShapeMsg table_mesh_msg;
  shapes::constructMsgFromShape(table_mesh, table_mesh_msg);
  table_mesh_ = boost::get<shape_msgs::Mesh>(table_mesh_msg);
 
  //Position of table
  geometry_msgs::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.orientation.x = 0.0;
  table_pose.orientation.y = 0.0;
  table_pose.orientation.z = 0.0;

  table_pose.position.x = 0.8;
  table_pose.position.y = -0.25;
  table_pose.position.z = 0.005;

  //Adding mesh to collision object
  workpiece_table.meshes.push_back(table_mesh_);
  workpiece_table.mesh_poses.push_back(table_pose);
  workpiece_table.operation = workpiece_table.ADD;

  add_collision_obj_to_world(workpiece_table, "work_piece_table_for_placing");
}

void Move_Group_Robot_1::pick()
{
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  //setting grasp pose
  grasps[0].grasp_pose.header.frame_id = ur5_robot1_group_ptr->getPlanningFrame();
  tf2::Quaternion orientation;
  orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = wp_pos[current_wp].position.x;
  grasps[0].grasp_pose.pose.position.y = wp_pos[current_wp].position.y;
  grasps[0].grasp_pose.pose.position.z = wp_pos[current_wp].position.z;

  //setting pre grasp pose
  grasps[0].pre_grasp_approach.direction.header.frame_id = ur5_robot1_group_ptr->getPlanningFrame();
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  //setting post grasp retreat
  grasps[0].post_grasp_retreat.direction.header.frame_id = ur5_robot1_group_ptr->getPlanningFrame();
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;  

  //picking up object
  ur5_robot1_group_ptr->setSupportSurfaceName("workpiece_table_1");
  
  std::string object_name = "workpiece_" + intToString(current_wp);
  ur5_robot1_group_ptr->pick(object_name, grasps);
}

void Move_Group_Robot_1::move_to_configuration(std::vector<double>& joint_angles)
{
  ur5_robot1_group_ptr->setStartStateToCurrentState();

  ur5_robot1_group_ptr->setJointValueTarget(joint_angles);

  bool success = (ur5_robot1_group_ptr->plan(ur5_robot1_goal_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ur5_robot1_group_ptr->move();
  std::cout<<ur5_robot1_goal_plan.trajectory_.joint_trajectory.points[0]<<std::endl;
  std::cout<<ur5_robot1_goal_plan.trajectory_.joint_trajectory.points.back()<<std::endl;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur5_robot1_planning");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  Move_Group_Robot_1 rob1_obj;
        
  rob1_obj.add_robot_table();

  std::cout<<"Robot Table added as a Collision Object"<<std::endl;

  

  ros::waitForShutdown();
  return 0;
}