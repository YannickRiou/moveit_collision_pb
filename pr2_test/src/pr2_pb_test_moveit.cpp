#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/fixed_state.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include<string>

#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

using namespace moveit::task_constructor;

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of panda_link8. |br|
  // From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
  // of the cube). |br|
  // Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
  // extra padding)
  grasps[0].grasp_pose.header.frame_id = "obj_230";
  tf2::Quaternion orientation;
  orientation.setRPY(1.57, 1.57, 0);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);

  grasps[0].grasp_pose.pose.position.x = 0.0;
  grasps[0].grasp_pose.pose.position.y = 0.0;
  grasps[0].grasp_pose.pose.position.z = 0.20;

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++  
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "base_footprint";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.z = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.01;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "base_footprint";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

    grasps[0].pre_grasp_posture.joint_names.resize(6);
    grasps[0].pre_grasp_posture.joint_names[0] = "l_gripper_joint";
    grasps[0].pre_grasp_posture.joint_names[1] = "l_gripper_motor_screw_joint";
    grasps[0].pre_grasp_posture.joint_names[2] = "l_gripper_l_finger_joint";
    grasps[0].pre_grasp_posture.joint_names[3] = "l_gripper_r_finger_joint";
    grasps[0].pre_grasp_posture.joint_names[4] = "l_gripper_r_finger_tip_joint";
    grasps[0].pre_grasp_posture.joint_names[5] = "l_gripper_l_finger_tip_joint";

    /* Set them as open, wide enough for the object to fit. */
    grasps[0].pre_grasp_posture.points.resize(1);
    grasps[0].pre_grasp_posture.points[0].positions.resize(6);
    grasps[0].pre_grasp_posture.points[0].positions[0] = 0.088;
    grasps[0].pre_grasp_posture.points[0].positions[1] = 1;
    grasps[0].pre_grasp_posture.points[0].positions[2] = 0.477;
    grasps[0].pre_grasp_posture.points[0].positions[3] = 0.477;
    grasps[0].pre_grasp_posture.points[0].positions[4] = 0.477;
    grasps[0].pre_grasp_posture.points[0].positions[5] = 0.477;
    grasps[0].pre_grasp_posture.points[0].time_from_start = ros::Duration(5);

  	grasps[0].grasp_posture.joint_names.resize(6);
    grasps[0].grasp_posture.joint_names[0] = "l_gripper_joint";
    grasps[0].grasp_posture.joint_names[1] = "l_gripper_motor_screw_joint";
    grasps[0].grasp_posture.joint_names[2] = "l_gripper_l_finger_joint";
    grasps[0].grasp_posture.joint_names[3] = "l_gripper_r_finger_joint";
    grasps[0].grasp_posture.joint_names[4] = "l_gripper_r_finger_tip_joint";
    grasps[0].grasp_posture.joint_names[5] = "l_gripper_l_finger_tip_joint";

    grasps[0].grasp_posture.points.resize(1);
    grasps[0].grasp_posture.points[0].positions.resize(6);
    grasps[0].grasp_posture.points[0].positions[0] = 0.00;
    grasps[0].grasp_posture.points[0].positions[1] = 0.00;
    grasps[0].grasp_posture.points[0].positions[2] = 0.002;
    grasps[0].grasp_posture.points[0].positions[3] = 0.002;
    grasps[0].grasp_posture.points[0].positions[4] = 0.002;
    grasps[0].grasp_posture.points[0].positions[5] = 0.002;
    grasps[0].grasp_posture.points[0].time_from_start = ros::Duration(5);

  // BEGIN_SUB_TUTORIAL pick3
  // Set support surface as table1.
  move_group.setSupportSurfaceName("tableLaas");
  // Call pick to pick up the object using the grasps given
  move_group.pick("obj_230", grasps);
  // END_SUB_TUTORIAL
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pr2_pb");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh("~");

	char ch;

	const std::string RIGHT_ARM_PLANNING_GROUP = "right_arm";
	const std::string LEFT_ARM_PLANNING_GROUP = "left_arm";

	const std::string RIGHT_GRIPPER_PLANNING_GROUP = "right_gripper";
	const std::string LEFT_GRIPPER_PLANNING_GROUP = "left_gripper";

	const robot_state::JointModelGroup* right_arm_joint_model_group;
    const robot_state::JointModelGroup* left_arm_joint_model_group;

	const robot_state::JointModelGroup* right_gripper_joint_model_group;
    const robot_state::JointModelGroup* left_gripper_joint_model_group;

    moveit::planning_interface::MoveGroupInterface right_arm_move_group(RIGHT_ARM_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface left_arm_move_group(LEFT_ARM_PLANNING_GROUP);

	moveit::planning_interface::MoveGroupInterface right_gripper_move_group(RIGHT_GRIPPER_PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface left_gripper_move_group(LEFT_GRIPPER_PLANNING_GROUP);

	right_arm_joint_model_group =
		right_arm_move_group.getCurrentState()->getJointModelGroup(RIGHT_ARM_PLANNING_GROUP);
	left_arm_joint_model_group =
		left_arm_move_group.getCurrentState()->getJointModelGroup(LEFT_ARM_PLANNING_GROUP);

	right_gripper_joint_model_group =
		right_gripper_move_group.getCurrentState()->getJointModelGroup(RIGHT_GRIPPER_PLANNING_GROUP);
	left_arm_joint_model_group =
		left_gripper_move_group.getCurrentState()->getJointModelGroup(LEFT_GRIPPER_PLANNING_GROUP);

	// Set Arm to initial pose
	right_arm_move_group.setNamedTarget("RIGHT_ARM_INITIAL_POSE");
	left_arm_move_group.setNamedTarget("LEFT_ARM_INITIAL_POSE");

	// Move the arms to initial pose (away from robot vision)
	right_arm_move_group.move();
	left_arm_move_group.move();

	left_gripper_move_group.setNamedTarget("left_open");
	left_gripper_move_group.move();

	left_arm_move_group.setNamedTarget("start");
	left_arm_move_group.move();

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	//spawnObject(planning_scene_interface);

	std::cout << "waiting for any key + <enter>\n";
	std::cin >> ch;

	moveit_msgs::AttachedCollisionObject ao;
	ao.link_name = "l_gripper_tool_frame";
	ao.object.id="obj_230";
	ao.object.operation = moveit_msgs::CollisionObject::ADD;

	//left_arm_move_group.attachObject("obj_230","l_gripper_tool_frame");
	planning_scene_interface.applyAttachedCollisionObject(ao);

	 std::vector<std::string> ids;
	 ids.push_back("obj_230");

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface2;


	ROS_ERROR_STREAM("OBJ IS ATTACHED TO :" << planning_scene_interface2.getAttachedObjects(ids).at("obj_230").link_name);

	//pick(left_arm_move_group);


	moveit::planning_interface::MoveGroupInterface::Plan my_plan;


	std::cout << "waiting for any key + <enter>\n";
	std::cin >> ch;

	left_arm_move_group.setNamedTarget("end");


	// Plan doesn't show the collision object moving with the gripper
	bool success = (left_arm_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


	std::cout << "waiting for any key + <enter>\n";
	std::cin >> ch;
	
	left_arm_move_group.execute(my_plan);

	//left_arm_move_group.attachObject("obj_230","l_gripper_tool_frame");
	planning_scene_interface.applyAttachedCollisionObject(ao);




	return 0;
}
