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

#include<string>

#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

using namespace moveit::task_constructor;
void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi) {

	moveit_msgs::CollisionObject o;
	o.id= "object";
	o.header.frame_id= "base_footprint";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.65;
	o.primitive_poses[0].position.y = 0.18;
	o.primitive_poses[0].position.z = 0.86;
	o.primitive_poses[0].orientation.x =0.0;
	o.primitive_poses[0].orientation.y =0.0;
	o.primitive_poses[0].orientation.z =0.0;
	o.primitive_poses[0].orientation.w =1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
	o.primitives[0].dimensions.resize(3);
	o.primitives[0].dimensions[0]= 0.05;
	o.primitives[0].dimensions[1]= 0.05;
	o.primitives[0].dimensions[2]= 0.20;
	psi.applyCollisionObject(o);

	o.id= "obstacle";
	o.header.frame_id= "base_footprint";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.6;
	o.primitive_poses[0].position.y = -0.05;
	o.primitive_poses[0].position.z = 0.75+(0.30/2);
	o.primitive_poses[0].orientation.x =0.0;
	o.primitive_poses[0].orientation.y =0.0;
	o.primitive_poses[0].orientation.z =0.0;
	o.primitive_poses[0].orientation.w =1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
	o.primitives[0].dimensions.resize(3);
	o.primitives[0].dimensions[0]= 0.05;
	o.primitives[0].dimensions[1]= 0.05;
	o.primitives[0].dimensions[2]= 0.30;
	psi.applyCollisionObject(o);

	o.id= "obstacle2";
	o.header.frame_id= "base_footprint";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.6;
	o.primitive_poses[0].position.y = -0.35;
	o.primitive_poses[0].position.z = 0.75+(0.30/2);
	o.primitive_poses[0].orientation.x =0.0;
	o.primitive_poses[0].orientation.y =0.0;
	o.primitive_poses[0].orientation.z =0.0;
	o.primitive_poses[0].orientation.w =1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
	o.primitives[0].dimensions.resize(3);
	o.primitives[0].dimensions[0]= 0.05;
	o.primitives[0].dimensions[1]= 0.05;
	o.primitives[0].dimensions[2]= 0.30;
	psi.applyCollisionObject(o);


	o.id= "obstacle3";
	o.header.frame_id= "base_footprint";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.6;
	o.primitive_poses[0].position.y = -0.20;
	o.primitive_poses[0].position.z = 0.75+0.3;
	o.primitive_poses[0].orientation.x =0.0;
	o.primitive_poses[0].orientation.y =0.0;
	o.primitive_poses[0].orientation.z =0.0;
	o.primitive_poses[0].orientation.w =1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
	o.primitives[0].dimensions.resize(3);
	o.primitives[0].dimensions[0]= 0.05;
	o.primitives[0].dimensions[1]= 0.30;
	o.primitives[0].dimensions[2]= 0.05;
	psi.applyCollisionObject(o);


	moveit_msgs::CollisionObject table;
	table.id= "tableLaas";
	table.header.frame_id= "base_footprint";
	table.primitive_poses.resize(1);
	table.primitive_poses[0].position.x = 0.95;
	table.primitive_poses[0].position.y = 0.0;
	table.primitive_poses[0].position.z = 0.75/2;
	table.primitive_poses[0].orientation.x =0.0;
	table.primitive_poses[0].orientation.y =0.0;
	table.primitive_poses[0].orientation.z =0.0;
	table.primitive_poses[0].orientation.w =1.0;
	table.primitives.resize(1);
	table.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
	table.primitives[0].dimensions.resize(3);
	table.primitives[0].dimensions[0]= 0.85;
	table.primitives[0].dimensions[1]= 1.35;
	table.primitives[0].dimensions[2]= 0.75;
	psi.applyCollisionObject(table);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pr2_pb");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh("~");

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
	spawnObject(planning_scene_interface);

	moveit_msgs::CollisionObject o;
	o.id= "object";
	o.header.frame_id= "base_footprint";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.65;
	o.primitive_poses[0].position.y = 0.18;
	o.primitive_poses[0].position.z = 0.86;
	o.primitive_poses[0].orientation.x =0.0;
	o.primitive_poses[0].orientation.y =0.0;
	o.primitive_poses[0].orientation.z =0.0;
	o.primitive_poses[0].orientation.w =1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
	o.primitives[0].dimensions.resize(3);
	o.primitives[0].dimensions[0]= 0.05;
	o.primitives[0].dimensions[1]= 0.05;
	o.primitives[0].dimensions[2]= 0.20;

	moveit_msgs::AttachedCollisionObject ao;
	ao.link_name = "l_gripper_tool_frame";
	ao.object=o;

	planning_scene_interface.applyAttachedCollisionObject(ao);

	left_arm_move_group.setNamedTarget("end");

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	// Plan doesn't show the collision object moving with the gripper
	bool success = (left_arm_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	std::cout << "waiting for any key + <enter>\n";
	char ch;
	std::cin >> ch;
	
	left_arm_move_group.execute(my_plan);




	return 0;
}
