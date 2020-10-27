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
#include <moveit/task_constructor/solvers/joint_interpolation.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include<string>

#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

using namespace moveit::task_constructor;

void planTest2(Task &t) {

	t.loadRobotModel();

	auto cartesian = std::make_shared<solvers::CartesianPath>();
	cartesian->setProperty("jump_threshold", 0.0);

	auto gripper_planner = std::make_shared<solvers::JointInterpolationPlanner>();

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnect");
	pipeline->setProperty("longest_valid_segment_fraction", 0.00001);

	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	t.add(std::move(initial));

	{
		auto stage = std::make_unique<stages::MoveTo>("right home", pipeline);
		stage->setProperty("group", "right_arm");
		stage->setGoal("RIGHT_ARM_INITIAL_POSE");
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("Left home", pipeline);
		stage->setProperty("group", "left_arm");
		stage->setGoal("LEFT_ARM_INITIAL_POSE");
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("Left open", gripper_planner);
		stage->setProperty("group", "left_gripper");
		stage->setGoal("left_open");
		t.add(std::move(stage));
	}

		{
		auto stage = std::make_unique<stages::MoveTo>("Left start", pipeline);
		stage->setProperty("group", "left_arm");
		stage->setGoal("start");
		t.add(std::move(stage));
	}
	
	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
		stage->attachObject("obj_230", "l_gripper_tool_frame");
		t.add(std::move(stage));
	}

	{
		
		auto stage = std::make_unique<stages::MoveTo>("left end", pipeline);
		stage->setProperty("group", "left_arm");
		stage->setGoal("end");
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("pose object", cartesian);
		stage->setProperty("group", "left_arm");
		stage->setMinMaxDistance(0.01, 0.05);
		stage->setIKFrame("l_gripper_tool_frame");
		geometry_msgs::Vector3Stamped vec;
		vec.header.frame_id = "l_gripper_tool_frame";
		vec.vector.x = 1.0;
		stage->setDirection(vec);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
		stage->detachObject("obj_230", "l_gripper_tool_frame");
		t.add(std::move(stage));
	}

  //std::cerr << t << std::endl;


	t.plan(10);
}

int execute(Task &t)
{

	if(t.solutions().size() > 0)
	{
		actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> ac("execute_task_solution", true);
		ac.waitForServer();
		ROS_INFO("Executing solution trajectory");
		moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
		t.solutions().front()->fillMessage(execute_goal.solution);
		ac.sendGoal(execute_goal);
		ac.waitForResult();
		moveit_msgs::MoveItErrorCodes execute_result = ac.getResult()->error_code;

		if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
			ROS_ERROR_STREAM("Task execution failed and returned: " << ac.getState().toString());
			return -1;
		}
		return 0;
	}

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "pr2_pb");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh("~");

	std::cout << "waiting for any key + <enter>\n";
	char ch;
	std::cin >> ch;

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	std::vector<std::string> ids = planning_scene_interface.getKnownObjectNames();
	ROS_ERROR_STREAM("==================IDS BEFORE================");
	for(int i = 0; i < ids.size(); i ++)
	{
	ROS_ERROR_STREAM("[" << ids[i] << "]");
	}


	Task t1("myTask");
	try {
		planTest2(t1);

		std::cout << "waiting for any key + <enter>\n";
		std::cin >> ch;

		execute(t1);
	}
	catch (const InitStageException &e) {
		std::cerr << e;
		return EINVAL;
	}

	ids = planning_scene_interface.getKnownObjectNames();
	ROS_ERROR_STREAM("==================IDS AFTER================");
	for(int i = 0; i < ids.size(); i ++)
	{
	ROS_ERROR_STREAM("[" << ids[i] << "]");
	}


	return 0;
}
