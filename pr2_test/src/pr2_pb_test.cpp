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

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi) {

	moveit_msgs::CollisionObject o;
	o.id= "object";
	o.header.frame_id= "base_footprint";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.65;
	o.primitive_poses[0].position.y = 0.18;
	o.primitive_poses[0].position.z = 0.88;
	o.primitive_poses[0].orientation.x =0.0;
	o.primitive_poses[0].orientation.y =0.0;
	o.primitive_poses[0].orientation.z =0.0;
	o.primitive_poses[0].orientation.w =1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
	o.primitives[0].dimensions.resize(3);
	o.primitives[0].dimensions[0]= 0.055;
	o.primitives[0].dimensions[1]= 0.055;
	o.primitives[0].dimensions[2]= 0.055*2;
	psi.applyCollisionObject(o);

	/*o.id= "obstacle";
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

	*/
	moveit_msgs::CollisionObject box;
	shape_msgs::Mesh mesh;
  	shapes::ShapeMsg mesh_msg;
  	shapes::Mesh* m;

	box.id = "obstacle";
	box.header.frame_id= "base_footprint";
	std::string mesh_uri("package://pr2_test/mesh/dt_box.dae");
	m = shapes::createMeshFromResource(mesh_uri);
	shapes::constructMsgFromShape(m, mesh_msg);
	mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
	// Add the mesh to the Collision object message
	box.meshes.push_back(mesh);
	geometry_msgs::Pose pose;
	pose.orientation.x = 0.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = -0.707;
	pose.orientation.w = 0.707;	
	pose.position.x = 0.66;
	pose.position.y = -0.10;
	pose.position.z = 0.93;
	box.mesh_poses.push_back(pose);
	psi.applyCollisionObject(box);

	moveit_msgs::CollisionObject boxUp;
	shape_msgs::Mesh meshUp;
  	shapes::ShapeMsg mesh_msgUp;
  	shapes::Mesh* mUp;

	boxUp.id = "obstacleUp";
	boxUp.header.frame_id= "base_footprint";
	std::string mesh_uriUp("package://pr2_test/mesh/dt_box.dae");
	mUp = shapes::createMeshFromResource(mesh_uriUp);
	shapes::constructMsgFromShape(mUp, mesh_msgUp);
	meshUp = boost::get<shape_msgs::Mesh>(mesh_msgUp);
	// Add the mesh to the Collision object message
	boxUp.meshes.push_back(meshUp);
	geometry_msgs::Pose poseUp;
	poseUp.orientation.x = 0.0;
	poseUp.orientation.y = 0.0;
	poseUp.orientation.z = -0.707;
	poseUp.orientation.w = 0.707;	
	poseUp.position.x = 0.66;
	poseUp.position.y = -0.10;
	poseUp.position.z = 1.2;
	boxUp.mesh_poses.push_back(poseUp);
	psi.applyCollisionObject(boxUp);


	
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

void planTest(Task &t) {

	t.loadRobotModel();

	auto cartesian = std::make_shared<solvers::CartesianPath>();
	cartesian->setProperty("jump_threshold", 0.0);

	auto gripper_planner = std::make_shared<solvers::JointInterpolationPlanner>();

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnect");
	pipeline->setProperty("longest_valid_segment_fraction", 0.001);

	Stage* current_state = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	current_state = initial.get();
	t.add(std::move(initial));

	{
		// connect to place
		stages::Connect::GroupPlannerVector planners = {{"left_arm", pipeline},{"left_gripper", gripper_planner}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(connect));
	}

	{
		// grasp generator
		auto grasp_generator = std::make_unique<stages::GenerateGraspPose>("generate grasp pose left");
		grasp_generator->setAngleDelta(M_PI/2);
		//grasp_generator->setTopGraspEnable(true);
		grasp_generator->setPreGraspPose("left_open");
		grasp_generator->setGraspPose("left_close");
		grasp_generator->setProperty("object", std::string("object"));
		grasp_generator->setMonitoredStage(current_state);
		auto grasp = std::make_unique<stages::SimpleGrasp>(std::move(grasp_generator));
		Eigen::Isometry3d tr = Eigen::Isometry3d::Identity();
		//tr.translation() = Eigen::Vector3d(0.0,0.0,0.0);
		grasp->setIKFrame(tr, "l_gripper_tool_frame");
		grasp->setMaxIKSolutions(32);

		// pick container, using the generated grasp generator
		auto pick = std::make_unique<stages::Pick>(std::move(grasp));
		pick->setProperty("eef", "left_gripper");
		pick->setProperty("group","left_arm");
		pick->setProperty("object", std::string("object"));
		geometry_msgs::TwistStamped approach;
		approach.header.frame_id = "l_gripper_tool_frame";
		approach.twist.linear.x = 1.0;
		pick->setApproachMotion(approach, 0.01, 0.10);

		geometry_msgs::TwistStamped lift;
		lift.header.frame_id = "base_footprint";
		lift.twist.linear.x =  0.0;
		lift.twist.linear.y =  0.0;
		lift.twist.linear.z =  0.1;
		pick->setLiftMotion(lift, 0.01, 0.10);
		current_state = pick.get();
		t.add(std::move(pick));
	}

	{
		// connect to place
		stages::Connect::GroupPlannerVector planners = {{"left_arm", pipeline},{"left_gripper", gripper_planner}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(connect));
	}


	{
		auto stage = std::make_unique<stages::MoveTo>("Left home", pipeline);
		stage->setProperty("group", "left_arm");
		stage->setGoal("LEFT_ARM_INITIAL_POSE");
		t.add(std::move(stage));
	}


	{
		auto stage = std::make_unique<stages::GeneratePose>("pose");
		geometry_msgs::PoseStamped p;
		p.header.frame_id= "obstacleUp";
		p.pose.position.x=  0.0;
		p.pose.position.y=  -0.05;
		p.pose.position.z=  0;
		p.pose.orientation.x=  0.0;
		p.pose.orientation.y=  0;
		p.pose.orientation.z=  0.707;
		p.pose.orientation.w=  0.707;

		stage->setPose(p);
		stage->properties().configureInitFrom(Stage::PARENT);

		stage->setMonitoredStage(current_state);

		auto wrapper = std::make_unique<stages::ComputeIK>("pose IK", std::move(stage) );
		wrapper->setMaxIKSolutions(32);
		wrapper->setIKFrame(Eigen::Isometry3d::Identity(),"l_gripper_tool_frame");
		wrapper->setProperty("group","left_arm");
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		t.add(std::move(wrapper));
	}

  //std::cerr << t << std::endl;


	t.plan(5);
}

void planTest2(Task &t) {

	t.loadRobotModel();

	auto cartesian = std::make_shared<solvers::CartesianPath>();
	cartesian->setProperty("jump_threshold", 0.0);

	auto gripper_planner = std::make_shared<solvers::JointInterpolationPlanner>();

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnect");
	pipeline->setProperty("longest_valid_segment_fraction", 0.01);

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
		stage->attachObject("object", "l_gripper_tool_frame");
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

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	spawnObject(planning_scene_interface);

	ros::Duration(3).sleep();

	std::cout << "waiting for any key + <enter>\n";
	char ch;
	std::cin >> ch;

	Task t("myTask");
	try {
		planTest(t);

		std::cout << "waiting for any key + <enter>\n";
		std::cin >> ch;

	}
	catch (const InitStageException &e) {
		std::cerr << e;
		return EINVAL;
	}

	execute(t);

	return 0;
}
