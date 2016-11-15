#include <ros/ros.h>
#include <pluginlib/class_loader.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit/robot_state/robot_state.h>

#include <math.h>
#include <eigen_conversions/eigen_msg.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <kinect2_viewer/Merosy_Obj.h>
#include <kinect2_viewer/Vec_Obj.h>
#include <sstream>

// Globals
ros::Subscriber sub;
int aux = -1;
int obj_nr = 1;
std::vector<kinect2_viewer::Merosy_Obj> detected_objects;
std::vector<kinect2_viewer::Merosy_Obj> manipulator;
int SLEEP_TIME = 3; // in seconds


class Pose {
	public:
		Pose(double x, double y, double z, double roll, double pitch, double yaw) {
			dX = x;
			dY = y;
			dZ = z;
			roll_deg = roll;
			pitch_deg = pitch;
			yaw_deg = yaw;
		}
		double dX;
		double dY;
		double dZ;
		double roll_deg;
		double pitch_deg;
		double yaw_deg;
};


void send_joints_to_robot(const std::vector<std::string>& joint_names_received,
		std::vector<double>& joint_values_received, robot_state::RobotState &current_values)
{

	/* Move group for Robot and joints set */
	moveit::planning_interface::MoveGroup group("manipulator");

	group.setPlannerId("RRTkConfigDefault");
	std::map<std::string, double> joints;

	for(std::size_t i = 0; i < joint_names_received.size(); ++i)
	{
		joints[joint_names_received[i].c_str()] = joint_values_received[i];
		//ROS_INFO("Joints r %s: %f", joint_names_received[i].c_str(), joint_values_received[i]);
	}

	// New current state
	planning_scene_monitor::PlanningSceneMonitorPtr final_planning_scene =
		boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
	group.setStartState(current_values);

	group.setJointValueTarget(joints);

	moveit::planning_interface::MoveGroup::Plan plan;
	if (!group.plan(plan))
	{     //If fails we try with PRMkConfigDefault
		group.setPlannerId("PRMkConfigDefault");
		group.setStartState(current_values);
		group.setJointValueTarget(joints);

		if (!group.plan(plan))
			ROS_FATAL("Unable to create motion plan.  Aborting.");
		else
		{
			std::cout << "Move the robot? (Press 1 if YES)";
			aux = 1;
			if(aux==1)
				group.execute(plan);
		}
	}
	else
	{
		std::cout << "Move the robot? (Press 1 if YES)";
		aux = 1;
		if(aux==1)
			group.execute(plan);
	}
}

bool isStateValid(const planning_scene::PlanningScene *planning_scene, robot_state::RobotState *state,
		const robot_state::JointModelGroup *group, const double *joint_group_variable_values)
{
	state->setJointGroupPositions(group, joint_group_variable_values);
	std::vector<double> joint_values;
	state->copyJointGroupPositions(group, joint_values);
	collision_detection::CollisionRequest request;
	request.verbose = false;
	request.group_name = group->getName();
	collision_detection::CollisionResult result;
	planning_scene->checkCollision(request, result, *state);

	if (result.collision)
		return false;
	// limit for joints for the IK validation to be above the table
	else if (joint_values[1] > 0.3 || joint_values[0] < -0.5  || joint_values[4] < 0.3 || joint_values[4] > 1.5)
		return false;

	return planning_scene->isStateFeasible(*state);
}

kinect2_viewer::Merosy_Obj get_obj_closest_to_camera(
		std::vector<kinect2_viewer::Merosy_Obj>& objs)
{
	double closest = 100;
	for (size_t i = 0; i < objs.size(); ++i) {
		if (objs.at(i).center_x < closest) {
			closest = objs.at(i).center_x;
		}
	}
	return objs.at(closest);
}

void objCallback(const kinect2_viewer::Vec_Obj::ConstPtr& msg)
{
	// no detexted objects
	if (msg->objs.size() <= 1)
	{
		ROS_ERROR("Please put at least one object on the table");
		return;
	}

	// gripper and many objects
	else {
		for (size_t i = 0; i < msg->objs.size(); ++i) {
			if (msg->objs[i].min_z < 0.1) {
				detected_objects.push_back(msg->objs[i]);
			} else {
				manipulator.push_back(msg->objs[i]);
			}
		}
	}

	sub.shutdown();

	for (size_t i = 0; i < detected_objects.size(); ++i) {
		std::cout << "[" << i+1 << "]- Found object at (" <<
			detected_objects.at(i).center_x << ", " <<
			detected_objects.at(i).center_y << "," <<
			detected_objects.at(i).center_z << ")" << std::endl;
	}

	std::cout << "To where do you want to move? (Press 0 to Cancel)" << std::endl;
	aux = 1;
}

Eigen::Affine3d get_obj_pose(
		double dXObj,double dYObj, double dZObj,
		double roll_deg,double pitch_deg,double yaw_deg)
{
	double roll_rad = roll_deg * (M_PI/180);
	double pitch_rad = pitch_deg * (M_PI/180);
	double yaw_rad = yaw_deg * (M_PI/180);

	tf::Quaternion l_q;
	l_q.setRPY(roll_rad, pitch_rad, yaw_rad);

	double dXCenter = 0.64;
	double dYCenter = -0.015;
	double dZCenter = 0.185;

	/* Arm position */
	Eigen::Affine3d goal_pose = Eigen::Translation3d(dXCenter-dXObj, dYCenter-dYObj, dZCenter-dZObj) * Eigen::Quaterniond(l_q);
	return goal_pose;
}

void move_robot_to(Pose& pose)
{
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	// wait a bit for ros things to initialize
	ros::WallDuration(1.0).sleep();

	/* Model, scene and state */
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	planning_scene::PlanningScene planning_scene(kinematic_model);
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

	/* Joints and Robot group */
	std::vector<double> joint_values;
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

	/* Get the current state */
	planning_scene_monitor::PlanningSceneMonitor psm("robot_description");
	psm.startStateMonitor();
	sleep(1);
	robot_state::RobotState current_values = psm.getPlanningScene()->getCurrentState();
	psm.stopStateMonitor();

	/* Put the kinematic state in current state */
	current_values.copyJointGroupPositions(joint_model_group, joint_values);
	kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

	/* Print joint names and values */
	for(std::size_t i = 0; i < joint_names.size(); ++i)
	{
		ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}

	// Move the arm in position
	Eigen::Affine3d goal_pose = get_obj_pose(pose.dX, pose.dY, pose.dZ, pose.roll_deg, pose.pitch_deg, pose.yaw_deg);

	/* Arm group, model and end-effector */
	moveit::planning_interface::MoveGroup group("manipulator");
	std::string end_effector_name = group.getEndEffectorLink();

	/* Arm IK */
	if (!kinematic_state->setFromIK(joint_model_group, goal_pose, end_effector_name, 200, 0.1,
				boost::bind(&isStateValid, &planning_scene, _1, _2, _3)))
	{
		ROS_WARN("Was NOT able to set IK");
		return;
	}
	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

	send_joints_to_robot(joint_names, joint_values, current_values);
	sleep(SLEEP_TIME);
}


int main(int argc, char **argv)
{
	ros::init (argc, argv, "move");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	std::cout << "argc = " << argc << std::endl;
	double dXGripper = 0.25;
	double dYGripper = -0.0003;
	double dZGripper = 0.441;
	double dXObj = 0;
	double dYObj = 0;
	double dZObj = 0;
	double roll_deg=0;
	double pitch_deg=180;
	double yaw_deg=-90;
	if (argc >= 7) {
		std::istringstream iss1( argv[1] );
		iss1 >> dXObj;
		std::istringstream iss2( argv[2] );
		iss2 >> dYObj;
		std::istringstream iss3( argv[3] );
		iss3 >> dZObj;
		std::istringstream iss4( argv[4] );
		iss4 >> roll_deg;
		std::istringstream iss5( argv[5] );
		iss5 >> pitch_deg;
		std::istringstream iss6( argv[6] );
		iss6 >> yaw_deg;
	}

	std::cout << "argv[1] = " << argv[1] << ", " << dXObj << std::endl;
	std::cout << "argv[2] = " << argv[2] << ", " << dYObj << std::endl;
	std::cout << "argv[3] = " << argv[3] << ", " << dZObj << std::endl;
	std::cout << "argv[4] = " << argv[4] << ", " << roll_deg << std::endl;
	std::cout << "argv[5] = " << argv[5] << ", " << pitch_deg << std::endl;
	std::cout << "argv[6] = " << argv[6] << ", " << yaw_deg << std::endl;
	// Move the arm in position
	Pose pose1 = Pose(dXObj, dYObj, dZObj-0.02, roll_deg, pitch_deg, yaw_deg);
	move_robot_to(pose1);

	// {
	// 	// Move to preset active position
	// 	roll_deg=30;
	// 	pitch_deg=180;
	// 	yaw_deg=90;
	// 	Pose pose = Pose(dXGripper, dYGripper, dZGripper-0.73, roll_deg, pitch_deg, yaw_deg);
	// 	move_robot_to(pose);
	// }

	ros::shutdown();
	std::cout << "DONE" << std::endl;

	return 0;
}

