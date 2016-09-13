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

// #include <shape_tools/solid_primitive_dims.h>

#include <math.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <moveit/robot_state/robot_state.h>
#include <kinect2_viewer/Merosy_Obj.h>
#include <kinect2_viewer/Vec_Obj.h>
//#include <moveit/robot_state/

double dXObj;
double dYObj;

double dXGripper;
double dYGripper;
double dZGripper;

ros::Subscriber sub;
int aux;

void send_joints_to_robot(const std::vector<std::string>& joint_names_received, std::vector<double>& joint_values_received, robot_state::RobotState &current_values)
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
            std::cin>>aux;
            if(aux==1)
                group.execute(plan);
        }
    }
    else
    {
        std::cout << "Move the robot? (Press 1 if YES)";
        std::cin>>aux;
        if(aux==1)
            group.execute(plan);
    }

}

void send_home(const std::vector<std::string> &joint_names, robot_state::RobotState &current_values)
{
    std::vector<double> joint_values;
	for (size_t i = 0; i < joint_values.size(); ++i) {
		joint_values.push_back(0.0);
	}
    send_joints_to_robot(joint_names, joint_values, current_values);
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
    // limit for joints (shoulder_pan_joint, shoulder_lift_joint)
    // else if (joint_values[2]<0.3 || joint_values[0]<-1.0 || joint_values[1]<-1.0)
    //    return false;
    //    else if ((group->getName()=="Right") && joint_values[0]<-0.2 || joint_values[0]>3.2
    //             && joint_values[1]<-3.33 || joint_values[1]>0.16)
    //        return false;


    return planning_scene->isStateFeasible(*state);
}

void objCallback(const kinect2_viewer::Vec_Obj::ConstPtr& msg)
{
	std::vector<kinect2_viewer::Merosy_Obj> detected_objects;

	// no detexted objects
    if (msg->objs.size() <= 1)
    {
        ROS_ERROR("Please put at least one object on the table");
        return;
    }

	// gripper and many objects
    else if (msg->objs.size() > 1) {
		for (size_t i = 0; i < msg->objs.size(); ++i) {
			if (msg->objs[i].center_z < 0.1) {
				detected_objects.push_back(msg->objs[i]);
			} else {
				kinect2_viewer::Merosy_Obj gripper = msg->objs[i];
				dXGripper = gripper.center_x;
				dYGripper = gripper.center_y;
				dZGripper = gripper.center_z;
			}
		}
	}

    sub.shutdown();

	for (size_t i = 0; i < detected_objects.size(); ++i) {
    std::cout << "[" << i+1 << "]- Found object at (" <<
		detected_objects.at(i).center_x << ", " <<
		detected_objects.at(i).center_y << ")" << std::endl;
	}

	std::cout << "To where do you want to move? (Press 0 to Cancel)" << std::endl;
    std::cin>>aux;

	std::cout << "Moving to Object " << aux << std::endl;

	if (aux > 0 and aux <= (int)detected_objects.size()) {
		dXObj = detected_objects.at(aux-1).center_x;
		dYObj = detected_objects.at(aux-1).center_y;
	}


//    for (int i=0;i<msg->objs.size();i++)
//    {
//        kinect2_viewer::Merosy_Obj obj = msg->objs[i];
//        ROS_INFO("I heard: [%d]", obj.id);
//    }
}


int main(int argc, char **argv)
{
    ros::init (argc, argv, "pick_place");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    dXObj = 0;
    dYObj = 0;

    aux = 0;

    ros::NodeHandle nh;

    sub = nh.subscribe("/merosy_objects", 5, objCallback);

    while (aux < 1)
        sleep(1);

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
    double l_roll_deg=0;
    double l_pitch_deg=180;
    double l_yaw_deg=-90;

    double l_roll_rad = l_roll_deg * (M_PI/180);
    double l_pitch_rad = l_pitch_deg * (M_PI/180);
    double l_yaw_rad = l_yaw_deg * (M_PI/180);

    tf::Quaternion createQuaternionFromRPY(double l_roll_rad,double l_pitch_rad,double l_yaw_rad);
    tf::Quaternion l_q;
    l_q.setRPY(l_roll_rad, l_pitch_rad, l_yaw_rad);


    double dXCenter = 0.64;
    double dYCenter = -0.015;
    double dZCenter = 0.185;

    /* Arm position */
    Eigen::Affine3d goal_pose = Eigen::Translation3d(dXCenter-dXObj, dYCenter-dYObj, dZCenter) //Z + Height/2 + error
            * Eigen::Quaterniond(l_q);

    /* Arm group, model and end-effector */
    moveit::planning_interface::MoveGroup group("manipulator");
    std::string end_effector_name = group.getEndEffectorLink();


    /* Arm IK */
    if (!kinematic_state->setFromIK(joint_model_group, goal_pose, end_effector_name, 200, 0.1,
                                    boost::bind(&isStateValid, &planning_scene, _1, _2, _3)))
    {
        ROS_WARN("Was NOT able to set IK for ");
        return false;
    }

    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

    send_joints_to_robot(joint_names, joint_values, current_values);
    sleep(5);

	{
		/* Get the current state */
		psm.startStateMonitor();
		sleep(1);
		current_values = psm.getPlanningScene()->getCurrentState();
		psm.stopStateMonitor();

		// Move the arm in position
		l_roll_deg=-45;
		l_pitch_deg=-180;
		l_yaw_deg=-90;

		l_roll_rad = l_roll_deg * (M_PI/180);
		l_pitch_rad = l_pitch_deg * (M_PI/180);
		l_yaw_rad = l_yaw_deg * (M_PI/180);

		l_q.setRPY(l_roll_rad, l_pitch_rad, l_yaw_rad);

		/* Put the kinematic state in current state */
		current_values.copyJointGroupPositions(joint_model_group, joint_values);
		kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

		goal_pose = Eigen::Translation3d(dXGripper+0.2, dYGripper, dZGripper + 0.1) * Eigen::Quaterniond(l_q);

		if (!kinematic_state->setFromIK(joint_model_group, goal_pose, end_effector_name, 200, 0.1,
					boost::bind(&isStateValid, &planning_scene, _1, _2, _3)))
		{
			ROS_WARN("Was NOT able to set IK for ");
			return false;
		}

		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

		send_joints_to_robot(joint_names, joint_values, current_values);
		sleep(5);
	}

    ros::shutdown();
    return 0;
}
