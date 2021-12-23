#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "payload_position_controller_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control
{
PayloadPositionControllerNode::PayloadPositionControllerNode(const
                ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
	:nh_(nh),
	 private_nh_(private_nh)
{

	cmd_sub_ = nh_.subscribe(
	                   "/payload/desired_trajectory", 1,
	                   &PayloadPositionControllerNode::CommandCallback, this);
	/*
	cmd_multi_dof_joint_trajectory_sub_ = nh_.subscribe(
	                mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
	                &PayloadPositionControllerNode::MultiDofJointTrajectoryCallback, this);
	*/
	odometry_sub_ = nh_.subscribe("/payload/odometry", 1,
	                              &PayloadPositionControllerNode::OdometryCallback, this);

	error_pub_ = nh_.advertise<nav_msgs::Odometry>("/system/error", 1);

	iris1_control_input_pub_ = nh_.advertise<nav_msgs::Odometry>("/iris1_control_input", 1);
	iris2_control_input_pub_ = nh_.advertise<nav_msgs::Odometry>("/iris2_control_input", 1);

	command_timer_ = nh_.createTimer(ros::Duration(0), &PayloadPositionControllerNode::TimedCommandCallback, this,
	                                 true, false); 
}

PayloadPositionControllerNode::~PayloadPositionControllerNode() { }

void PayloadPositionControllerNode::InitializeParams()
{
	// Read parameters from rosparam.
	GetRosParameter(private_nh_, "position_gain/x",
	                payload_position_controller_.controller_parameters_.position_gain_.x(),
	                &payload_position_controller_.controller_parameters_.position_gain_.x());
	GetRosParameter(private_nh_, "position_gain/y",
	                payload_position_controller_.controller_parameters_.position_gain_.y(),
	                &payload_position_controller_.controller_parameters_.position_gain_.y());
	GetRosParameter(private_nh_, "position_gain/z",
	                payload_position_controller_.controller_parameters_.position_gain_.z(),
	                &payload_position_controller_.controller_parameters_.position_gain_.z());
	GetRosParameter(private_nh_, "velocity_gain/x",
	                payload_position_controller_.controller_parameters_.velocity_gain_.x(),
	                &payload_position_controller_.controller_parameters_.velocity_gain_.x());
	GetRosParameter(private_nh_, "velocity_gain/y",
	                payload_position_controller_.controller_parameters_.velocity_gain_.y(),
	                &payload_position_controller_.controller_parameters_.velocity_gain_.y());
	GetRosParameter(private_nh_, "velocity_gain/z",
	                payload_position_controller_.controller_parameters_.velocity_gain_.z(),
	                &payload_position_controller_.controller_parameters_.velocity_gain_.z());
	GetRosParameter(private_nh_, "attitude_gain/x",
	                payload_position_controller_.controller_parameters_.attitude_gain_.x(),
	                &payload_position_controller_.controller_parameters_.attitude_gain_.x());
	GetRosParameter(private_nh_, "attitude_gain/y",
	                payload_position_controller_.controller_parameters_.attitude_gain_.y(),
	                &payload_position_controller_.controller_parameters_.attitude_gain_.y());
	GetRosParameter(private_nh_, "attitude_gain/z",
	                payload_position_controller_.controller_parameters_.attitude_gain_.z(),
	                &payload_position_controller_.controller_parameters_.attitude_gain_.z());
	GetRosParameter(private_nh_, "angular_rate_gain/x",
	                payload_position_controller_.controller_parameters_.angular_rate_gain_.x(),
	                &payload_position_controller_.controller_parameters_.angular_rate_gain_.x());
	GetRosParameter(private_nh_, "angular_rate_gain/y",
	                payload_position_controller_.controller_parameters_.angular_rate_gain_.y(),
	                &payload_position_controller_.controller_parameters_.angular_rate_gain_.y());
	GetRosParameter(private_nh_, "angular_rate_gain/z",
	                payload_position_controller_.controller_parameters_.angular_rate_gain_.z(),
	                &payload_position_controller_.controller_parameters_.angular_rate_gain_.z());
	GetVehicleParameters(private_nh_, &payload_position_controller_.vehicle_parameters_);
}

void PayloadPositionControllerNode::CommandCallback(
        const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr& cmd_msg)
{
	// Clear all pending commands.
	command_timer_.stop();
	commands_.clear();
	command_waiting_times_.clear();

	mav_msgs::EigenTrajectoryPoint eigen_reference;
	// put cmd_msg data into eigen_reference
	mav_msgs::eigenTrajectoryPointFromMsg(*cmd_msg, &eigen_reference);
	commands_.push_front(eigen_reference);
	// put eigen_reference into command_trajectory_ under the payload_position_controller_
	payload_position_controller_.SetTrajectoryPoint(commands_.front());
	commands_.pop_front();
	// std::cout << "In payload CommandCallback function" << std::endl;
}

void PayloadPositionControllerNode::MultiDofJointTrajectoryCallback(
        const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg)
{
	// Clear all pending commands.
	command_timer_.stop();
	commands_.clear();
	command_waiting_times_.clear();

	const size_t n_commands = msg->points.size();

	if(n_commands < 1) {
		ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
		return;
	}

	mav_msgs::EigenTrajectoryPoint eigen_reference;
	mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
	commands_.push_front(eigen_reference);

	for (size_t i = 1; i < n_commands; ++i) {
		const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
		const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];

		mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

		commands_.push_back(eigen_reference);
		command_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
	}

	// We can trigger the first command immediately.
	payload_position_controller_.SetTrajectoryPoint(commands_.front());
	commands_.pop_front();

	if (n_commands > 1) {
		command_timer_.setPeriod(command_waiting_times_.front());
		command_waiting_times_.pop_front();
		command_timer_.start();
	}
}

void PayloadPositionControllerNode::TimedCommandCallback(const ros::TimerEvent& e)
{
	if(commands_.empty()) {
		ROS_WARN("Commands empty, this should not happen here");
		return;
	}

	const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
	payload_position_controller_.SetTrajectoryPoint(commands_.front());
	commands_.pop_front();
	command_timer_.stop();
	if(!command_waiting_times_.empty()) {
		command_timer_.setPeriod(command_waiting_times_.front());
		command_waiting_times_.pop_front();
		command_timer_.start();
	}
}

void PayloadPositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
	ROS_INFO_ONCE("PayloadPositionController got first odometry message.");

	// put sensor data from gazebo to payload_position_controller_
	// odometry contain position, orientation, velocity, angular_velocity
	EigenOdometry odometry;
	eigenOdometryFromMsg(odometry_msg, &odometry);
	payload_position_controller_.SetOdometry(odometry);
	std::cout << "In payload OdometryCallback function" << std::endl;

	// CalculateRotorVelocities() is called to calculate rotor velocities and put into ref_rotor_velocities
	nav_msgs::Odometry payload_error;
	nav_msgs::Odometry iris1_control_input, iris2_control_input;
	payload_position_controller_.CalculateControlInput(&iris1_control_input, &iris2_control_input, &payload_error);

	error_pub_.publish(payload_error);
	iris1_control_input_pub_.publish(iris1_control_input);
	iris2_control_input_pub_.publish(iris2_control_input);
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "payload_position_controller_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	rotors_control::PayloadPositionControllerNode payload_position_controller_node(nh, private_nh);

	// this node will call subscribe function as long as someone publish data to topic
	// PayloadPositionControllerNode::CommandCallback()
	// PayloadPositionControllerNode::MultiDofJointTrajectoryCallback()
	// PayloadPositionControllerNode::OdometryCallback()

	ros::spin();

	return 0;
}
