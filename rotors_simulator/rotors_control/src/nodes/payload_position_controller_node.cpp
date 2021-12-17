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
	                   "/firefly1/desired_trajectory", 1,
	                   &PayloadPositionControllerNode::CommandCallback, this);

	cmd_multi_dof_joint_trajectory_sub_ = nh_.subscribe(
	                mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
	                &PayloadPositionControllerNode::MultiDofJointTrajectoryCallback, this);

	odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
	                              &PayloadPositionControllerNode::OdometryCallback, this);

	motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(
	                                        mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

	error_pub_ = nh_.advertise<nav_msgs::Odometry>("/error", 1);

	command_timer_ = nh_.createTimer(ros::Duration(0), &PayloadPositionControllerNode::TimedCommandCallback, this,
	                                 true, false);
}

PayloadPositionControllerNode::~PayloadPositionControllerNode() { }


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
	// put eigen_reference into command_trajectory_ under the Payload_position_controller_
	Payload_position_controller_.SetTrajectoryPoint(commands_.front());
	commands_.pop_front();
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
	Payload_position_controller_.SetTrajectoryPoint(commands_.front());
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
	Payload_position_controller_.SetTrajectoryPoint(commands_.front());
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

	// put sensor data from gazebo to Payload_position_controller_
	// odometry contain position, orientation, velocity, angular_velocity
	EigenOdometry odometry;
	eigenOdometryFromMsg(odometry_msg, &odometry);
	Payload_position_controller_.SetOdometry(odometry);

	// CalculateRotorVelocities() is called to calculate rotor velocities and put into ref_rotor_velocities
	Eigen::VectorXd ref_rotor_velocities;
	nav_msgs::Odometry error;

	// Todo(ffurrer): Do this in the conversions header.
	mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

	actuator_msg->angular_velocities.clear();
	for (int i = 0; i < ref_rotor_velocities.size(); i++)
		actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
	actuator_msg->header.stamp = odometry_msg->header.stamp;

	motor_velocity_reference_pub_.publish(actuator_msg);

	error_pub_.publish(error);
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Payload_position_controller_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	rotors_control::PayloadPositionControllerNode Payload_position_controller_node(nh, private_nh);

	// this node will call subscribe function as long as someone publish data to topic
	// PayloadPositionControllerNode::CommandCallback()
	// PayloadPositionControllerNode::MultiDofJointTrajectoryCallback()
	// PayloadPositionControllerNode::OdometryCallback()

	ros::spin();

	return 0;
}
