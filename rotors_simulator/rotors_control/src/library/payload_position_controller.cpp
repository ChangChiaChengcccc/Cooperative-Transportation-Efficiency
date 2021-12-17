#include "rotors_control/payload_position_controller.h"
#include "rotors_control/tictoc.h"
#include <ctime>
#include <cstdlib>
#include <chrono>

namespace rotors_control
{

PayloadPositionController::PayloadPositionController()
	: initialized_params_(false),
	  controller_active_(false)
{

}

PayloadPositionController::~PayloadPositionController() {}

void PayloadPositionController::CalculateControlInput(nav_msgs::Odometry* error, nav_msgs::Odometry* payload_control_input)
{
	
	// compute b_3_d and the acceleration
	Eigen::Vector3d force_control_input;
	ComputeDesiredForce(&force_control_input);

	// compute angular acceleration and moment control input
	Eigen::Vector3d moment_control_input;
	ComputeDesiredMoment(force_control_input, &moment_control_input);

	error->pose.pose.position.x = position_error(0);
	error->pose.pose.position.y = position_error(1);
	error->pose.pose.position.z = position_error(2);
	error->pose.pose.orientation.x = angle_error(0);
	error->pose.pose.orientation.y = angle_error(1);
	error->pose.pose.orientation.z = angle_error(2);
	error->pose.pose.orientation.w = Psi;
	error->twist.twist.linear.x = velocity_error(0);
	error->twist.twist.linear.y = velocity_error(1);
	error->twist.twist.linear.z = velocity_error(2);
	error->twist.twist.angular.x = angular_rate_error(0);
	error->twist.twist.angular.y = angular_rate_error(1);
	error->twist.twist.angular.z = angular_rate_error(2);

	// comput thrust control input and project thrust onto body z axis.
	double thrust = -force_control_input.dot(odometry_.orientation.toRotationMatrix().col(2));

	// this block use moment control input to compute the rotor velocities of every rotor
	// [4, 1] vector for moment and thrust
	Eigen::Vector4d moment_thrust;
	moment_thrust.block<3, 1>(0, 0) = moment_control_input;
	moment_thrust(3) = thrust;	

	// for publish payload_control_input
	payload_control_input->pose.pose.orientation.x = moment_thrust(0);
	payload_control_input->pose.pose.orientation.y = moment_thrust(1);
	payload_control_input->pose.pose.orientation.z = moment_thrust(2);
	payload_control_input->pose.pose.orientation.w = moment_thrust(3);
}


void PayloadPositionController::SetOdometry(const EigenOdometry& odometry)
{
	odometry_ = odometry;
}

void PayloadPositionController::SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory)
{
	command_trajectory_ =  command_trajectory;
	controller_active_ = true;
}

void PayloadPositionController::ComputeDesiredForce(Eigen::Vector3d* force_control_input)
{
	assert(force_control_input);
	// this function is used to compute b_3_d in paper
	Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());

	// calculate position error ([3, 1] vector)
	position_error = odometry_.position - command_trajectory_.position_W;

	// Transform velocity to world frame.
	// quaternion -> rotation matrix
	// compute velocity error in world frame
	const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
	Eigen::Vector3d velocity_W =  R_W_I * odometry_.velocity;
	velocity_error = velocity_W - command_trajectory_.velocity_W;

	//connect the desired force with the acceleration command
	*force_control_input = (position_error.cwiseProduct(controller_parameters_.position_gain_)
	                        + velocity_error.cwiseProduct(controller_parameters_.velocity_gain_))
	                       - vehicle_parameters_.mass_ * vehicle_parameters_.gravity_ * e_3
	                       - vehicle_parameters_.mass_ * command_trajectory_.acceleration_W;
}

// Implementation from the T. Payload et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void PayloadPositionController::ComputeDesiredMoment(const Eigen::Vector3d& force_control_input,
                Eigen::Vector3d* moment_control_input)
{
	assert(moment_control_input);

	// quaternion -> rotation matrix
	Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();

	// Get the desired rotation matrix.
	// b_1_d is the time derivative of desired trajectory
	Eigen::Vector3d b1_des;
	double yaw = atan2(  command_trajectory_.velocity_W(1),command_trajectory_.velocity_W(0) );
	if(yaw <0 ) {
		yaw+=6.28;
	}

	/*double yaw = command_trajectory_.getYaw();*/
	b1_des << cos(yaw), sin(yaw), 0;

	// b_3_d is calculated in ComputeDesiredForce()
	Eigen::Vector3d b3_des;
	b3_des = -force_control_input / force_control_input.norm();

	// b2_des = b3_des x b1_des
	Eigen::Vector3d b2_des;
	b2_des = b3_des.cross(b1_des);
	b2_des.normalize();

	// R_des = [b2_des x b3_des; b2_des; b3_des]
	Eigen::Matrix3d R_des;
	R_des.col(0) = b2_des.cross(b3_des);
	R_des.col(1) = b2_des;
	R_des.col(2) = b3_des;

	// Angle error according to Payload et al.
	// use vectorFromSkewMatrix() to compute e_R and put it into angle_error
	Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
	vectorFromSkewMatrix(angle_error_matrix, &angle_error);

	// TODO(burrimi) include angular rate references at some point.
	Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
	angular_rate_des[2] = command_trajectory_.getYawRate();
	angular_rate_error = odometry_.angular_velocity - R.transpose() * R_des * angular_rate_des;

	// Psi
	Eigen::Matrix3d I_RdtR = Eigen::Matrix3d::Identity(3, 3) - (R_des.transpose())*R;
	Psi = 0.5*(I_RdtR.trace());

	*moment_control_input = - angle_error.cwiseProduct(controller_parameters_.attitude_gain_)
	                        - angular_rate_error.cwiseProduct(controller_parameters_.angular_rate_gain_)
	                        + odometry_.angular_velocity.cross(vehicle_parameters_.inertia_*odometry_.angular_velocity);
}
}
