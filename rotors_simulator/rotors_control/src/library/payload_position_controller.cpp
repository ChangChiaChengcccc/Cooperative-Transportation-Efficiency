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
