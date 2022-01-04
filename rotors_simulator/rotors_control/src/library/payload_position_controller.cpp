#include "rotors_control/payload_position_controller.h"
#include "rotors_control/tictoc.h"
#include "rotors_control/phy.h"
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

void PayloadPositionController::CalculateControlInput(nav_msgs::Odometry* iris1_control_input, nav_msgs::Odometry* iris2_control_input, nav_msgs::Odometry* error)
{
	
	// compute b_3_d and the acceleration
	Eigen::Vector3d force_control_input;
	ComputeDesiredForce(&force_control_input);
	//std::cout << "force_control_input\n" << force_control_input << std::endl;

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
	//std::cout << "thrust\n" << thrust << std::endl;

	// this block use moment control input to compute the rotor velocities of every rotor
	// [4, 1] vector for moment and thrust
	Eigen::Vector4d sys_thrust_moment;
	Eigen::Vector4d iris1_thrust_moment,iris2_thrust_moment;
	sys_thrust_moment(0) = thrust;
	sys_thrust_moment.block<3, 1>(1, 0) = moment_control_input;
	//std::cout << "sys_thrust_moment\n" << sys_thrust_moment << std::endl;
	

	// compute iris1 and iris2 control input algorithm
	Eigen::MatrixXd u_star;
	ComputeUstar(&u_star, &sys_thrust_moment);
	//std::cout << u_star << std::endl;
	iris1_thrust_moment = u_star.block<4, 1>(0, 0);
	iris2_thrust_moment = u_star.block<4, 1>(4, 0);


	// publish payload_control_input and it has already changed the order (thrust_moment --> moment_thrust)
	iris1_control_input->pose.pose.orientation.x = iris1_thrust_moment(1);
	iris1_control_input->pose.pose.orientation.y = iris1_thrust_moment(2);
	iris1_control_input->pose.pose.orientation.z = iris1_thrust_moment(3);
	iris1_control_input->pose.pose.orientation.w = iris1_thrust_moment(0);

	iris2_control_input->pose.pose.orientation.x = iris2_thrust_moment(1);
	iris2_control_input->pose.pose.orientation.y = iris2_thrust_moment(2);
	iris2_control_input->pose.pose.orientation.z = iris2_thrust_moment(3);
	iris2_control_input->pose.pose.orientation.w = iris2_thrust_moment(0);
}

void PayloadPositionController::ComputeUstar(Eigen::MatrixXd* u_star, Eigen::Vector4d* desired_control_input)
{
	//assert(u_star);
	Eigen::MatrixXd A(4, 8); 
	Eigen::MatrixXd H(8, 8);
	Eigen::MatrixXd A_T(8 ,4);
	//Eigen::MatrixXf u_star(8 ,1);
	//Eigen::MatrixXd desire_control_input(4 ,1);
	A.setZero(4,8);
	H.setZero(8,8);
	A_T.setZero(8,4);
	
	float w = odometry_.orientation.w();
	float x = odometry_.orientation.x();
	float y = odometry_.orientation.y();
	float z = odometry_.orientation.z();
	float A_yaw = atan2((2.0 * (w * z + x * y)),(1.0 - 2.0 * (y*y + z* z)));
	
	//desired_control_input << 1.0,2.0,3.0,4.0;
	A <<    1 ,                    0,                    0,  0,  1,                    0,                   0, 0, 
                0 ,  cos(A_yaw),  -sin(A_yaw),  0,  0,  cos(A_yaw), -sin(A_yaw), 0,
               -0.6 ,  sin(A_yaw),   cos(A_yaw),  0,  0.6,  sin(A_yaw),  cos(A_yaw), 0,
                0 ,                    0,                    0,  1,  0,                    0,                    0, 1; 
	A_T = A.transpose();
	H<< 	sqrt(2),            0,         0,          0,           0,            0,           0,           0,
                          0,  sqrt(2),          0,          0,           0,            0,           0,           0,
                          0,           0, sqrt(2),          0,           0,            0,           0,           0,
                          0,           0,          0, sqrt(2),           0,            0,           0,           0,
                          0,           0,          0,          0,  sqrt(2),            0,           0,           0,
                          0,           0,          0,          0,            0,  sqrt(2),           0,           0,
                          0,           0,          0,          0,            0,            0, sqrt(2),           0,
                          0,           0,          0,          0,            0,            0,           0, sqrt(2);			
	*u_star = (H.inverse()*H.inverse()*A_T*(A*(H.inverse()*H.inverse())*A_T).inverse())* (*desired_control_input);	
	
	//std::cout << "-------------A-------------"<<std::endl;
	//std::cout << A <<std::endl;
	//std::cout << "-------------Desired control input-------"<<std::endl;
	//std::cout << *desired_control_input <<std::endl;
	//std::cout << "-------------USTAR-------------"<<std::endl;
	//std::cout << *u_star <<std::endl;
	//std::cout << "-------------A*u_star-----------" <<std::endl;
	//std::cout << A*(*u_star) <<std::endl;
	
}


void PayloadPositionController::SetOdometry(const EigenOdometry& odometry)
{
	PhysicsParameters phy;
	odometry_ = odometry;
	odometry_.position.z() = odometry_.position.z()+phy.z_offset;
}

void PayloadPositionController::SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory)
{
	command_trajectory_ =  command_trajectory;
	controller_active_ = true;
}

void PayloadPositionController::ComputeDesiredForce(Eigen::Vector3d* force_control_input)
{
	PhysicsParameters phy;
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

	//Get Moment of Inertia
	phy.I_system << phy.Ixx_system, 0, 0,
                    0, phy.Iyy_system, 0,
                    0, 0, phy.Izz_system;  

	//connect the desired force with the acceleration command
	*force_control_input = (position_error.cwiseProduct(controller_parameters_.position_gain_)
	                        + velocity_error.cwiseProduct(controller_parameters_.velocity_gain_))
	                       - phy.m_system * vehicle_parameters_.gravity_  * e_3
	                       - phy.m_system * command_trajectory_.acceleration_W;
	
	//std::cout << "phy.m_system\n" << phy.m_system <<std::endl;
	//std::cout << "phy.I_system\n" << phy.I_system <<std::endl;

	//debug
	/*
	Eigen::Vector3d first_term, second_term, third_term, forth_term;
	first_term = position_error.cwiseProduct(controller_parameters_.position_gain_);
	std::cout << "position_error.cwiseProduct(controller_parameters_.position_gain_)\n" << first_term << std::endl;
	second_term = velocity_error.cwiseProduct(controller_parameters_.velocity_gain_);
	std::cout << "velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)\n" << second_term << std::endl;
	third_term = -phy.m_system * vehicle_parameters_.gravity_  * e_3;
	std::cout << "- phy.m_system * vehicle_parameters_.gravity_  * e_3\n" << third_term << std::endl;
	forth_term = -phy.m_system * command_trajectory_.acceleration_W ;
	std::cout << "- phy.m_system * command_trajectory_.acceleration_W\n" << forth_term << std::endl;
	*/
	//std::cout << "*force_control_input\n" << *force_control_input << std::endl;
}

// Implementation from the T. Payload et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void PayloadPositionController::ComputeDesiredMoment(const Eigen::Vector3d& force_control_input,
                Eigen::Vector3d* moment_control_input)
{
	PhysicsParameters phy;
	phy.I_system << phy.Ixx_system,              0,          	 	 0,
                             	 0, phy.Iyy_system,         		 0,
                            	 0,              0,  	phy.Izz_system;   
	assert(moment_control_input);

	// quaternion -> rotation matrix
	Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();

	// Get the desired rotation matrix.
	// b_1_d is the time derivative of desired trajectory
	Eigen::Vector3d b1_des;
	// yaw rotate function 
	/*double yaw = atan2(  command_trajectory_.velocity_W(1),command_trajectory_.velocity_W(0) );
	if(yaw <0 ) {
		//yaw+=6.28;
		yaw+=0.1;
	}*/

	double yaw = command_trajectory_.getYaw();
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
	                        + odometry_.angular_velocity.cross(phy.I_system*odometry_.angular_velocity);
}

Eigen::Vector3d PayloadPositionController::System_Error_rqt(){
	Eigen::Vector3d pos_err;
	pos_err = position_error;
	return pos_err;

}
}
