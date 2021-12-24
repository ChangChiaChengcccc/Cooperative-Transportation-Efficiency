#ifndef ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H
#define ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H


#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <list>
#include <ctime>
#include <cstdlib>
#include <chrono>
#include "rotors_control/common.h"
#include "rotors_control/parameters_payload.h"

namespace rotors_control
{


// Default values for the Payload position controller and the Asctec Firefly.
// original
/*
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(6, 6, 6);              // 16, 16, 16
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7);        // 14.7, 14.7, 14.7
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 3, 0.15);           // 2, 1.5, 0.035
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 0.52, 0.18);  // 0.52, 0.52, 0.025
*/
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(6, 6, 6);            
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7);      
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 9, 0.15);          
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 4, 0.18);  

class PayloadPositionControllerParameters
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	PayloadPositionControllerParameters()
		: position_gain_(kDefaultPositionGain),
		  velocity_gain_(kDefaultVelocityGain),
		  attitude_gain_(kDefaultAttitudeGain),
		  angular_rate_gain_(kDefaultAngularRateGain)
	{
		
	}

	Eigen::Matrix4Xd allocation_matrix_;
	Eigen::Vector3d position_gain_;
	Eigen::Vector3d velocity_gain_;
	Eigen::Vector3d attitude_gain_;
	Eigen::Vector3d angular_rate_gain_;
	RotorConfiguration rotor_configuration_;
};

class PayloadPositionController
{
public:
	PayloadPositionController();
	~PayloadPositionController();
	void CalculateControlInput(nav_msgs::Odometry* iris1_control_input, nav_msgs::Odometry* iris2_control_input, nav_msgs::Odometry* error);
	void ComputeUstar(Eigen::MatrixXd* u_star, Eigen::Vector4d* desired_control_input);
	void SetOdometry(const EigenOdometry& odometry);
	void SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory);

	PayloadPositionControllerParameters controller_parameters_;
	VehicleParameters vehicle_parameters_;

	double Psi;
	Eigen::Vector3d   position_error;
	Eigen::Vector3d   velocity_error;
	Eigen::Vector3d   angular_rate_error;
	Eigen::Vector3d   angle_error;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	bool initialized_params_;
	bool controller_active_;

	Eigen::MatrixX4d moment_thrust_to_rotor_velocities_;

	mav_msgs::EigenTrajectoryPoint command_trajectory_;
	EigenOdometry odometry_;

	void ComputeDesiredForce(Eigen::Vector3d* force_control_input);
	void ComputeDesiredMoment(const Eigen::Vector3d& force_control_input, Eigen::Vector3d* moment_control_input);
};
}

#endif // ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H
