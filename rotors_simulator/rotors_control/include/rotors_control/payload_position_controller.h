#ifndef ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H
#define ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H


#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <geometry_msgs/WrenchStamped.h>
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
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(6, 6, 6);            
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7);      
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 9, 0.15);          
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 4, 0.18);  
*/
/*
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(6, 6, 6);            
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7);      
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 9, 0.15);          
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 4, 0.18);    
*/
/*
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(6, 6, 6);              // 16, 16, 16
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7);        // 14.7, 14.7, 14.7
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 9, 0.15);           // 2, 1.5, 0.035
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 5, 0.18);  // 0.52, 0.52, 0.025
*/
/*
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(8, 8, 6);              // 16, 16, 16
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7);        // 14.7, 14.7, 14.7
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 11, 0.15);           // 2, 1.5, 0.035
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 5, 0.18);  // 0.52, 0.52, 0.025
*/
/* crash because of gain
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(8, 8, 6);              // 16, 16, 16
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7);        // 14.7, 14.7, 14.7
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 11, 0.15);           // 2, 1.5, 0.035
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 5, 0.18);  // 0.52, 0.52, 0.025
*/
/*
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(15, 15, 6);              // 16, 16, 16
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7);        // 14.7, 14.7, 14.7
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 22, 0.15);           // 2, 1.5, 0.035
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 5, 0.18);  // 0.52, 0.52, 0.025
*/
/*best now no y axis swing and stable at last
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(15, 15, 6);              // 16, 16, 16
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7);        // 14.7, 14.7, 14.7
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 30, 0.15);           // 2, 1.5, 0.035
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 5, 0.18);  // 0.52, 0.52, 0.025
*/
/*
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(22, 22, 6);              // 16, 16, 16
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7);        // 14.7, 14.7, 14.7
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 80, 0.15);           // 2, 1.5, 0.035
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 15, 0.18);  // 0.52, 0.52, 0.025
*/
// the key is tuning  Kpos and katt 22 80

///Good Gain Error <1
/*
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(18, 18, 6);              // 15,15,6
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(5, 5, 4.7);        // 4.7, 4.7, 4.7
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 40, 0.3);           // 3, 35, 0.15
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 6, 0.18);  // 0.52, 5, 0.18
*/

// Error <1
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(22, 22, 6);              // 15,15,6
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(5, 5, 4.7);        // 4.7, 4.7, 4.7
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3.2, 48, 0.3);           // 3, 35, 0.15
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 7.5, 0.18);  // 0.52, 5, 0.18

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
	void CalculateControlInput(nav_msgs::Odometry* iris1_control_input, nav_msgs::Odometry* iris2_control_input, nav_msgs::Odometry* error, Eigen::Vector4d* iris1_control_input_multiarray, Eigen::Vector4d* iris2_control_input_multiarray);
	void ComputeUstar(Eigen::MatrixXd* u_star, Eigen::Vector4d* desired_control_input);
	void ComputeQuadStates(Eigen::Vector3d* x1 ,Eigen::Vector3d* x2 ,Eigen::Vector3d* v1,Eigen::Vector3d* v2);
	void SetOdometry(const EigenOdometry& odometry);
	void SetFTsensor1(const geometry_msgs::WrenchStampedConstPtr &ft1_msg);
	void SetFTsensor2(const geometry_msgs::WrenchStampedConstPtr &ft2_msg);
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

	Eigen::Vector3d F1;
	Eigen::Vector3d F2;
	Eigen::Vector3d T1;
	Eigen::Vector3d T2;

	void ComputeDesiredForce(Eigen::Vector3d* force_control_input);
	void ComputeDesiredMoment(const Eigen::Vector3d& force_control_input, Eigen::Vector3d* moment_control_input);
	Eigen::Vector3d System_Error_rqt();
};
}

#endif // ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H
