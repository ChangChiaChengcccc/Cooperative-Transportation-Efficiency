#ifndef PAYLOADUKF_H
#define PAYLOADUKF_H
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <nav_msgs/Odometry.h>

class UKF
{
public:
	UKF();
	~UKF();

	double L;
	double dt;
	//vector size
	int x_size;
	int y_size;
	int sigmavector_size;
	
	//Eigen::MatrixXd rotate(double roll, double yaw, double pitch);

	//void set_process_noise(Eigen::MatrixXd matrix);
	//void set_measurement_noise(Eigen::MatrixXd matrix);
	//void set_covariance_matrix(Eigen::MatrixXd matrix);
	//void set_measurement_matrix(Eigen::MatrixXd matrix);
	//void set_parameter(double alpha,double beta, double lambda, double kappa);
	void Predict();
	void Correct(Eigen::VectorXd);
	Eigen::MatrixXd Dynamics(Eigen::MatrixXd);
	Eigen::VectorXd x ; //states
	Eigen::VectorXd x_a;
	Eigen::VectorXd x_a_hat;
	Eigen::VectorXd y ; //measurements

	Eigen::VectorXd x_hat; //x mean
	Eigen::VectorXd y_hat; //y mean

	double alpha ;
	double kappa ;
	double beta ;
	double lambda ;
	//Eigen::Vector4d quat;
	//Eigen::Vector4d last_quat;
	//Eigen::Vector4d quat_m;
	//Eigen::Vector4d qk11;
	//Eigen::Vector3d euler_angle;
	//Eigen::Vector4d quaternion;
	//Eigen::Vector3d angular_v_measure;
	Eigen::Vector3d gausian_noise;
private:

	Eigen::VectorXd w_c ; //weight c
	Eigen::VectorXd w_m ;  //weight m



	Eigen::MatrixXd x_a_sigmavector;
	Eigen::MatrixXd x_sigmavector ;
	Eigen::MatrixXd y_sigmavector ;
	Eigen::MatrixXd H ;    //measurement transform

	Eigen::MatrixXd P ; //covariance matrix

	Eigen::MatrixXd Q ; //noise matrix
	Eigen::MatrixXd R ; //noise matrix

	Eigen::MatrixXd P_a ; //covariance matrix
	Eigen::MatrixXd P_ ; //covariance matrix
	Eigen::MatrixXd P_yy ;
	Eigen::MatrixXd P_xy ;

	Eigen::MatrixXd Kalman_gain ;


	// double y_hat = 0.0;
	// double x_hat = 0.0;


};

#endif // UKF_H


