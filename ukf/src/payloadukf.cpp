#include "ros/ros.h"
#include "payloadukf.h"
#include "Eigen/Dense"
#include <Eigen/Core>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
using Eigen::MatrixXd;
using Eigen::VectorXd;

Eigen::VectorXd  odometry;
geometry_msgs::Point force1,force2,pose,vel;
UKF::UKF()
{
    x_size = 12;
    y_size = 3;
    sigmavector_size = 2*x_size + 1;
    L = x_size;
    alpha = 1e-3;
    kappa = 0;
    beta = 2;
    lambda = alpha * alpha * (L + kappa) - L;

    //is_initialized_ = false;
    // initial state vector
    x = VectorXd(x_size);
    y = VectorXd(y_size);
    // initial covariance matrix
    P = MatrixXd(x_size, x_size);
    P_ = MatrixXd(x_size, x_size);

	x.setZero(x_size);
	y.setZero(y_size);
	x_hat.setZero(x_size);
	y_hat.setZero(y_size);
	x_a.setZero(x_size+x_size+y_size);
	x_sigmavector.setZero(x_size, sigmavector_size);
	y_sigmavector.setZero(y_size, sigmavector_size);
    H.setZero(y_size,x_size); 
    H << 1,0,0,0,0,0,0,0,0,0,0,0,
              0,1,0,0,0,0,0,0,0,0,0,0,
              0,0,1,0,0,0,0,0,0,0,0,0;
	w_c.setZero(sigmavector_size);
	w_m.setZero(sigmavector_size);

	//y = H*x;
	w_c(0) = (lambda / (L+lambda))+(1-alpha*alpha+beta);
	w_m(0) = (lambda)/(L+lambda);
	for(int i = 1 ; i < sigmavector_size ; i++) {
		w_c(i) = 1/(2*(L+lambda));
		w_m(i) = 1/(2*(L+lambda));
    }
    Q = 5e-7*Eigen::MatrixXd::Identity(x_size, x_size);
	R = 5e-4*Eigen::MatrixXd::Identity(y_size,y_size);
	P = 1e-3*Eigen::MatrixXd::Identity(x_size, x_size);
    P_.setZero(x_size,x_size);
	P_yy.setZero(y_size,y_size);
	P_xy.setZero(x_size,y_size);
    //Kalman_gain.setZero();
}

Eigen::MatrixXd UKF::Dynamics(Eigen::MatrixXd sigma_state)
{
	Eigen::MatrixXd predict_sigma_state(this->x_size , this->sigmavector_size);
	for(int i=0; i<this->sigmavector_size; i++) 
    {
		//sigma points
		double px = sigma_state(0,i);
		double py = sigma_state(1,i);
		double pz = sigma_state(2,i);
		double vx = sigma_state(3,i);
		double vy = sigma_state(4,i);
		double vz = sigma_state(5,i);
		double F1x = sigma_state(6,i) + gausian_noise(0);
		double F1y = sigma_state(7,i) + gausian_noise(1);
		double F1z = sigma_state(8,i) + gausian_noise(2);
		double F2x = sigma_state(9,i) + gausian_noise(0);
		double F2y = sigma_state(10,i) + gausian_noise(1);
		double F2z = sigma_state(11,i) + gausian_noise(2);

		Eigen::Vector3d P_last , V_last , P, V;
        Eigen::Vector3d F1_last , F2_last;
        Eigen::Vector3d payload_a, gravity;

		double dt = 0.02;
        double mass = 0.3;
	
        P_last << px, py, pz;
        V_last << vx, vy, vz;
        F1_last << F1x,F1y,F1z;
        F2_last << F2x,F2y,F2z;
        gravity << 0,0,9.81; 

        payload_a = ((F1_last+F2_last)/mass)+gravity;
        V = V_last + payload_a * dt;
        P = P_last + V * dt;
        F1_last = F1_last;
        F2_last = F2_last;

        predict_sigma_state(0,i) = P(0);
		predict_sigma_state(1,i) = P(1);
		predict_sigma_state(2,i) = P(2);
		predict_sigma_state(3,i) = V(0);
		predict_sigma_state(4,i) = V(1);
		predict_sigma_state(5,i) = V(2);
		predict_sigma_state(6,i) = F1_last(0);
		predict_sigma_state(7,i) = F1_last(1);
		predict_sigma_state(8,i) = F1_last(2);
		predict_sigma_state(9,i) = F2_last(0);
		predict_sigma_state(10,i) = F2_last(1);
		predict_sigma_state(11,i) = F2_last(2);
	}
	return predict_sigma_state;       
}

void UKF::Predict()
{
    P = (lambda+L)*(P+Q);
	Eigen::MatrixXd M = (P).llt().matrixL(); //Cholesky decomposition
	Eigen::MatrixXd buffer;
	x_sigmavector.setZero();
	x_sigmavector.col(0) = x;

	for(int i=0; i<x_size; i++) {
		Eigen::VectorXd sigma =(M.row(i)).transpose();
		x_sigmavector.col(i+1) = x + sigma;
		x_sigmavector.col(i+x_size+1) = x - sigma;
	}
    //Dynamics model
	buffer = Dynamics(x_sigmavector);
	x_sigmavector = buffer;
    // x_hat_k_-
 	x_hat.setZero(x_size);   //initialize x_hat
	for(int i=0; i<sigmavector_size; i++) {
		x_hat += w_m(i)* x_sigmavector.col(i);
		Eigen::VectorXd sigmavector_;
		sigmavector_ = x_sigmavector.col(i);
	}   
	//covariance
    // P_k_-
    P_.setZero(x_size,x_size);
	for(int i=0 ; i<sigmavector_size ; i++) {
		P_+=   w_c(i) * (x_sigmavector.col(i)-x_hat) * ((x_sigmavector.col(i)-x_hat).transpose());
	}
	//add process noise covariance
	P_+= Q;
    /*
    for(int i=0; i<x_sigmavector_size ; i++) {
		y_sigmavector = H*x_sigmavector;
	}
    //y_hat (mean)
    y_hat.setZero(y_size);
	for(int i=0; i< x_sigmavector_size; i++) {
		y_hat += w_m(i) * y_sigmavector.col(i);
	}
    */
}
void pose_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
	odometry << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
}
void UKF::Correct(Eigen::VectorXd odometry)
{
	for(int i=0; i<sigmavector_size ; i++) {
		y_sigmavector = H*x_sigmavector;
	}
    //y_hat (mean)
    y_hat.setZero(y_size);
	for(int i=0; i< sigmavector_size; i++) {
		y_hat += w_m(i) * y_sigmavector.col(i);
	}
	y = y - odometry;
	P_yy.setZero(y_size,y_size);
	P_xy.setZero(x_size,y_size);
    //Pykyk
	for(int i=0; i<sigmavector_size; i++) {
		Eigen::MatrixXd err;
		Eigen::MatrixXd err_t;
		err = y_sigmavector.col(i) - y_hat;
		err_t = err.transpose();
		P_yy += w_c(i) * err * err_t;
	}
	//add measurement noise covarinace
	P_yy +=R;

    //Pxkyk
	for(int i=0; i<sigmavector_size; i++) {
		Eigen::VectorXd err_y, err_x;
		err_y = y_sigmavector.col(i) - y_hat;
		err_x = x_sigmavector.col(i) - x_hat;
		P_xy += w_c(i) * err_x * err_y.transpose();
	}
	Kalman_gain = P_xy * (P_yy.inverse());
	x = x_hat + Kalman_gain *(y-y_hat);
	P = P_ - Kalman_gain*P_yy*(Kalman_gain.transpose());
}
UKF::~UKF() {}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ukf");
    ros::NodeHandle nh;
    ros::Subscriber odometry_sub;
	ros::Publisher pose_pub;
	ros::Publisher vel_pub;
	ros::Publisher force1_pub;
	ros::Publisher force2_pub;
    while (ros::ok())
    {
        pose_pub = nh.advertise<geometry_msgs::Point>("pose_estimate", 2);
        vel_pub = nh.advertise<geometry_msgs::Point>("vel_estimate", 2);
        force1_pub = nh.advertise<geometry_msgs::Point>("force1_estimate", 2);
        pose_pub = nh.advertise<geometry_msgs::Point>("force2_estimate", 2);
        odometry_sub = nh.subscribe("/payload/position", 50,pose_cb);
        //imu_corr_sub = nh.subscribe("/mavros/imu/mag", 1,Correct);
        ros::Rate r(50);
        while(1){
            pose_pub.publish(pose);
            vel_pub.publish(vel);
            force1_pub.publish(force1);
            force2_pub.publish(force2);
            ros::spinOnce();
            r.sleep();
        }
    }
    return 0;
}   