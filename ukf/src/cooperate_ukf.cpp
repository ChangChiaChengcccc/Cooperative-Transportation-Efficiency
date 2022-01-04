#include <ros/ros.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "forceest.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Point.h"
#include "math.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"
#include "mavros_msgs/RCOut.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/BatteryState.h"
#include <string>
#include <gazebo_msgs/ModelStates.h>
#include "geometry_msgs/WrenchStamped.h"
#include <random>
#include <nav_msgs/Odometry.h>



#define l 0.25
#define k 0.02
std::string model_name;
int drone_flag;
forceest forceest1(statesize,measurementsize);
geometry_msgs::Point euler, euler_ref, force1,force2, torque, bias, angular_v,pwm,battery_p,pose,vel;
sensor_msgs::Imu drone2_imu;

nav_msgs::Odometry odometry;
geometry_msgs::PoseStamped payload_pose;
geometry_msgs::TwistStamped payload_vel;


void pose_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
	odometry = *msg;
	payload_pose.pose = odometry.pose.pose;
	payload_vel.twist = odometry.twist.twist;

}





int main(int argc, char **argv)
{
	ros::init(argc, argv, "cooperate_ukf_estimate");
	ros::NodeHandle nh;

	std::string topic_thrust, topic_vel, topic_mag;
	ros::param::get("~topic_thrust", topic_thrust);
	ros::param::get("~topic_vel", topic_vel);
	ros::param::get("~topic_drone", drone_flag);
	ros::param::get("~topic_mag", topic_mag);
	

/*
	if(drone_flag == 2) {
		model_name = "/firefly2/firefly2";
		std::cout << "drone 2"<<std::endl;
	}
	if(drone_flag ==1 ) {
		model_name = "/firefly1/firefly1";
		std::cout << "drone 1"<<std::endl;

	}
*/



	ros::Subscriber pos_sub = nh.subscribe<nav_msgs::Odometry>("/payload/odometry", 4, pose_cb);

	ros::Publisher pose_pub = nh.advertise<geometry_msgs::Point>("pose_estimate", 2);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Point>("vel_estimate", 2);
    ros::Publisher force1_pub = nh.advertise<geometry_msgs::Point>("force1_estimate", 2);
    ros::Publisher force2_pub = nh.advertise<geometry_msgs::Point>("force2_estimate", 2);
	ros::Rate loop_rate(50);

	double measure_ex, measure_ey, measure_ez;
	double sum_pwm;
	int count = 1;
	Eigen::MatrixXd mnoise;
	mnoise.setZero(measurementsize,measurementsize);
	mnoise   = 3e-3*Eigen::MatrixXd::Identity(measurementsize, measurementsize);

	mnoise(mp_x,mp_x) = 1e-4;
	mnoise(mp_y,mp_y) = 1e-4;
	mnoise(mp_z,mp_z) = 1e-4;
	mnoise(mv_x,mv_x) = 1e-2;
	mnoise(mv_y,mv_y) = 1e-2;
	mnoise(mv_z,mv_z) = 1e-2;

	mnoise(me_x,me_x) = 1;//1
	mnoise(me_y,me_y) = 1;
	mnoise(me_z,me_z) = 1;

	forceest1.set_measurement_noise(mnoise);

	Eigen::MatrixXd pnoise;
	pnoise.setZero(statesize,statesize);
	pnoise(p_x,p_x) = 1e-2;
	pnoise(p_y,p_y) = 1e-2;
	pnoise(p_z,p_z) = 1e-2;
	pnoise(v_x,v_x) = 1e-2;
	pnoise(v_y,v_y) = 1e-2;
	pnoise(v_z,v_z) = 1e-2;

	pnoise(e_x,e_x) = 0.005;//0.5,調小beta收斂較快
	pnoise(e_y,e_y) = 0.005;
	pnoise(e_z,e_z) = 0.005;

	pnoise(F1_x,F1_x) = 1.5;
	pnoise(F1_y,F1_y) = 1.5;
	pnoise(F1_z,F1_z) = 1.5;

    pnoise(F2_x,F2_x) = 1.5;
	pnoise(F2_y,F2_y) = 1.5;
	pnoise(F2_z,F2_z) = 1.5;


	forceest1.set_process_noise(pnoise);



	Eigen::MatrixXd measurement_matrix;
	measurement_matrix.setZero(measurementsize,statesize);

	measurement_matrix(mp_x,p_x) = 1;
	measurement_matrix(mp_y,p_y) = 1;
	measurement_matrix(mp_z,p_z) = 1;


	measurement_matrix(mv_x,v_x) = 1;
	measurement_matrix(mv_y,v_y) = 1;
	measurement_matrix(mv_z,v_z) = 1;

	measurement_matrix(me_x,e_x) = 1;//1,調小，beta會劇烈震盪
	measurement_matrix(me_y,e_y) = 1;
	measurement_matrix(me_z,e_z) = 1;

	forceest1.set_measurement_matrix(measurement_matrix);
	double start_time;

	while(ros::ok()) {

		double F1, F2, F3, F4,F5,F6;
		double U_x, U_y, U_z;
		double battery_dt;

		Eigen::MatrixXd theta_q(4,3), phi_q(4,3);
		Eigen::Matrix3d A_q;
		Eigen::Vector3d y_k;
		Eigen::Vector3d mag_v;

		const double mean = 0.0;
		const double stddev = 0.1;
		std::default_random_engine generatorx,generatory,generatorz;
		std::normal_distribution<double> distx(mean,stddev);
		std::normal_distribution<double> disty(mean,stddev);
		std::normal_distribution<double> distz(mean,stddev);
		forceest1.gausian_noise << distx(generatorx), disty(generatory), distz(generatorz);

		pose.x = payload_pose.pose.position.x;

		battery_dt = ros::Time::now().toSec() - start_time;
        /*
		if(drone2_imu.angular_velocity.x!=0 && drone2_pose.pose.position.x !=0 && drone2_vel.twist.linear.x !=0) {


			if((drone_flag==2)||(drone_flag ==1)) {
				double a;
				F1 = f3(2);//(6.13176e-06*(pwm3*pwm3) -0.0181164*pwm3 + 15.9815); //drone2
				F2 = f1(2);//(6.13176e-06*(pwm1*pwm1) -0.0181164*pwm1 + 15.9815); //left_right:265.7775
				F3 =  f4(2);//(6.13176e-06*(pwm4*pwm4) -0.0181164*pwm4 + 15.9815); //up_down:265.7775
				F4 = f2(2);//(6.13176e-06*(pwm2*pwm2) -0.0181164*pwm2 + 15.9815);
			}
			forceest1.thrust = f1(2) + f2(2) + f3(2) + f4(2)  +f5(2)+f6(2);

			U_x = (sqrt(2)/2)*l*(F1 - F2 - F3 + F4);
			U_y = (sqrt(2)/2)*l*(-F1 - F2 + F3 + F4);
			U_z = k*F1 - k*F2 + k*F3 - k*F4;

			forceest1.U << U_x, U_y, U_z;
			double x = drone2_pose.pose.orientation.x;
			double y = drone2_pose.pose.orientation.y;
			double z = drone2_pose.pose.orientation.z;
			double w = drone2_pose.pose.orientation.w;

			forceest1.R_IB.setZero();
			forceest1.R_IB << w*w+x*x-y*y-z*z, 2*x*y-2*w*z,            2*x*z+2*w*y,
			               2*x*y +2*w*z, w*w-x*x+y*y-z*z,2*y*z-2*w*x,
			               2*x*z -2*w*y, 2*y*z+2*w*x,w*w-x*x-y*y+z*z;


			forceest1.angular_v_measure << drone2_imu.angular_velocity.x,
			                            drone2_imu.angular_velocity.y,
			                            drone2_imu.angular_velocity.z;

			forceest1.predict();
			Eigen::VectorXd measure;
			measure.setZero(measurementsize);

			measure << drone2_pose.pose.position.x, drone2_pose.pose.position.y, drone2_pose.pose.position.z,
			        drone2_vel.twist.linear.x, drone2_vel.twist.linear.y, drone2_vel.twist.linear.z,
			        measure_ex, measure_ey, measure_ez,
			        drone2_imu.angular_velocity.x, drone2_imu.angular_velocity.y, drone2_imu.angular_velocity.z,
			        drone2_imu.linear_acceleration.x,drone2_imu.linear_acceleration.y,drone2_imu.linear_acceleration.z;

			forceest1.qk11 = forceest1.qk1;

			forceest1.correct(measure);
			forceest1.x[e_x] = 0;
			forceest1.x[e_y] = 0;
			forceest1.x[e_z] = 0;

			bias.x = forceest1.x[beta_x];
			bias.y = forceest1.x[beta_y];
			bias.z = forceest1.x[beta_z];

			euler.x = forceest1.euler_angle(0);//roll:forceest1.euler_angle(0)
			euler.y = forceest1.euler_angle(1);//pitch:forceest1.euler_angle(1)
			euler.z = forceest1.euler_angle(2);//yaw:forceest1.euler_angle(2)

			angular_v.x = drone2_imu.angular_velocity.x;
			angular_v.y = drone2_imu.angular_velocity.y;
			angular_v.z = drone2_imu.angular_velocity.z;
			tf::Quaternion quat_transform_ref(drone2_pose.pose.orientation.x,drone2_pose.pose.orientation.y,drone2_pose.pose.orientation.z,drone2_pose.pose.orientation.w);
			double roll_ref,pitch_ref,yaw_ref;
			tf::Matrix3x3(quat_transform_ref).getRPY(roll_ref,pitch_ref,yaw_ref);

			euler_ref.x = roll_ref*180/3.1415926;        //roll_ref*180/3.1415926
			euler_ref.y = pitch_ref*180/3.1415926;       //pitch_ref*180/3.1415926
			euler_ref.z = yaw_ref*180/3.1415926;         //yaw_ref*180/3.1415926

			euler_pub.publish(euler);
			euler_ref_pub.publish(euler_ref);


			force.x = forceest1.x[F_x];
			force.y = forceest1.x[F_y];
			force.z = forceest1.x[F_z];
			torque.z = forceest1.x[tau_z];

			force_pub.publish(force);

		}
        */
       forceest1.predict();
		Eigen::VectorXd measure;
		measure.setZero(measurementsize);

		measure << payload_pose.pose.position.x, payload_pose.pose.position.y, payload_pose.pose.position.z,
			     payload_vel.twist.linear.x, payload_vel.twist.linear.y, payload_vel.twist.linear.z;
			    //measure_ex, measure_ey, measure_ez,
			    

		forceest1.qk11 = forceest1.qk1;

		forceest1.correct(measure);
		forceest1.x[e_x] = 0;
		forceest1.x[e_y] = 0;
		forceest1.x[e_z] = 0;



		force1.x = forceest1.x[F1_x];
		force1.y = forceest1.x[F1_y];
		force1.z = forceest1.x[F1_z];
		force1_pub.publish(force1);
        
	    force2.x = forceest1.x[F2_x];
		force2.y = forceest1.x[F2_y];
		force2.z = forceest1.x[F2_z];
		force2_pub.publish(force2);

        pose.x = forceest1.x[p_x];
		pose.y = forceest1.x[p_y];
		pose.z = forceest1.x[p_z];
		pose_pub.publish(pose);

        vel.x = forceest1.x[v_x];
		vel.y = forceest1.x[v_y];
		vel.z = forceest1.x[v_z];
		vel_pub.publish(vel);

		loop_rate.sleep();
		ros::spinOnce();

	}
}
