#include <ros/ros.h>
#include <geometric_controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf/transform_datatypes.h>
#include <qptrajectory.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <qptrajectory.h>
#include <tf2/transform_datatypes.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#define normal
#define PI 3.14159


gazebo_msgs::ModelStates model_states;
float vir_x, vir_y,vir_z,vx,vy,vz,ax,ay,az;
unsigned int tick=0;
unsigned int counter = 0;
bool flag = false;
mavros_msgs::State current_state;
gazebo_msgs::LinkStates link_states;
geometry_msgs::PoseStamped desired_pose;
double tt;
double r = 2;
double T = 280*M_PI;


int main(int argc, char **argv)
{

	ros::init(argc, argv, "geo");
	ros::NodeHandle nh;

	ROS_INFO("Hello world!");

	ros::Publisher  traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/firefly1/desired_trajectory", 1);
	ros::Publisher  traj_pub2= nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/firefly2/desired_trajectory", 1);
	double dt = 50.0;
	ros::Rate loop_rate(dt);
	nh.setParam("/start",true);
	trajectory_msgs::MultiDOFJointTrajectoryPoint traj,traj2;

	//planning
	qptrajectory plan;
	path_def path;
	trajectory_profile p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
	std::vector<trajectory_profile> data;
	double sample = 0.003;

	p1.pos << 1.2, 0, 1.3;
	p1.vel<< 0, 0, 0;
	p1.acc<< 0, 0, 0;
	p1.yaw = 0;

	p2.pos<< 4, 4, 1.3;
	p2.vel<< 0, 0, 0;
	p2.acc<< 0, 0, 0;
	p2.yaw = 0;

	p3.pos<< -4, 4, 1.3;
	p3.vel<< 0, 0, 0;
	p3.acc<< 0, 0, 0;
	p3.yaw = 0;

	p4.pos << -4, -4, 1.3;
	p4.vel << 0, 0, 0;
	p4.acc << 0, 0, 0;
	p4.yaw = 0;

	p5.pos << 4, -4, 1.3;
	p5.vel << 0, 0, 0;
	p5.acc << 0, 0, 0;
	p5.yaw = 0;

	p6.pos<< 4, 4, 1.3;
	p6.vel<< 0, 0, 0;
	p6.acc<< 0, 0, 0;
	p6.yaw = 0;

	p7.pos << -4, 4, 1.3;
	p7.vel << 0, 0, 0;
	p7.acc << 0, 0, 0;
	p7.yaw = 0;

	p8.pos << 0, 0, 1.3;
	p8.vel << 0, 0, 0;
	p8.acc << 0, 0, 0;
	p8.yaw = 0;

	p9.pos << 0, 0, 1.3;
	p9.vel << 0, 0, 0;
	p9.acc << 0, 0, 0;
	p9.yaw = 0;

	path.push_back(segments(p1, p2, 3));
	path.push_back(segments(p2, p3, 2));
	path.push_back(segments(p3, p4, 2));
	path.push_back(segments(p4, p5, 2));
	path.push_back(segments(p5, p6, 2));
	path.push_back(segments(p6, p7, 2));
	path.push_back(segments(p7, p8, 3));
	data = plan.get_profile(path, path.size(), sample);

	desired_pose.pose.position.x = 1.2;
	desired_pose.pose.position.y = 0.0;
	desired_pose.pose.position.z = 1.3;

	geometry_msgs::Transform transform;
	geometry_msgs::Twist twist;
	transform.translation.x = 0;
	transform.translation.y = 0;
	transform.translation.z = 0;
	transform.rotation.x = 0;
	transform.rotation.y = 0;
	transform.rotation.z = 0;
	transform.rotation.w = 0;
	twist.linear.x = 0;
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = 0;

	traj.transforms.push_back(transform);
	traj.velocities.push_back(twist);
	traj.accelerations.push_back(twist);

	while(ros::ok()) {

		if(nh.getParam("/start",flag)) {

		};

		if(flag == false || (tick>data.size())) {

			//do position control
			nh.setParam("/start",false);
			tick = 0;
			vir_x = desired_pose.pose.position.x;
			vir_y = desired_pose.pose.position.y;
			vir_z = desired_pose.pose.position.z;
		} else {

			vir_x = data[tick].pos(0);
			vir_y = data[tick].pos(1);
			vir_z = data[tick].pos(2);
			vx = data[tick].vel(0);
			vy = data[tick].vel(1);
			vz = data[tick].vel(2);
			ax = data[tick].acc(0);
			ay = data[tick].acc(1);
			az = data[tick].acc(2);
			tick++;

			double t = ros::Time::now().toSec();
			//std::cout << "time is " << t << std::endl;
#if 0
			vir_x = 0;
			vir_y = 0;
			vir_z = desired_pose.pose.position.z;// + 0.3*sin(1.4*M_PI*t/T);

			vx = 0;
			vy = 0;
			vz = 0;//0.3*cos(1.4*M_PI*t/T)*1.4*M_PI*t/T;

			ax = 0;
			ay = 0;
			az = 0;//-0.3*sin(1.4*M_PI*t/T)*1.4*M_PI/T*1.4*M_PI/T;
#endif
		}

		traj.transforms[0].translation.x = vir_x;
		traj.transforms[0].translation.y = vir_y;
		traj.transforms[0].translation.z = vir_z;
		traj.velocities[0].linear.x = vx;
		traj.velocities[0].linear.y = vy;
		traj.velocities[0].linear.z = vz;
		traj.accelerations[0].linear.x = ax;
		traj.accelerations[0].linear.y = ay;
		traj.accelerations[0].linear.z = az;

		traj2.transforms[0].translation.x = vir_x-1.2;
		traj2.transforms[0].translation.y = vir_y;
		traj2.transforms[0].translation.z = vir_z;
		traj2.velocities[0].linear.x = vx;
		traj2.velocities[0].linear.y = vy;
		traj2.velocities[0].linear.z = vz;
		traj2.accelerations[0].linear.x = ax;
		traj2.accelerations[0].linear.y = ay;
		traj2.accelerations[0].linear.z = az;

		traj_pub.publish(traj);
		traj_pub2.publish(traj2);

		ros::spinOnce();
		loop_rate.sleep();
	}

}
