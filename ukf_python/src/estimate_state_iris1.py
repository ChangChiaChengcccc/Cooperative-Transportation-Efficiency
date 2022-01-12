#!/usr/bin/env python

import rospy
from ukf import UKF
import numpy as np
import math
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Imu
from pyquaternion import Quaternion

#time variables
time_last = 0 
dt = 0.0

#state variables
state_dim = 12
rpy_tmp = np.array([0.0, 0.0, 0.0])
d_rpy = np.array([0.0, 0.0, 0.0])
rpy = 0.0,0.0,0.0 #tuple

#measurement variables
measurement_dim = 6
measurement_noise = np.zeros(6)
sensor_data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
initial_state = np.array([0.6, 0, 1.3, 0, 0, 0, # x,v
                          0, 0, 0, 0, 0, 0]) # O,w

#process parameters
thrust_moment = np.array([0.0,0.0,0.0,0.0])
R_iris1 = np.zeros((3,3))
R_imu = np.zeros((3,3))
F = np.zeros(3)
tau = np.zeros(3)
imu_acc = np.zeros(3)
imu_acc_tmp = np.zeros(3)
acc = np.zeros(3)
e3 = np.array([0.0,0.0,1])

#for publish variables
estimate_state_list = Float64MultiArray()
rpy_list = Float64MultiArray()
debug_list = Float64MultiArray()

# parameters
m = 1.55
J = np.array([[0.03, 0, 0], [0, 0.05, 0], [0, 0, 0.1]])

# Process Noise
q = np.eye(state_dim)
# x,v
q[0][0] = 0.0001 
q[1][1] = 0.0001
q[2][2] = 0.0001
q[3][3] = 0.1
q[4][4] = 0.001
q[5][5] = 0.001
# O,w
q[6][6] = 0.0001 
q[7][7] = 0.0001
q[8][8] = 0.0001
q[9][9] = 0.001
q[10][10] = 0.001
q[11][11] = 0.001

# create measurement noise covariance matrices
p_yy_noise = np.eye(measurement_dim)
p_yy_noise[0][0] = 0.0001
p_yy_noise[1][1] = 0.0001
p_yy_noise[2][2] = 0.0001
p_yy_noise[3][3] = 0.0001
p_yy_noise[4][4] = 0.0001
p_yy_noise[5][5] = 0.0001

def iterate_x(x, timestep):
    '''this function is based on the x_dot and can be nonlinear as needed'''
    global thrust_moment,R_iris1,R_imu,m,F,e3,imu_acc,acc
    ret = np.zeros(len(x))
    # dynamics
    a = thrust_moment[0]*np.dot(R_iris1,e3)/m - 9.81*e3 - F/m
    w = np.array([x[9],x[10],x[11]])
    w_dot = np.dot(np.linalg.inv(J),(thrust_moment[1:4]- np.cross(w,np.dot(J,w)) - tau))
    acc = np.dot(R_imu,imu_acc)
    #print(acc)
    # x,v
    ret[0] = x[0] + x[3] * timestep
    ret[1] = x[1] + x[4] * timestep
    ret[2] = x[2] + x[5] * timestep
    ret[3] = x[3] + a[0] * timestep #+ a[0] * timestep
    ret[4] = x[4] #+ imu_acc[1] * timestep#+ a[1] * timestep
    ret[5] = x[5] #+ imu_acc[2] * timestep#+ a[2] * timestep
    # O,w
    ret[6] = x[6]   + x[9]  * timestep
    ret[7] = x[7]   + x[10] * timestep
    ret[8] = x[8]   + x[11] * timestep
    ret[9] = x[9]   #+ w_dot[0] * timestep
    ret[10] = x[10] #+ w_dot[1] * timestep
    ret[11] = x[11] #+ w_dot[2] * timestep
    return ret

def measurement_model(x):
    """
    :param x: states
    """
    global measurement_dim
    ret = np.zeros(measurement_dim)
    ret[0] = x[0]
    ret[1] = x[1]
    ret[2] = x[2]
    ret[3] = x[6]
    ret[4] = x[7]
    ret[5] = x[8]
    return ret

def pos_rpy_cb(data):
    # pass the subscribed data
    global sensor_data    
    global rpy,R_iris1

    quaternion = Quaternion(data.pose.pose.orientation.w, data.pose.pose.orientation.x, 
                      data.pose.pose.orientation.y, data.pose.pose.orientation.z)
    rpy = [quaternion.yaw_pitch_roll[2], quaternion.yaw_pitch_roll[1], quaternion.yaw_pitch_roll[0]]

    sensor_data =  np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z,
                             quaternion.yaw_pitch_roll[2], quaternion.yaw_pitch_roll[1], quaternion.yaw_pitch_roll[0]])
    R_iris1 = quaternion.rotation_matrix

def thrust_moment_cb(data):
    global thrust_moment
    thrust_moment = np.array([data.pose.pose.orientation.w, data.pose.pose.orientation.x, 
                              data.pose.pose.orientation.y, data.pose.pose.orientation.z])

def ft_cb(data):
    global F,tau
    F = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])
    tau = np.array([data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])

def imu_cb(data):
    global imu_acc,imu_acc_tmp,R_imu
    imu_acc = np.array([data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z])
    imu_acc = 0.97*imu_acc_tmp + 0.03*imu_acc  
    imu_acc_tmp = imu_acc

    quaternion = Quaternion(data.orientation.w, data.orientation.x, 
                            data.orientation.y, data.orientation.z)
    R_imu = quaternion.rotation_matrix

def add_sensor_noise(sensor_data):
    sensor_data[0:3] += np.random.normal(0,0.002,3)
    sensor_data[3:6] += np.random.normal(0,0.017,3)

def ukf():
    global time_last
    global dt
    dt = rospy.Time.now().to_sec() - time_last
    ukf_module.predict(dt)
    ukf_module.update(measurement_dim, sensor_data, p_yy_noise)
    time_last = rospy.Time.now().to_sec()


if __name__ == "__main__":
    try:
        rospy.init_node('UKF')
        state_pub = rospy.Publisher("/estimated_state", Float64MultiArray, queue_size=10)
        rpy_pub = rospy.Publisher("/iris1/rpy", Float64MultiArray, queue_size=10)
        debug_pub = rospy.Publisher("/debug", Float64MultiArray, queue_size=10)
        rospy.Subscriber("/iris1/ground_truth/odometry", Odometry, pos_rpy_cb, queue_size=10)
        rospy.Subscriber("/iris1_control_input", Odometry, thrust_moment_cb, queue_size=10)
        rospy.Subscriber("/payload_joint1_ft_sensor", WrenchStamped, ft_cb, queue_size=10)
        rospy.Subscriber("/iris1/ground_truth/imu", Imu, imu_cb, queue_size=10)
        add_sensor_noise(sensor_data)

        # pass all the parameters into the UKF!
        # number of state variables, process noise, initial state, initial coariance, three tuning paramters, and the iterate function
        #def __init__(self, num_states, process_noise, initial_state, initial_covar, alpha, k, beta, iterate_function, measurement_model):
        ukf_module = UKF(state_dim, q, initial_state, 0.01*np.eye(state_dim), 0.001, 0.0, 2.0, iterate_x, measurement_model)
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():         
            ukf()
            estimate_state = ukf_module.get_state()
            estimate_state_list.data = list(estimate_state)
            state_pub.publish(estimate_state_list)

            rpy_list.data = list(rpy)
            rpy_pub.publish(rpy_list)

            debug_list.data = list(acc)
            debug_pub.publish(debug_list)
            #print(ukf_module.get_covar())

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
