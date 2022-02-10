#!/usr/bin/env python

import rospy
from ukf import UKF
from common_tool import eulerAnglesToRotationMatrix
from numpy.linalg import matrix_rank
import numpy as np
import math
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Imu
from mav_msgs.msg import Actuators
from pyquaternion import Quaternion

#time variables
time_last = 0 
dt = 0.025

#state variables
state_dim = 32
initial_state = np.zeros(state_dim)
e_symbol = 1.0
initial_state[28:32] = np.array([e_symbol, e_symbol, e_symbol, e_symbol])
rpy_tmp = np.zeros(3)
d_rpy = np.zeros(3)
rpy = 0.0,0.0,0.0 #tuple

#measurement variables
measurement_dim = 22
measurement_noise = np.zeros(measurement_dim)
sensor_data = np.zeros(measurement_dim)

#process parameters
thrust_moment = np.zeros(4)
R_iris1 = np.zeros((3,3))
R_imu = np.zeros((3,3))
F = np.zeros(3)
tau = np.zeros(3)
acc_imu = np.zeros(3)
acc_imu_tmp = np.zeros(3)
acc = np.zeros(3)
e3 = np.array([0.0,0.0,1])
get_estimate_state = np.zeros(3)
f_vector = np.zeros(4)

#for publish variables
estimate_state_list = Float64MultiArray()
rpy_list = Float64MultiArray()
debug_list = Float64MultiArray()
debug_tmp = np.zeros(17)

# parameters
m = 1.55
J = np.array([[0.03, 0, 0], [0, 0.05, 0], [0, 0, 0.1]])

# allocation matrix how to check
allo_mat = np.array([
                     [1,   1,   1,   1],
                     [-0.255539*math.sin(1.0371),0.238537*math.sin(0.99439),0.255539*math.sin(1.0371),-0.238537*math.sin(0.99439)],
                     [-0.255539*math.cos(1.0371),0.238537*math.cos(0.99439),-0.255539*math.cos(1.0371),0.238537*math.cos(0.99439)],
                     [-1.6e-2,   -1.6e-2,   1.6e-2,   1.6e-2]    
                     ])

#allo_mat_inv =  np.dot(np.transpose(allo_mat),np.linalg.inv(np.dot(allo_mat,np.transpose(allo_mat))))
#print(allo_mat_inv * thrust_moment)
# Process Noise
q = np.eye(state_dim)
# x,v,a
q[0][0] = 0.0001
q[1][1] = 0.0001
q[2][2] = 0.0001
q[3][3] = 0.01
q[4][4] = 0.01
q[5][5] = 0.1
q[6][6] = 0.0001
q[7][7] = 0.0001
q[8][8] = 0.0001
# O,w,w_dot
q[9][9] = 0.0001 
q[10][10] = 0.0001
q[11][11] = 0.0001
q[12][12] = 1
q[13][13] = 50
q[14][14] = 1
q[15][15] = 0.001
q[16][16] = 0.01
q[17][17] = 0.001
# F
q[18][18] = 0.0001
q[19][19] = 0.0001
q[20][20] = 0.0001
# tau
q[21][21] = 0.0001
q[22][22] = 0.0001
q[23][23] = 0.0001
#f1~f4
q[24][24] = 0.0001
q[25][25] = 0.0001
q[26][26] = 0.0001
q[27][27] = 0.0001
#E
q[28][28] = 0.0001
q[29][29] = 0.0001
q[30][30] = 0.0001
q[31][31] = 0.0001



# create measurement noise covariance matrices
p_yy_noise = np.eye(measurement_dim)
# x,a
p_yy_noise[0][0] = 0.0001
p_yy_noise[1][1] = 0.0001
p_yy_noise[2][2] = 0.0001
p_yy_noise[3][3] = 0.01
p_yy_noise[4][4] = 0.1
p_yy_noise[5][5] = 0.01
# O,w_dot
p_yy_noise[6][6] = 0.0001
p_yy_noise[7][7] = 0.0001
p_yy_noise[8][8] = 0.0001
p_yy_noise[9][9] = 0.09
p_yy_noise[10][10] = 1
p_yy_noise[11][11] = 5
# F
p_yy_noise[12][12] = 0.0001
p_yy_noise[13][13] = 0.0001
p_yy_noise[14][14] = 0.0001
# tau
p_yy_noise[15][15] = 0.01
p_yy_noise[16][16] = 0.001
p_yy_noise[17][17] = 0.001
# f1~f4
p_yy_noise[18][18] = 0.0001
p_yy_noise[19][19] = 0.0001
p_yy_noise[20][20] = 0.0001
p_yy_noise[21][21] = 0.0001


def iterate_x(x, timestep):
    '''this function is based on the x_dot and can be nonlinear as needed'''
    global thrust_moment,R_iris1,R_imu,m,F,e3,acc_imu,acc,debug_tmp,f_vector,allo_mat,allo_mat_inv,rpy
    ret = np.zeros(len(x))
    # dynamics
    a = thrust_moment[0]*np.dot(R_iris1,e3)/m - F/m 

    rpy_state = x[9:12]
    w_state = x[12:15]
    F_state = x[18:21]
    tau_state = x[21:24]
    f_vector_state = x[24:28]
    E_diag = np.diag(x[28:32])
    
    control_input = np.dot(allo_mat,np.dot(E_diag,f_vector_state))


    a_state = control_input[0]*np.dot(eulerAnglesToRotationMatrix(rpy_state),e3)/m - F_state/m   
    w = x[12:15]
    w_dot = np.dot(np.linalg.inv(J),(thrust_moment[1:4]- np.cross(w,np.dot(J,w)) - tau))

    
    w_dot_state = np.dot(np.linalg.inv(J),(control_input[1:4]- np.cross(w_state,np.dot(J,w_state)) - tau_state))

    acc = np.dot(R_imu,acc_imu)

    sensor_data[3:6] = a_state
    sensor_data[9:12] = w_dot_state
    #print(a_state)

    # x,v,a
    ret[0] = x[0] + x[3] * timestep 
    ret[1] = x[1] + x[4] * timestep 
    ret[2] = x[2] + x[5] * timestep 
    ret[3] = x[3] + x[6] * timestep
    ret[4] = x[4] + x[7] * timestep
    ret[5] = x[5] + (x[8]- 9.81) * timestep
    ret[6] = a_state[0]# x[6]#
    ret[7] = a_state[1]# x[7]# 
    ret[8] = a_state[2]# x[8]#
    # O,w,w_dot
    ret[9] = x[9]   + x[12] * timestep
    ret[10] = x[10] + x[13] * timestep
    ret[11] = x[11] + x[14] * timestep
    ret[12] = x[12] + x[15] * timestep 
    ret[13] = x[13] + x[16] * timestep
    ret[14] = x[14] + x[17] * timestep
    ret[15] = w_dot_state[0]#x[15]#  w_dot[0]#
    ret[16] = w_dot_state[1]#x[16]#  w_dot[1]#
    ret[17] = w_dot_state[2]#x[17]#  w_dot[2]#
    # F
    ret[18] = x[18]
    ret[19] = x[19]   
    ret[20] = x[20]
    # tau
    ret[21] = x[21]
    ret[22] = x[22]   
    ret[23] = x[23]
    # f1~f4
    ret[24] = x[24]
    ret[25] = x[25]   
    ret[26] = x[26]
    ret[27] = x[27]    
    # E
    ret[28] = x[28]
    ret[29] = x[29]
    ret[30] = x[30]
    ret[31] = x[31]

    return ret

def measurement_model(x):
    """
    :param x: states
    """
    global measurement_dim
    # x,a,0,6
    ret = np.zeros(measurement_dim)
    ret[0] = x[0]
    ret[1] = x[1]
    ret[2] = x[2]
    ret[3] = x[6]
    ret[4] = x[7]
    ret[5] = x[8]
    #O,w_dot,9,15
    ret[6] = x[9]
    ret[7] = x[10]
    ret[8] = x[11]
    ret[9] = x[15]
    ret[10] = x[16]
    ret[11] = x[17]
    #F
    ret[12] = x[18]
    ret[13] = x[19]
    ret[14] = x[20]
    #tau
    ret[15] = x[21]
    ret[16] = x[22]
    ret[17] = x[23]
    #f1~f4
    ret[18] = x[24]    
    ret[19] = x[25]    
    ret[20] = x[26]    
    ret[21] = x[27]    
    return ret

def pos_rpy_cb(data):
    # pass the subscribed data
    global sensor_data,rpy,R_iris1,debug_tmp
    quaternion = Quaternion(data.pose.pose.orientation.w, data.pose.pose.orientation.x, 
                      data.pose.pose.orientation.y, data.pose.pose.orientation.z)
    rpy = [quaternion.yaw_pitch_roll[2], quaternion.yaw_pitch_roll[1], quaternion.yaw_pitch_roll[0]]

    sensor_data[0:3] =  np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
    sensor_data[6:9] =  np.array([quaternion.yaw_pitch_roll[2], quaternion.yaw_pitch_roll[1], quaternion.yaw_pitch_roll[0]])
    R_iris1 = quaternion.rotation_matrix

    #print("R_iris1")
    #print(R_iris1)
    #print("test")
    #print(debug_tmp)
    
def thrust_moment_cb(data):
    global thrust_moment
    thrust_moment = np.array([data.pose.pose.orientation.w, data.pose.pose.orientation.x, 
                              data.pose.pose.orientation.y, data.pose.pose.orientation.z])
    #print(thrust_moment)


def ft_cb(data):
    global F,tau
    # Force sensor has bouncing value, so I new the if/else below
    if data.wrench.force.z < 0:
        F = np.array([-data.wrench.force.x, -data.wrench.force.y, -data.wrench.force.z])
    else:
        F = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])

    sensor_data[12:15] = F
    tau = np.array([data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])
    sensor_data[15:18] = tau
    debug_tmp[14:17] = F
    #print(tau)

def imu_cb(data):
    global acc_imu, acc_imu_tmp, R_imu
    acc_imu = np.array([data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z])
   
    quaternion = Quaternion(data.orientation.w, data.orientation.x, 
                            data.orientation.y, data.orientation.z)
    R_imu = quaternion.rotation_matrix

def rotors_cb(data):
    global f_vector,allo_mat,thrust_moment
    rotor_force_constant = 8.54858e-06#8.3e-06#
    f_vector = np.array([data.data[0]*data.data[0]*rotor_force_constant,
                         data.data[1]*data.data[1]*rotor_force_constant,
                         data.data[2]*data.data[2]*rotor_force_constant, 
                         data.data[3]*data.data[3]*rotor_force_constant])
    sensor_data[18:22] = f_vector
    debug_tmp[10:14] = f_vector
    

def debug_cb(data):
    global get_estimate_state
    get_estimate_state = data.data[12:15]

def add_sensor_noise(sensor_data): # call by value
    sensor_data[0:3] += np.random.normal(0,0.002,3)
    sensor_data[3:6] += np.random.normal(0,0.017,3)

def ukf():
    global time_last
    global dt
    #dt = rospy.Time.now().to_sec() - time_last
    ukf_module.predict(dt)
    ukf_module.update(measurement_dim, sensor_data, p_yy_noise)
    time_last = rospy.Time.now().to_sec()


if __name__ == "__main__":
    try:
        rospy.init_node('UKF_iris1')
        state_pub = rospy.Publisher("/estimated_state", Float64MultiArray, queue_size=10)
        rpy_pub = rospy.Publisher("/iris1/rpy", Float64MultiArray, queue_size=10)
        debug_pub = rospy.Publisher("/debug", Float64MultiArray, queue_size=10) 
        rospy.Subscriber("/iris1/ground_truth/odometry", Odometry, pos_rpy_cb, queue_size=10)
        rospy.Subscriber("/iris1_control_input", Odometry, thrust_moment_cb, queue_size=10)
        rospy.Subscriber("/payload_joint1_ft_sensor", WrenchStamped, ft_cb, queue_size=10)
        rospy.Subscriber("/iris1/ground_truth/imu", Imu, imu_cb, queue_size=10)
        rospy.Subscriber("/estimated_state", Float64MultiArray, debug_cb, queue_size=10)
        rospy.Subscriber("/iris1_original_rotor_vel", Float64MultiArray, rotors_cb, queue_size=10)
        #add_sensor_noise(sensor_data)

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

            debug_list.data = list(debug_tmp)
            debug_pub.publish(debug_list)
            #print(ukf_module.get_covar())

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
