#!/usr/bin/env python

import rospy
from ukf import UKF
import numpy as np
#from math import sin,cos,sqrt,atan2
import math
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from pyquaternion import Quaternion


sensor_data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
time_last = 0
measurement_noise = np.array([0.0,0.0,0.0])
rpy = 0.0,0.0,0.0 #tuple
estimate_state_list = Float64MultiArray()
rpy_list = Float64MultiArray()

# Process Noise
q = np.eye(12)
# x,v
q[0][0] = 0.01 
q[1][1] = 0.01
q[2][2] = 0.01
q[3][3] = 0.001
q[4][4] = 0.001
q[5][5] = 0.001
# O,w
q[6][6] = 0.001 
q[7][7] = 0.001
q[8][8] = 0.001
q[9][9] = 0.0001
q[10][10] = 0.0001
q[11][11] = 0.0001

# create measurement noise covariance matrices
p_yy_noise = np.eye(6)
p_yy_noise[0][0] = 0.01
p_yy_noise[1][1] = 0.01
p_yy_noise[2][2] = 0.01
p_yy_noise[3][3] = 0.0001
p_yy_noise[4][4] = 0.0001
p_yy_noise[5][5] = 0.0001

# create initial state
initial_state = np.array([0, 0, 1.3, 0, 0, 0, # x,v
                          0, 0, 0, 0, 0, 0])  # O,w


def iterate_x(x, timestep):
    '''this function is based on the x_dot and can be nonlinear as needed'''
    ret = np.zeros(len(x))
    # x,v
    ret[0] = x[0] + x[3] * timestep
    ret[1] = x[1] + x[4] * timestep
    ret[2] = x[2] + x[5] * timestep
    ret[3] = x[3] 
    ret[4] = x[4] 
    ret[5] = x[5] 

    # O,w
    ret[6] = x[6] + x[9] * timestep
    ret[7] = x[7] + x[10] * timestep
    ret[8] = x[8] + x[11] * timestep
    ret[9] = x[9] 
    ret[10] = x[10] 
    ret[11] = x[11] 
    return ret

def measurement_model(x):
    """
    :param x: states
    """
    m_dim = 6
    ret = np.zeros(m_dim)
    ret[0] = x[0]
    ret[1] = x[1]
    ret[2] = x[2]
    ret[3] = x[6]
    ret[4] = x[7]
    ret[5] = x[8]
    return ret

def pos_rpy_cb(data):
    #rospy.loginfo("I get the topic")
    # pass the subscribed data
    global sensor_data    
    global rpy
    #rpy = euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y,
    #                            data.pose.pose.orientation.z, data.pose.pose.orientation.w)

    test = Quaternion(data.pose.pose.orientation.w, data.pose.pose.orientation.x, 
                      data.pose.pose.orientation.y, data.pose.pose.orientation.z)
    rpy = [test.yaw_pitch_roll[2], test.yaw_pitch_roll[1], test.yaw_pitch_roll[0]]

    sensor_data =  np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z,
                             test.yaw_pitch_roll[2], test.yaw_pitch_roll[1], test.yaw_pitch_roll[0]])

def add_sensor_noise(sensor_data):
    sensor_data[0:3] += np.random.normal(0,0.1,3)
    #sensor_data[3:6] += np.random.normal(0,0.001,3)

def ukf():
    global time_last
    dt = rospy.Time.now().to_sec() - time_last
    ukf_module.predict(dt)
    ukf_module.update(6, sensor_data, p_yy_noise)
    time_last = rospy.Time.now().to_sec()

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

if __name__ == "__main__":
    try:
        rospy.init_node('UKF')
        state_pub = rospy.Publisher("/estimated_state", Float64MultiArray, queue_size=10)
        rpy_pub = rospy.Publisher("/payload/rpy", Float64MultiArray, queue_size=10)
        rospy.Subscriber("/payload/odometry", Odometry, pos_rpy_cb, queue_size=10)
        add_sensor_noise(sensor_data)

        # pass all the parameters into the UKF!
        # number of state variables, process noise, initial state, initial coariance, three tuning paramters, and the iterate function
        #def __init__(self, num_states, process_noise, initial_state, initial_covar, alpha, k, beta, iterate_function, measurement_model):
        ukf_module = UKF(12, q, initial_state, 0.01*np.eye(12), 0.001, 0.0, 2.0, iterate_x, measurement_model)
        rate = rospy.Rate(40)
        h=15
        while not rospy.is_shutdown():         
            ukf()
            estimate_state = ukf_module.get_state()
            estimate_state_list.data = list(estimate_state)
            state_pub.publish(estimate_state_list)

            rpy_list.data = list(rpy)
            rpy_pub.publish(rpy_list)

            #print(ukf_module.get_covar())
            h+=1
            if h==10:
                break
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
