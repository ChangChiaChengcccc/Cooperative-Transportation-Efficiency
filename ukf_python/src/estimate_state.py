#!/usr/bin/env python

import rospy
from ukf import UKF
import numpy as np
from math import sin,cos,sqrt,atan2
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry


sensor_data = np.array([0,0,0])
time_last = 0
measurement_noise = np.array([0.0,0.0,0.0])
estimate_state_F = Float64MultiArray()
# Process Noise
q = np.eye(6)
q[0][0] = 0.0001 
q[1][1] = 0.0001
q[2][2] = 0.0001
q[3][3] = 0.0001
q[4][4] = 0.0001
q[5][5] = 0.0001

# create measurement noise covariance matrices
p_yy_noise = np.eye(3)
p_yy_noise[0][0] = 0.01
p_yy_noise[1][1] = 0.01
p_yy_noise[2][2] = 0.01

# create initial state
initial_state = np.array([0,0,1.3,0,0,0])


def iterate_x(x, timestep):
    '''this function is based on the x_dot and can be nonlinear as needed'''
    ret = np.zeros(len(x))
    ret[0] = x[0] + x[3] * timestep
    ret[1] = x[1] + x[4] * timestep
    ret[2] = x[2] + x[5] * timestep
    ret[3] = x[3] 
    ret[4] = x[4] 
    ret[5] = x[5] 
    return ret

def measurement_model(x):
    """
    :param x: states
    """
    m_dim = 3
    ret = np.zeros(m_dim)
    ret[0] = x[0]
    ret[1] = x[1]
    ret[2] = x[2]
    return ret

def callback(data):
    #rospy.loginfo("I get the topic")
    # pass the subscribed data
    global sensor_data
    sensor_data =  np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
    #print(sensor_data)
    sensor_data += np.random.normal(0,0.1,3)

    

def ukf():
    global time_last
    dt = rospy.Time.now().to_sec() - time_last
    ukf_module.predict(dt)
    ukf_module.update(3, sensor_data, p_yy_noise)
    time_last = rospy.Time.now().to_sec()

if __name__ == "__main__":
    try:
        rospy.init_node('UKF')
        state_pub = rospy.Publisher("/estimated_state", Float64MultiArray, queue_size=10)

        # pass all the parameters into the UKF!
        # number of state variables, process noise, initial state, initial coariance, three tuning paramters, and the iterate function
        #def __init__(self, num_states, process_noise, initial_state, initial_covar, alpha, k, beta, iterate_function, measurement_model):
        ukf_module = UKF(6, q, initial_state, 0.01*np.eye(6), 0.001, 0.0, 2.0, iterate_x, measurement_model)
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():

            rospy.Subscriber("/payload/odometry", Odometry, callback, queue_size=10)
            ukf()
            estimate_state = ukf_module.get_state()
            estimate_state_F.data = list(estimate_state)
            state_pub.publish(estimate_state_F)
#            print "Estimated state: ", ukf_module.get_state()
#            print "Covariance: ", np.linalg.det(ukf_module.get_covar())
#            position_covar = ukf_module.get_covar()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
