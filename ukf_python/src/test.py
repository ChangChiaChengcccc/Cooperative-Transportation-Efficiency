import numpy as np
import scipy.linalg
from pyquaternion import Quaternion
#initial_state = np.array([0, 0, 1.3, 0, 0, 0, # x,v
#                          0, 0, 0, 0, 0, 0])  # O,w
#print(initial_state)
'''
a = np.array([[4,0,0],[0,4,0],[0,0,4]])
print(a)
b = np.linalg.cholesky(a)
print(b)
'''
tmp = Quaternion(0,1,0,0)
print('yaw_pitch_roll:')
print(tmp.yaw_pitch_roll)