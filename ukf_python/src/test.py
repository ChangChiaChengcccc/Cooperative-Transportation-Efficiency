from math import degrees
import numpy as np
from common_tool import eulerAnglesToRotationMatrix 
from pyquaternion import Quaternion


J = np.array([[0.03, 0, 0], [0, 0.05, 0], [0, 0, 0.1]])
a = np.array([7,8,9])
rpy = np.array([-np.pi/2,0,-np.pi/2])
u = np.array([1,0,0])
print(np.dot(eulerAnglesToRotationMatrix(rpy),u))