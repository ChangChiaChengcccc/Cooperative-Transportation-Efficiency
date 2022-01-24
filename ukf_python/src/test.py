from math import degrees
import numpy as np
from common_tool import eulerAnglesToRotationMatrix 
from pyquaternion import Quaternion


J = np.array([[1, 1, 1], [2, 2, 2], [3, 3, 3]])
a = np.array([7,8,9])

print(np.dot(J,a))