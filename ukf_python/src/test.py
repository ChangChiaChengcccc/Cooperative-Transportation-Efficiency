import numpy as np
from pyquaternion import Quaternion
J = np.array([[0.03, 0, 0], [0, 0.05, 0], [0, 0, 0.1]])
a = np.array([7,8,9])
print(J/2)
print(J[1][1])