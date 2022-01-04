import numpy as np
import scipy.linalg
#initial_state = np.array([0, 0, 1.3, 0, 0, 0, # x,v
#                          0, 0, 0, 0, 0, 0])  # O,w
#print(initial_state)

a = np.array([[4,0,0],[0,4,0],[0,0,4]])
print(a)
b = np.linalg.cholesky(a)
print(b)
