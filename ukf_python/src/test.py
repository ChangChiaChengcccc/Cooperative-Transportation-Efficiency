import math
import numpy as np
from common_tool import eulerAnglesToRotationMatrix 
from pyquaternion import Quaternion


J = np.array([[1, 1, 1], [2, 2, 2], [3, 3, 3]])
a = np.array([7,8,9])

R_x = np.array([[1,         0,                  0                   ],
                [0,         math.cos(np.pi/4), -math.sin(np.pi/4) ],
                [0,         math.sin(np.pi/4), math.cos(np.pi/4)  ]
                ])

R_y = np.array([[math.cos(np.pi/4),    0,      math.sin(np.pi/4)  ],
                [0,                     1,      0                   ],
                [-math.sin(np.pi/4),   0,      math.cos(np.pi/4)  ]
                ])


R_z = np.array([[math.cos(np.pi/4),    -math.sin(np.pi/4),    0],
                [math.sin(np.pi/4),    math.cos(np.pi/4),     0],
                [0,                     0,                      1]
                ])

R = np.dot(R_x, np.dot( R_y, R_z))

print(R)