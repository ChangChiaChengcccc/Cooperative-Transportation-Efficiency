import math
import numpy as np

# Calculates Rotation Matrix given euler angles.
def eulerAnglesToRotationMatrix(theta) :

    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])

    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])

    R_z1 = np.array([[math.cos(theta[0]),    -math.sin(theta[0]),    0],
                    [math.sin(theta[0]),    math.cos(theta[0]),     0],
                    [0,                     0,                      1]
                    ])

    R_z2 = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])

    #ZXY

    R = np.dot(R_x, np.dot( R_y, R_z2 ))

    return R