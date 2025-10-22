import numpy as np
import numpy.typing as npt
np.set_printoptions(suppress=True)
pi = np.pi

a = [0    , -0.425, -0.392 , 0    , 0  , 0]
d = [0.163, 0,  0          , 0.127, 0.1, 0.1]
q = [0, 0, 0.0, 0, 0, 0]


def dh_row(theta: float, d: float, a: float, alpha: float) -> npt.NDArray[np.float64]:
    return np.array([[ np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                       [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                       [0            ,  np.sin(alpha)                ,  np.cos(alpha)                , d                ],
                       [0            ,  0                            , 0                             , 1                ]
        ])

dh1 = (dh_row(q[0], d[0], a[0], pi/2))
dh2 = (dh_row(q[1], d[1], a[1], 0))
dh3 = (dh_row(q[2], d[2], a[2], 0))
dh4 = (dh_row(q[3], d[3], a[3], pi/2))
dh5 = (dh_row(q[4], d[4], a[4], -pi/2))
dh6 = (dh_row(q[5], d[5], a[5], 0))

result = np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])
dhTable = [dh1, dh2, dh3, dh4, dh5, dh6]
for dh in dhTable:
    result = np.dot(result, dh)
print(result)
