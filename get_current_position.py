import rclpy
from sensor_msgs.msg import JointState
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sympy as sp


class GetPosition(Node):
    def __init__(self):
        super().__init__('current_position')
        self.pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.joint_angles = [0.0] * 6
        # callback function is called every time the node "spins"
        self.sub = self.create_subscription(
                JointState,
                "/joint_state",
                self.joint_angle_callback,
                10
                )

        self.dh_params_a = [0.0, -0.425, -0.392, 0.0, 0.0, 0.0]
        self.dh_params_d = [0.163, 0, 0, 0.127, 0.1, 0.1]


    def dh_row(self, theta, d, a, alpha):
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0            ,  np.sin(alpha)              ,  np.cos(alpha)              , d             ],
            [0            ,  0                          ,  0                         , 1             ]
        ])

    def compute_fk(self, q):
        pi = np.pi
        dh_matrices = [
            self.dh_row(q[0], self.dh_params_d[0], self.dh_params_a[0], pi/2),
            self.dh_row(q[1], self.dh_params_d[1], self.dh_params_a[1], 0),
            self.dh_row(q[2], self.dh_params_d[2], self.dh_params_a[2], 0),
            self.dh_row(q[3], self.dh_params_d[3], self.dh_params_a[3], pi/2),
            self.dh_row(q[4], self.dh_params_d[4], self.dh_params_a[4], -pi/2),
            self.dh_row(q[5], self.dh_params_d[5], self.dh_params_a[5], 0)
        ]

        result = np.eye(4)
        for dh in dh_matrices:
            result = np.dot(result, dh)

        return (np.round(result,4))


    def joint_angle_callback(self, msg):
        self.joint_angles = list(msg.position)
        current_fk = self.compute_fk(self.joint_angles)
        x = current_fk[0, 3]
        y = current_fk[1, 3]
        z = current_fk[2, 3]
        print(f"X: {x}")
        print(f"Y: {y}")
        print(f"Z: {z}")


def main(args=None):
    rclpy.init(args=args)
    node = GetPosition()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

