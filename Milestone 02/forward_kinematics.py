#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
np.set_printoptions(suppress=True, precision=4)

class UR5eController(Node):
    def __init__(self, joint_angles):
        super().__init__('ur5e_controller')
        self.publisher = self.create_publisher(Float64MultiArray, '/joint_commands', 10)

        self.joint_angles = [float(x) for x in joint_angles]
        self.dh_params_a = [0, -0.425, -0.392, 0, 0, 0]
        self.dh_params_d = [0.163, 0, 0, 0.127, 0.1, 0.1]

        fk_matrix = self.compute_fk(self.joint_angles)
        print("\nForward Kinematics (Transformation Matrix):\n", fk_matrix)

        self.publish_joint_angles(self.joint_angles)

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
        result[0, 3] *= -1
        result[1, 3] *= -1

        return (np.round(result,4))

    def publish_joint_angles(self, q):
        msg = Float64MultiArray()
        msg.data = q
        self.publisher.publish(msg)
        self.get_logger().info(f"Published joint angles: {q}")

def main(args=None):
    rclpy.init(args=args)

    try:
        while True:
            user_input = input("Enter 6 joint angles (space-separated, or 'q' to quit): ").strip()
            if user_input.lower() == 'q':
                print("Exiting...")
                break

            q = user_input.split()
            if len(q) != 6:
                print("Please enter exactly 6 joint angles!")
                continue

            q_floats = [float(angle) for angle in q]

            node = UR5eController(joint_angles=q_floats)
            rclpy.spin_once(node, timeout_sec=1)
            node.destroy_node()

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

