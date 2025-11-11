import rclpy
from sensor_msgs.msg import JointState
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sympy as sp


class ForwardVelocityPublisher(Node):
    def __init__(self):
        super().__init__('forward_velocity_publisher')
        self.pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.joint_angles = [0.0] * 6
        # callback function is called every time the node "spins"
        self.sub = self.create_subscription(
                JointState,
                "/joint_state",
                self.joint_angle_callback,
                10
                )
        q1, q2, q3, q4, q5, q6 = sp.symbols('q1 q2 q3 q4 q5 q6')
        self.q = [q1, q2, q3, q4, q5, q6]
        self.J_sym = self.compute_jacobian_symbolic()

    def joint_angle_callback(self, msg):
        self.joint_angles = list(msg.position)

    def compute_jacobian_symbolic(self):
        q1, q2, q3, q4, q5, q6 = self.q
        x = (
            0.1 * sp.sin(q1) * sp.cos(q5)
            + 0.127 * sp.sin(q1)
            - 0.1 * sp.sin(q5) * sp.cos(q1) * sp.cos(q2 + q3 + q4)
            + 0.1 * sp.sin(q2 + q3 + q4) * sp.cos(q1)
            - 0.425 * sp.cos(q1) * sp.cos(q2)
            - 0.392 * sp.cos(q1) * sp.cos(q2 + q3)
        )

        y = (
            -0.1 * sp.sin(q1) * sp.sin(q5) * sp.cos(q2 + q3 + q4)
            + 0.1 * sp.sin(q1) * sp.sin(q2 + q3 + q4)
            - 0.425 * sp.sin(q1) * sp.cos(q2)
            - 0.392 * sp.sin(q1) * sp.cos(q2 + q3)
            - 0.1 * sp.cos(q1) * sp.cos(q5)
            - 0.127 * sp.cos(q1)
        )

        z = (
            -0.425 * sp.sin(q2)
            - 0.1 * sp.sin(q5) * sp.sin(q2 + q3 + q4)
            - 0.392 * sp.sin(q2 + q3)
            - 0.1 * sp.cos(q2 + q3 + q4)
            + 0.163
        )

        p = [x, y, z]
        J = sp.Matrix(p).jacobian(self.q)
        return J

    def compute_jacobian_numeric(self):
        subs = {self.q[i]: self.joint_angles[i] for i in range(6)}
        J_num = np.array(self.J_sym.evalf(subs=subs), dtype=np.float64)
        return J_num


    def compute_forward_velocity(self, J, dq):
        velocity_ee = J @ dq
        return velocity_ee

    def publish_velocity(self, dq):
        msg = Float64MultiArray()
        msg.data = dq + [0.0]
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing joint velocities: {dq}')


def main(args=None):
    rclpy.init(args=args)
    node = ForwardVelocityPublisher()

    try:
        while True:
            user_input = input("\nEnter 6 joint velocities rad/s (space-separated), or 'q' to quit:\n> ")
            if user_input.lower() == 'q':
                print("Extiing...")
                break

            dq = user_input.split()
            if len(dq) != 6:
                print("Please enter exactly 6 joint velocities!")
                continue

            dq_floats = [float(velocity) for velocity in dq]

            J_num = node.compute_jacobian_numeric()
            velocity_ee = node.compute_forward_velocity(J_num, dq_floats)
            print("End-effector velocity is: ", velocity_ee.flatten())
            node.publish_velocity(dq_floats)
            rclpy.spin_once(node, timeout_sec=0.1)


    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
