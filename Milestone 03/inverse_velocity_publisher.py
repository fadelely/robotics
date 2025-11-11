import rclpy
from sensor_msgs.msg import JointState
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sympy as sp


class InverseVelocityPublisher(Node):
    
    def __init__(self):
        super().__init__('inverse_velocity_publisher')
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


    def compute_inverse_velocity(self, J, ee):
        J_pinv = np.linalg.pinv(J)
        dq_solution = J_pinv @ ee
        return dq_solution
    
    def publish_velocity(self, dq):
        msg = Float64MultiArray()
        msg.data = dq + [0.0]
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing joint velocities: {dq}')
        
def main(args=None):
    rclpy.init(args=args)
    node = InverseVelocityPublisher()

    try:
        while True:
            v_input = input("Enter desired end-effector velocities vx vy vz (m/s, space separated): ")
           

            ee = v_input.split()
            if len(ee) != 3:
                print("Please enter exactly 3 ee velocities!")
                continue

            ee_floats = [float(velocity) for velocity in ee]

            J_num = node.compute_jacobian_numeric()
            velocity_dq = node.compute_inverse_velocity(J_num, ee_floats)
            print("dq velocities are : ", velocity_dq.flatten())
            dq_floats = [float(velocity) for velocity in velocity_dq]
            node.publish_velocity(dq_floats)
            rclpy.spin_once(node, timeout_sec=0.1)


    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
