#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
import sympy as sp


#  forward and inverse kinematics classes
class InverseVelocityKinematics():
    def __init__(self):
        q1, q2, q3, q4, q5, q6 = sp.symbols('q1 q2 q3 q4 q5 q6')
        self.q = [q1, q2, q3, q4, q5, q6]
        self.J_sym = self._compute_jacobian_symbolic()

    def _compute_jacobian_symbolic(self):
        q1, q2, q3, q4, q5, q6 = self.q

        x = (
            0.1 * sp.sin(q1) * sp.cos(q5)
            + 0.127 * sp.sin(q1)
            - 0.1 * sp.sin(q5) * sp.cos(q1) * sp.cos(q2+q3+q4)
            + 0.1 * sp.sin(q2+q3+q4) * sp.cos(q1)
            - 0.425 * sp.cos(q1) * sp.cos(q2)
            - 0.392 * sp.cos(q1) * sp.cos(q2+q3)
        )

        y = (
            -0.1 * sp.sin(q1) * sp.sin(q5) * sp.cos(q2+q3+q4)
            + 0.1 * sp.sin(q1) * sp.sin(q2+q3+q4)
            - 0.425 * sp.sin(q1) * sp.cos(q2)
            - 0.392 * sp.sin(q1) * sp.cos(q2+q3)
            - 0.1 * sp.cos(q1) * sp.cos(q5)
            - 0.127 * sp.cos(q1)
        )

        z = (
            -0.425 * sp.sin(q2)
            - 0.392 * sp.sin(q2+q3)
            - 0.1 * sp.sin(q5) * sp.sin(q2+q3+q4)
            - 0.1 * sp.cos(q2+q3+q4)
            + 0.163
        )

        p = [x, y, z]
        return sp.Matrix(p).jacobian(self.q)

    def _compute_jacobian_numeric(self, q_pos):
        subs = {self.q[i]: q_pos[i] for i in range(6)}
        return np.array(self.J_sym.evalf(subs=subs), dtype=np.float64)

    def get_inverse_velocity(self, ee_vel, q_pos):
        J = self._compute_jacobian_numeric(q_pos)
        J_pinv = np.linalg.pinv(J)
        return J_pinv @ ee_vel


class ForwardPositionKinematics():
    def __init__(self):
        self.dh_params_a = [0, -0.425, -0.392, 0, 0, 0]
        self.dh_params_d = [0.163, 0, 0, 0.127, 0.1, 0.1]

    def _dh_row(self, theta, d, a, alpha):
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0            ,  np.sin(alpha)              ,  np.cos(alpha)              , d             ],
            [0            ,  0                          ,  0                         , 1             ]
        ])

    def get_ee_position(self, q):
        pi = np.pi
        dh_matrices = [
            self._dh_row(q[0], self.dh_params_d[0], self.dh_params_a[0], pi/2),
            self._dh_row(q[1], self.dh_params_d[1], self.dh_params_a[1], 0),
            self._dh_row(q[2], self.dh_params_d[2], self.dh_params_a[2], 0),
            self._dh_row(q[3], self.dh_params_d[3], self.dh_params_a[3], pi/2),
            self._dh_row(q[4], self.dh_params_d[4], self.dh_params_a[4], -pi/2),
            self._dh_row(q[5], self.dh_params_d[5], self.dh_params_a[5], 0)
        ]

        T = np.eye(4)
        for dh in dh_matrices:
            T = T @ dh

        return np.array([T[0,3], T[1,3], T[2,3]])


# fuzzy logic controller class
class FuzzyLogicController():
    def __init__(self):
        # Rule table (we used the same as in the tutorial because it is generic for any 2D trajectory tracking robot)
        self.rule_table = {
            "LN": {"LN":"LP","MN":"MP","S":"S","MP":"MN","LP":"LN"},
            "MN": {"LN":"LP","MN":"MP","S":"S","MP":"MN","LP":"LN"},
            "S":  {"LN":"LP","MN":"MP","S":"S","MP":"MN","LP":"LN"},
            "MP": {"LN":"LP","MN":"MP","S":"S","MP":"MN","LP":"LN"},
            "LP": {"LN":"LP","MN":"MP","S":"S","MP":"MN","LP":"LN"}
        }
        self.center_map = {"LN":-2, "MN":-1, "S":0, "MP":1, "LP":2}

    # Membership functions ( again ,we used the same as in the tutorial because they are the normalized fuzzy spaces)
    def mf_LN(self, e): return max(0, min(1, (-2 - e) / 2))
    def mf_LP(self, e): return max(0, min(1, (e - 2) / 2))
    def mf_MN(self, e): return max(0, 1 - abs((e + 1)/1))
    def mf_S (self, e): return max(0, 1 - abs(e/1))
    def mf_MP(self, e): return max(0, 1 - abs((e - 1)/1))

    def fuzzify(self, e):
        return {
            "LN": self.mf_LN(e),
            "MN": self.mf_MN(e),
            "S":  self.mf_S(e),
            "MP": self.mf_MP(e),
            "LP": self.mf_LP(e)
        }

    def compute_output(self, Xe, Ye):
        X_sets = self.fuzzify(Xe)
        Y_sets = self.fuzzify(Ye)

        rule_outputs = {}

        for y_label, y_val in Y_sets.items():
            for x_label, x_val in X_sets.items():
                strength = min(x_val, y_val)
                inferred = self.rule_table[y_label][x_label]
                rule_outputs[inferred] = rule_outputs.get(inferred, 0) + strength

        num = sum(rule_outputs[label] * self.center_map[label] for label in rule_outputs)
        den = sum(rule_outputs.values()) if sum(rule_outputs.values()) != 0 else 1

        return num / den


#  main closed-loop node 
class ClosedLoopNode(Node):
    def __init__(self):
        super().__init__('fuzzy_closed_loop_node')

        self.publisher = self.create_publisher(Float64MultiArray, "/joint_commands", 10)
        self.joint_angles = [0.0]*6

        self.sub = self.create_subscription(
            JointState, "/joint_state", self.joint_angle_callback, 10
        )

        self.forward = ForwardPositionKinematics()
        self.inverse = InverseVelocityKinematics()
        self.fuzzy = FuzzyLogicController()

    def joint_angle_callback(self, msg):
        # Drop the 7th joint (box_slide) we don't need it because it's fixed
        self.joint_angles = list(msg.position[:-1])

    def publish_velocity(self, dq):
        msg = Float64MultiArray()
        msg.data = dq.tolist() + [0.0]
        self.publisher.publish(msg)

    # Closed loop circle tracking 
    def follow_circle(self):
        duration = 10
        timestep = 0.002
        t = 0
        start = time.time()

        while t <= duration:
            rclpy.spin_once(self, timeout_sec=0.0)

            xd = 0.2*np.cos(0.4*np.pi*t) + 0.48
            yd = 0.2*np.sin(0.4*np.pi*t) - 0.17
            zd = 0.40

            xa, ya, za = self.forward.get_ee_position(self.joint_angles)

            Xe = xd - xa
            Ye = yd - ya

            v = self.fuzzy.compute_output(Xe, Ye)

            ee_vel = np.array([v, v, 0.0])

            dq = self.inverse.get_inverse_velocity(ee_vel, self.joint_angles)

            self.publish_velocity(dq)

            t += timestep
            sleep = start + t - time.time()
            if sleep > 0:
                time.sleep(sleep)


#  main function to start the node
def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopNode()

    node.get_logger().info("Starting closed-loop fuzzy circle tracking...")
    time.sleep(1.0)

    node.follow_circle()

    node.get_logger().info("Finished.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
