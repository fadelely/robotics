#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
import sympy as sp
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import math


# Forward and Inverse Kinematics Classes
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


# Fuzzy Logic Controller Class
class FuzzyLogicController():
    def __init__(self):
        self.POS_MAX =  2 * np.pi      
        self.POS_MIN = -2 * np.pi     
        self.VEL_MAX =  np.pi        
        self.VEL_MIN = -np.pi

        # Fuzzy universe for the input (error)
        self.error = ctrl.Antecedent(
            np.linspace(self.POS_MIN, self.POS_MAX, 300), 'error'
        )

        # Output velocity universe
        self.velocity = ctrl.Consequent(
            np.linspace(self.VEL_MIN, self.VEL_MAX, 300), 'velocity'
        )

        self._define_memberships()
        self._define_rules()

        self.system = ctrl.ControlSystem(self.rules)
        self.sim = ctrl.ControlSystemSimulation(self.system)


    def _define_memberships(self):
        e = self.error
        v = self.velocity

        # ----- Error fuzzy sets -----
        e['NL'] = fuzz.trimf(e.universe, [self.POS_MIN, -3*np.pi/4, -np.pi/2])
        e['NS'] = fuzz.trimf(e.universe, [-2*np.pi/3, -np.pi/2, 0])
        e['Z']  = fuzz.trimf(e.universe, [-0.1, 0, 0.1])
        e['PS'] = fuzz.trimf(e.universe, [0, np.pi/2, 2*np.pi/3])
        e['PL'] = fuzz.trimf(e.universe, [np.pi/2, 3*np.pi/4, self.POS_MAX])

        # ----- Velocity fuzzy sets -----
        v['NF'] = fuzz.trimf(v.universe, [self.VEL_MIN, -self.VEL_MAX*0.75, -self.VEL_MAX/2])
        v['NS'] = fuzz.trimf(v.universe, [self.VEL_MIN, -self.VEL_MAX/2, 0])
        v['Z']  = fuzz.trimf(v.universe,   [-0.1, 0, 0.1])
        v['PS'] = fuzz.trimf(v.universe, [0, self.VEL_MAX/2, self.VEL_MAX])
        v['PF'] = fuzz.trimf(v.universe, [self.VEL_MAX/2, self.VEL_MAX*0.75, self.VEL_MAX])


    def _define_rules(self):
        e = self.error
        v = self.velocity

        self.rules = [
            ctrl.Rule(e['NL'], v['NF']),
            ctrl.Rule(e['NS'], v['NS']),
            ctrl.Rule(e['Z'],  v['Z']),
            ctrl.Rule(e['PS'], v['PS']),
            ctrl.Rule(e['PL'], v['PF']),
        ]

    def compute(self, error_value):
        self.sim.input['error'] = error_value
        self.sim.compute()
        return self.sim.output['velocity']



class ClosedLoopNode(Node):
    def __init__(self):
        super().__init__('fuzzy_closed_loop_node')

        self.publisher = self.create_publisher(Float64MultiArray, "/joint_commands", 10)
        self.joint_angles = [0.0]*6
        self.joint_received = False

        self.sub = self.create_subscription(
            JointState, "/joint_state", self.joint_angle_callback, 10
        )

        self.forward = ForwardPositionKinematics()
        self.inverse = InverseVelocityKinematics()
        self.fuzzy = FuzzyLogicController()

    def joint_angle_callback(self, msg):
        # Drop the 7th joint (box_slide) - we don't need it
        self.joint_angles = list(msg.position[:-1])
        self.joint_received = True

    def publish_velocity(self, dq):
        msg = Float64MultiArray()
        msg.data = dq.tolist() + [0.0]
        self.publisher.publish(msg)

    def daw2on_lama3_wasat_el_madena(self, duration, timestep):
        time_array = np.arange(-21, 21 + timestep, timestep)
        x_array = np.zeros_like(time_array)
        y_array = np.zeros_like(time_array)
        z_array = np.full_like(time_array, 0.4)
        for i, t_curr in enumerate(time_array):
            t_abs = abs(t_curr)

            if t_curr != 0:
                x_array[i] = (t_abs / t_curr) * (
                    (1/2) * t_abs
                    - (1/4) * abs(t_abs - 1)
                    + (1/2) * abs(t_abs - 3)
                    - (3/4) * abs(t_abs - 5)
                    - (3/2) * abs(t_abs - 13)
                    + (1/4) * abs(t_abs - 17)
                    + (5/4) * abs(t_abs - 21)
                    + 7 * np.sin(np.pi / 12 * (abs(t_abs - 5) - abs(t_abs - 8) + 3))
                    + (1/100) * (abs(t_abs - 8) - abs(t_abs - 13) - 5) ** 3
                    + 1.5
                )
            else:
                x_array[i] = 0  # handle t=0 to avoid division by zero

            y_array[i] = (
                (3/4) * abs(t_abs - 1)
                - (3/4) * abs(t_abs - 3)
                - (7/5) * abs(t_abs - 8)
                + (7/5) * abs(t_abs - 13)
                + (7/16) * (abs(t_abs - 3) - abs(t_abs - 5) - 2) ** 2
                + 4 * np.sin(np.pi / 12 * (abs(t_abs - 5) - abs(t_abs - 8) - 3))
                - (5/16) * (abs(t_abs - 13) - abs(t_abs - 17)) ** 2
                - (1/4) * (abs(t_abs - 17) - abs(t_abs - 21) + 2) ** 2
                + 11.5
            )


        x_array = self.scale_array(x_array, 0.29, 0.69)
        y_array = self.scale_array(y_array, -0.17, 0.07)
        return x_array, y_array, z_array

    def scale_array(self, arr, new_min=0.3, new_max=0.7):
        old_min = np.min(arr)
        old_max = np.max(arr)
        return new_min + (arr - old_min) * (new_max - new_min) / (old_max - old_min)


    def cut_batman(self):
        duration = 10
        timestep = 0.01
        x, y, z = self.daw2on_lama3_wasat_el_madena(duration, timestep)
        time_passed = 0
        start_time = time.time()
        N = len(x)
        for i in range(N):
            # spin does alot of things, but mainly its purpose is to activate callback functions
            # the timeout_sec is set to 0.0 to prevent it from blocking execution
            rclpy.spin_once(self, timeout_sec = 0.0)
            x_t = x[i]
            y_t = y[i]
            z_t = z[i]
            expected_pos = np.array([x_t, y_t, z_t])
            actual_pos = self.forward.get_ee_position(self.joint_angles)
            error = expected_pos - actual_pos
            error[2] = 0
            ee_vel = self.fuzzy.compute(error)
            dq = self.inverse.get_inverse_velocity(ee_vel, self.joint_angles)
            self.publish_velocity(dq)

            t_expected = duration * i / (len(x) - 1)
            t_actual = time.time() - start_time
            time_to_wait = t_expected - t_actual
            if time_to_wait > 0:
                time.sleep(time_to_wait)


    # Closed loop circle tracking with fuzzy control
    def follow_circle(self):
        # Circle parameters
        radius = 0.2
        center_x = 0.48
        center_y = -0.17
        z_height = 0.40
        omega = 0.4 * np.pi  # Angular velocity (rad/s)
        
        duration = 10  # seconds
        timestep = 0.002
        t = 0
        start = time.time()

        # Gain for the fuzzy controller output

        while t <= duration:
            rclpy.spin_once(self, timeout_sec=0.0)

            # Desired position on circle
            xd = radius * np.cos(omega * t) + center_x
            yd = radius * np.sin(omega * t) + center_y
            zd = z_height

            # Actual position from forward kinematics
            xa, ya, za = self.forward.get_ee_position(self.joint_angles)

            # Position errors
            ex = xd - xa
            ey = yd - ya

            error = np.array([ex, ey, 0.0])
            ee_vel = self.fuzzy.compute(error)
            dq = self.inverse.get_inverse_velocity(ee_vel, self.joint_angles)
            self.publish_velocity(dq)

            t += timestep
            sleep = start + t - time.time()
            if sleep > 0:
                time.sleep(sleep)


    def move_to_initial_pos(self):
        duration = 10
        time_curr = 0
        timestep = 0.001
        start_time = time.time()
        q_c2 = np.array([0.076432, -0.038377, 0.061726, 0.118620, -0.047124, -0.029308])
        q_c3 = np.array([-0.005095, 0.002558, -0.004115, -0.007908, 0.003142, 0.001954])

        while time_curr <= duration:
            rclpy.spin_once(self, timeout_sec = 0.0)
            velocity = 2*q_c2*time_curr + 3 * q_c3 * (time_curr**2)
            self.publish_velocity(velocity)

            time_curr += timestep
            time_to_wait = start_time + time_curr - time.time()
            if time_to_wait > 0:
                time.sleep(time_to_wait)



# Main function
def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopNode()

    node.get_logger().info("Moving to initial position...")
    node.move_to_initial_pos()
    time.sleep(0.5)

    node.get_logger().info("Starting closed-loop fuzzy circle tracking...")
    node.get_logger().info("Waiting for joint state data...")
    
    # Wait for joint state data
    while not node.joint_received:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    node.get_logger().info("Joint state received! Starting motion...")

    time_before_movement = time.time()
    node.cut_batman()
    # node.follow_circle()
    time_after_movement = time.time()
    total_time_taken = round(time_after_movement - time_before_movement, 3)
    node.get_logger().info(f"Finished trajectory plan in {total_time_taken} seconds!")


    node.get_logger().info("Finished.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
