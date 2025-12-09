#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
from ur_analytic_ik import ur5e
import time
import sympy as sp


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
    
    def _compute_jacobian_numeric(self, q_pos):
        subs = {self.q[i]: q_pos[i] for i in range(6)}
        J_num = np.array(self.J_sym.evalf(subs=subs), dtype=np.float64)
        return J_num

    # given the end effector velocity and the current joint angles, it caluclates the joint velocites
    def get_inverse_velocity(self, ee_vel, q_pos):
        J = self._compute_jacobian_numeric(q_pos)
        J_pinv = np.linalg.pinv(J)
        dq_solution = J_pinv @ ee_vel
        return dq_solution
    
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

    # given the current joint angles, it calculates the position of the end effector
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

        result = np.eye(4)
        for dh in dh_matrices:
            result = np.dot(result, dh)
        x = result[0, 3]
        y = result[1, 3]
        z = result[2, 3]

        return (np.array([x, y, z]))


class FuzzyLogicController():
    def __init__(self):
        print("hello")



class BatmanNode(Node):
    def __init__(self):
        super().__init__('ur5e_ik_node')
        self.publisher = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.joint_angles = [0.0] * 6
        # get the joint values every time the callback function is called
        self.sub = self.create_subscription(
        JointState,
            "/joint_state",
            self.joint_angle_callback,
            10
        )
        self.forward_pos = ForwardPositionKinematics()
        self.inverse_vel = InverseVelocityKinematics()
        self.fuzzy_controller = FuzzyLogicController()


    def joint_angle_callback(self, msg):
        self.joint_angles = list(msg.position[:-1])

    # creates an array of points for x and y (z is always the same never changes)
    # it tries to create as many points needed to last the duration, but this is 
    # not always consistent as sometimes the timestep is too big or duration too short
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


        x_array = self._scale_array(x_array, 0.29, 0.69)
        y_array = self._scale_array(y_array, -0.17, 0.07)
        return x_array, y_array, z_array

    def _scale_array(self, arr, new_min=0.3, new_max=0.7):
        old_min = np.min(arr)
        old_max = np.max(arr)
        return new_min + (arr - old_min) * (new_max - new_min) / (old_max - old_min)

    # TODO does this need to move using fuzzy controller too?
    # position equation = c2 * (time**2) + c3 * (time**3)
    # velocity equation = 2 * c2 * time2 + 3 * c3 * (time**2)
    # moves to the start of the batman cutting using joint space trajectory
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

        
    def cut_batman(self):
        duration = 10
        timestep = 0.001
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
            # compute the velocity of the joints and publish it using self.publish_velocity
            # to get the current position of end effector -> self.forward_pos.get_ee_position(self.joint_angles)
            # to get joint velocity using inverse kinematics -> self.inverse_vel.get_inverse_velocity(<velocity end effector>, self.joint_angles)

            t_expected = duration * i / (len(x) - 1)
            t_actual = time.time() - start_time
            time_to_wait = t_expected - t_actual
            if time_to_wait > 0:
                time.sleep(time_to_wait)



    def publish_velocity(self, dq_vel):
        msg = Float64MultiArray()
        msg.data = list(dq_vel) + [0.0]  
        self.publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = BatmanNode()

    node.get_logger().info("Started node! Moving to initial position...")
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.5)
    node.move_to_initial_pos()
    time.sleep(0.5)

    node.get_logger().info("Starting to cut out batman symbol!")
    time_before_movement = time.time()
    node.cut_batman()
    time_after_movement = time.time()
    total_time_taken = round(time_after_movement - time_before_movement, 3)
    node.get_logger().info(f"It took {total_time_taken} seconds to finish the cut.")


    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
