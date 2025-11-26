#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
from ur_analytic_ik import ur5e
import time


class UR5eIKNode(Node):
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


    def joint_angle_callback(self, msg):
        self.joint_angles = list(msg.position[:-1])

    def start_trajectory(self):
        # the trajectory is to go from (0.7, 0.4, 0.3) to (0.3, -0.7, 0.6) in 10 seconds
        # the arm will travel in a straight line, with a new point being calculated and sent to the robot arm
        # every 0.001 seconds, due the the simulation frequency itself being 1000 hz, so 1/1000 is 0.001 seconds
        duration = 10
        time_curr = 0
        timestep = 0.001
        start_time = time.time()

        # Since the robot arm is traveling in a straight line, a simple straight line equation works, defined as:
        # x(t) = x_initial + alpha * t
        # The initial point is given, so all we need to find is alpha. Since we know where the end effector will be
        # at the final timestep, we simply subsitute in the equation to find the alpha, so for example in x(t):
        # At t = 10, the x is 0.3, giving us (0.3 = 0.7 + alpha * 10)
        # By moving the terms around, we can reach (alpha = -0.04)
        # Giving us the equation x(t) = 0.7 - 0.04t
        # Repeating the above steps for both y(t) and z(t) results in the following equations for the end effector:
        # x(t) = 0.7 - 0.04t 
        # y(t) = 0.4 - 0.11t
        # z(t) = 0.3 + 0.03t
        # These equations are then used to find the position of the end effector at each timestep, as seen below.

        while time_curr <= duration:
            # spin does alot of things, but mainly its purpose is to activate callback functions
            # the timeout_sec is set to 0.0 to prevent it from blocking execution
            rclpy.spin_once(self, timeout_sec = 0.0)
            x_t = 0.7 - 0.04 * time_curr
            y_t = 0.4 - 0.11 * time_curr
            z_t = 0.3 + 0.03 * time_curr
            xyz = np.array([x_t, y_t, z_t])
            # calculate the joints to achieve their respective x, y, z
            self.compute_and_publish(xyz)

            time_curr += timestep
            # this is just to make it wait the timestep before sending the next one
            # we dont do time.sleep(time_step) directly due to os scheduling issues :)
            # i.e. the computation of the position could take more time than the timestep we have, causing a 
            # drift if we add onto that computation time an unnesscary sleep, so to keep it in sync, we 
            # calculate the time passed instead of the time step itself 
            time_to_wait = start_time + time_curr - time.time()
            if time_to_wait > 0:
                time.sleep(time_to_wait)


    def compute_and_publish(self, xyz):
        x, y, z = xyz
        # holds the position (and rotation if needed) that we want the robot to reach
        eef_pose = np.identity(4)
        eef_pose[0, 3] = x
        eef_pose[1, 3] = y
        eef_pose[2, 3] = z

        # below chooses the closests one depending on all the joints
        # if for some reason karar yesafer mabeen kol point, replace this
        # to check only the angle q1
        #
        # it also typically chooses elbow up choices, but sometimes decides otherwise
        # if that happens, manually check the first 2 qs to make sure it doesnt bend downward 
        # (inshallah trajectory bet3ana may3melesh keda :*) )
        closest_solution = ur5e.inverse_kinematics_closest(eef_pose, *self.joint_angles)

        if not closest_solution:
            self.get_logger().warn("No IK solutions found!")
            return

        msg = Float64MultiArray()
        # the extra 0.0 is due to the conveyor counting as a joint, so we need to pass an extra value for it
        msg.data = closest_solution[0].tolist() + [0.0]  
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UR5eIKNode()

    node.get_logger().info("Started node! Moving to initial position of (0.7, 0.4, 0.3)...")
    initial_pos = np.array([0.7, 0.4, 0.3])
    rclpy.spin_once(node, timeout_sec=0.1)
    node.compute_and_publish(initial_pos)
    time.sleep(1.0)

    time_before_movement = time.time()
    node.get_logger().info("Moving robot arm to (0.3, -0.7, 0.6) in 10 seconds using task-space trajectory planning...")
    node.start_trajectory()
    time_after_movement = time.time()
    total_time_taken = round(time_after_movement - time_before_movement, 3)
    node.get_logger().info(f"Finished trajectory plan in {total_time_taken} seconds!")


    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

