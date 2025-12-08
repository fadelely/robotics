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
        time_curr = 0
        timestep = 0.001
        start_time = time.time()
        duration = 10

        while time_curr <= duration:
            # spin does alot of things, but mainly its purpose is to activate callback functions
            # the timeout_sec is set to 0.0 to prevent it from blocking execution
            rclpy.spin_once(self, timeout_sec = 0.0)
            x_t = 0.2 * np.cos(np.pi * time_curr * 0.2) + 0.4849
            y_t = 0.2 * np.sin(np.pi * time_curr * 0.2) - 0.1741
            z_t = 0.4
            xyz = np.array([x_t, y_t, z_t])
            print(x_t)
            print(y_t)
            # calculate the joints to achieve their respective x, y, z
            self.compute_and_publish(xyz)
            time_curr += timestep
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
        solution = closest_solution[0]
        #[0.0, 0.0, 0.0, -1.57, -1.56, 0.0, 0.0])
        # solution[3] = -np.pi/2
        solution[4] = -np.pi/2
        # the extra 0.0 is due to the conveyor counting as a joint, so we need to pass an extra value for it
        msg.data = closest_solution[0].tolist() + [0.0]  
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UR5eIKNode()

    node.get_logger().info("Started node! Moving to initial position of (0.7, 0.4, 0.3)...")
    initial_pos = np.array([0.2, -0.3, 0.4])
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


