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
        self.sub = self.create_subscription(
        JointState,
        "/joint_state",
        self.joint_angle_callback,
        10
        )


    def joint_angle_callback(self, msg):
        self.joint_angles = list(msg.position[:-1])

    def start_trajectory(self):
        duration = 10
        time_curr = 0
        timestep = 0.001
        start_time = time.time()

        while time_curr <= duration:
            rclpy.spin_once(self, timeout_sec = 0.0)
            q_1 = -2.788547563050074 - 0.05*time_curr**2
            q_2 = -0.5807988920173397 - 0.01*time_curr**2 
            q_3 = 1.102026594222845  - 0.e-2*time_curr**2 
            q_4 = -2.0920240290004024 +  0.02*time_curr**2 
            q_5 = 1.5707963267948966 
            q_6 = -1.9238414173346157 + 0.05*time_curr**2
    
            joints = [q_1,q_2,q_3,q_4,q_5,q_6]
            self.publish(joints)

            time_curr += timestep
            # this is just to make it wait the timestep before sending the next one
            # we dont do time.sleep(time_step) directly due to os scheduling issues :)
            # i.e. the computation of the position could take more time than the timestep we have, causing a 
            # drift if we add onto that computation time an unnesscary sleep, so to keep it in sync, we 
            # calculate the time passed instead of the time step itself 
            time_to_wait = start_time + time_curr - time.time()
            if time_to_wait > 0:
                time.sleep(time_to_wait)




    def publish(self, joints):
      

        msg = Float64MultiArray()
        msg.data = joints + [0.0]  
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UR5eIKNode()

    node.get_logger().info("Started node! Moving with initial joints values (-2.78, -0.58 ,1.10, -2.09, 1.57, -1.92)...")
    Initial_Q = [-2.788547563050074, -0.5807988920173397, 1.102026594222845, 
            -2.0920240290004024, 1.5707963267948966, -1.9238414173346157]
    
    rclpy.spin_once(node, timeout_sec=0.1)
    node.publish(Initial_Q)
    time.sleep(1.0)

    time_before_movement = time.time()
    node.get_logger().info("Moving robot arm to the final pos reaching these joints values ([-4.48, -0.91, 0.91, -1.57, 1.57, -0.22) in 10 seconds using joint-space trajectory planning...")
    node.start_trajectory()
    time_after_movement = time.time()
    total_time_taken = round(time_after_movement - time_before_movement, 3)
    node.get_logger().info(f"Finished trajectory plan in {total_time_taken} seconds!")


    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

