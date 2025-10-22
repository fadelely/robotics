#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import logging
import numpy as np
from ikpy.chain import Chain
from ikpy.link import URDFLink
logger = logging.getLogger()
	
class UR5eNode(Node):
    def __init__(self,target):
        super().__init__('ur5e')
        self.get_logger().info('UR5e node started')
        self.publisher = self.create_publisher(Float64MultiArray,'/joint_commands',100)
        #self.get_topic_names_and_types()
        self.topics = self.get_topic_names_and_types()
        
        self.ik(target=target)
    
    def ik(self,target):
         #print(2)
        active_links_mask = [
		False,  # Base link
		True, False,  # shoulder_pan_joint, offset
		True, False,  # shoulder_lift_joint, offset
		True, False,  # elbow_joint, offset
		True, False,  # wrist_1_joint, offset
		True, False,  # wrist_2_joint, offset
		True, False   # wrist_3_joint, offset
	    ]
        
        #node = Node().__init__('ur5e')
        #publisher = self.create_publisher(Float64MultiArray,'/joint_commands',10)
        
        
        my_chain = Chain.from_urdf_file("ur.urdf",
		base_elements=["base"],
		active_links_mask=active_links_mask)
        
        joint_angles = my_chain.inverse_kinematics(target)
        
        revolute_indices = [i for i, link in enumerate(my_chain.links) if link.joint_type == 'revolute']
        revolute_angles = [joint_angles[i] for i in revolute_indices]
        
        print("6 Revolute Joint Angles:", revolute_angles)
        print("Joint angles : " , len(revolute_angles))
        
        msg = Float64MultiArray()
        msg.data = revolute_angles
        self.publisher.publish(msg)
        topics = self.get_topic_names_and_types()
        
        for name, types in topics:
            print(f"Topic: {name}  Type: {types}\n")	
		

def main(args=None):
    rclpy.init(args=args)
    
    target = list(map(float,input("Enter x y z: ").split()))
    while len(target)>0 :
        
        node = UR5eNode(target=target)
        target = list(map(float,input("Enter x y z: ").split()))
    
    #node.ik(target,node)
    
    rclpy.spin_once(node,timeout_sec=2)
  #  node.destroy_node()
    #rclpy.shutdown()
    

if __name__ == '__main__':
    main()




# ik(target)

#print(target)
