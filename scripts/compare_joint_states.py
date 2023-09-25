#!/usr/bin/env python
#
# script that subscribes to joint state topics and calculates differences


import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class CompareJointStates:
    def __init__(self):

    
        self.gazebo_joint_states = JointState()
        self.joint_state_differences = Float64MultiArray()
        self.joint_state_differences.data = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        
        rospy.Subscriber('/joint_states', JointState, self.callback_gazebo_jointstates, queue_size=1)
        rospy.Subscriber('/estimated_joint_states', JointState, self.callback_estimated_jointstates, queue_size=1)

        # publish gripper joinst state differences als float array
        self.pub_joint_state_differences = rospy.Publisher('/js_differences', Float64MultiArray, queue_size=1)
        
    

    def callback_gazebo_jointstates(self, states):
        self.gazebo_joint_states = states

    def callback_estimated_jointstates(self, states):
        self.estimated_joint_states = states

        # substract gripper joint states 
        self.joint_state_differences.data[0] = self.gazebo_joint_states.position[0] - self.estimated_joint_states.position[0] 
        self.joint_state_differences.data[1] = self.gazebo_joint_states.position[1] - self.estimated_joint_states.position[1] 
        self.joint_state_differences.data[2] = self.gazebo_joint_states.position[2] - self.estimated_joint_states.position[2] 
        self.joint_state_differences.data[3] = self.gazebo_joint_states.position[3] - self.estimated_joint_states.position[3] 
        self.joint_state_differences.data[4] = self.gazebo_joint_states.position[4] - self.estimated_joint_states.position[4] 
        self.joint_state_differences.data[5] = self.gazebo_joint_states.position[5] - self.estimated_joint_states.position[5] 
        self.joint_state_differences.data[6] = self.gazebo_joint_states.position[6] - self.estimated_joint_states.position[6] 
        self.joint_state_differences.data[7] = self.gazebo_joint_states.position[7] - self.estimated_joint_states.position[7] 
        self.joint_state_differences.data[8] = self.gazebo_joint_states.position[8] - self.estimated_joint_states.position[8] 
        # publish difference
    
        self.pub_joint_state_differences.publish(self.joint_state_differences)



   

    
if __name__ == "__main__":
    rospy.init_node('compare_joint_states', anonymous=True)

    newComparison = CompareJointStates()

    rospy.spin()

   
    