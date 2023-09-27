#!/usr/bin/env python
#
# script to spawn objects, load the move_fingers.py, delete object, adjust arm position and repeat with new object


import os
import sys
import rospy
import moveit_commander
import numpy as np
import subprocess
import actionlib

import tams_ur5_gazebo.msg

from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import JointState
#from control_msgs.msg import FollowJointTrajectoryActionGoal
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose

class ObjectIterator:
        def __init__(self) -> None:
                # move group am
                self.arm_group_name = "arm"
                self.arm_move_group = moveit_commander.MoveGroupCommander( self.arm_group_name )
                self.arm_move_group.set_max_velocity_scaling_factor( 1.0 )
                self.arm_move_group.set_max_acceleration_scaling_factor( 1.0 )

                # move group gripper
                gripper_group_name = "gripper"
                self.gripper_move_group = moveit_commander.MoveGroupCommander( gripper_group_name )
                self.gripper_move_group.set_max_velocity_scaling_factor( 1.0 )
                self.gripper_move_group.set_max_acceleration_scaling_factor( 1.0 )

                rospy.wait_for_service('gazebo/spawn_sdf_model')
                self.spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

                rospy.wait_for_service('gazebo/delete_model')
                self.delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
                
                self.client = actionlib.SimpleActionClient('grasp', tams_ur5_gazebo.msg.graspAction)
                self.client.wait_for_server()

                #self.client2 = actionlib.SimpleActionClient('grasp2', tams_ur5_gazebo.msg.graspAction)
                #self.client2.wait_for_server()

                self.recording = False

# arm positionieren
        def make_arm_pose(self, joint_angles ):
                # note: rostopic pub /joint_states has elbow lift pan wrist 1 2 3 :-(
                
                js = JointState()
                js.name = [ "ur5_elbow_joint", "ur5_shoulder_lift_joint", "ur5_shoulder_pan_joint", \
                        "ur5_wrist_1_joint", "ur5_wrist_2_joint", "ur5_wrist_3_joint" ]
                js.position = joint_angles
                return js

        def make_basic_gripper_pose(self, f1_1, f1_2, f1_3, f2_1, f2_2, f2_3, mf1, mf2, mf3 ):
                js = JointState()
                js.name = [ "s_model_finger_1_joint_1", "s_model_finger_1_joint_2", "s_model_finger_1_joint_3", \
                        "s_model_finger_2_joint_1", "s_model_finger_2_joint_2", "s_model_finger_2_joint_3", \
                        "s_model_finger_middle_joint_1", "s_model_finger_middle_joint_2", "s_model_finger_middle_joint_3", \
                        "s_model_palm_finger_1_joint", "s_model_palm_finger_2_joint" ]
                #js.position = [ mf1, mf2, mf3, mf1, mf2, mf3, mf1, mf2, mf3, -0.016, 0.016 ]
                js.position = [ f1_1, f1_2, f1_3, f2_1, f2_2, f2_3, mf1, mf2, mf3, -0.016, 0.016 ]
                return js 
        
        def open_hand(self):
                gripper_open_pose = self.make_basic_gripper_pose( 0.10, 0.02, -0.10, 0.10, 0.02, -0.10, 0.10, 0.02, -0.10 )
                self.gripper_move_group.go(gripper_open_pose, wait=True)


        def setup(self):        
                arm_home_pose = self.make_arm_pose( [0.0, -1.571, 0.0, -1.571, 0.0, 0.0] )
                self.arm_move_group.go( arm_home_pose, wait=True )
                self.arm_move_group.stop()  
                # gripper horizontal over middle of the table
                arm_pose_1 = self.make_arm_pose( [ -1.83, -2.68, -1.123, -1.77, -1.13, -3.17 ] )
                self.arm_move_group.go( arm_pose_1, wait=True )
                self.arm_move_group.stop()   
                rospy.sleep(2)

                

        def spwan_model(self, object_path, object_pose):
                # object laden
                # by default, Gazebo SDF models are found in /home/<username>/.gazebo/<modelname>/model.sdf
                #f = open( os.path.expanduser( '~' )  + '/.gazebo/models/tube_9_5mm/model.sdf', 'r' )
                f = open( os.path.expanduser( '~' )  + object_path, 'r' )
                #'/.gazebo/models/coke_can/model.sdf'
                sdfmodel = f.read()
                #spawn_model_prox("tube_9_5mm", sdfmodel, "", initial_pose, "world")  # namespace "coke_can_2" at (1,0.8,0.777) 
                self.spawn_model_prox("coke_can", sdfmodel, "", object_pose, "world")  # namespace "coke_can_2" at (1,0.8,0.777) 


        #rosbag record starten
        def start_bag_record(self):
                if self.recording:
                        rospy.logerr('tried to start recording, while one was running')
                        return
                self.recording = True
                filename = '/home/aura/Schreibtisch/Bags/dings'
                command = 'rosbag record -o {filename} {topics} __name:=grasp_bag'.format(
                filename=filename,
                topics='/joint_states /estimated_joint_states')
                print(command)
                self.rosbag_proc = subprocess.Popen(command, shell=True)
                rospy.sleep(2)

        def stop_bag_record(self):
                if not self.recording:
                        rospy.logerr('tried to stop recording, which did not start')
                        return
                command = 'rosnode kill /grasp_bag'
                self.rosbag_close = subprocess.Popen(command, shell=True)
                self.recording = False
                rospy.sleep(2)

        # move_fingers.py triggern
        def perform_grasp(self, g, g1, g2):
                goal = tams_ur5_gazebo.msg.graspActionGoal()
                goal.goal.g = g
                goal.goal.g1 = g1
                goal.goal.g2 = g2
            
                self.client.send_goal(goal.goal)
                #self.client2.send_goal(goal.goal)
                self.client.wait_for_result()
                #self.client2.wait_for_result()   

        #object löschen
        def delete_object(self):
                self.delete_model_prox("coke_can")




if __name__ == "__main__":
        rospy.init_node('grasp_client')
        rospy.sleep(1)

        x = ObjectIterator()
        
        # delete object if one is left over from previous run
        x.delete_object()
        # open hand if not already open
        x.perform_grasp(0, 0, 0)
        x.open_hand()

        #print( "... spawning a object at (1.0, 0.8, 0.78)..." )
        initial_pose = Pose()
        # tube right pose
        #initial_pose.position.x = 0.96 # 0.96 nach vorne 
        #initial_pose.position.y = 0.77 # 0.80 nach links von bot aus gesehen
        #initial_pose.position.z = 0.78
        # can sorta middle pose
        initial_pose.position.x = 0.98 # 0.96 nach vorne 
        initial_pose.position.y = 0.806 # 0.80 nach links von bot aus gesehen
        initial_pose.position.z = 0.80 
        """# tube pinch position sorta middle
        initial_pose.position.x = 1.04 # 0.96 nach vorne 
        initial_pose.position.y = 0.806 # 0.80 nach links von bot aus gesehen
        initial_pose.position.z = 0.80 """

        
        """x.setup('/ycb/gazebo_ycb/models/chips_can/model.sdf', initial_pose)
        x.start_bag_record()
        x.perform_grasp(254, 254, 254)
        #x.perform_grasp2(24, 24, 24)
        x.stop_bag_record()
        x.delete_object()
        x.perform_grasp(0, 0, 0)
        x.open_hand()
        #x.perform_grasp2(0, 0, 0)  """

        cube_pose = Pose()
        cube_pose.position.x = 0.98  
        cube_pose.position.y = 0.80
        cube_pose.position.z = 0.87 
        x.setup()
        #x.spwan_model('/.gazebo/models/wood_cube_5cm/model.sdf', cube_pose)
        x.start_bag_record()
        x.perform_grasp(254, 254, 254)
        x.stop_bag_record()
        x.delete_object()
        x.perform_grasp(0, 0, 0)
        x.open_hand()

        """beer_pose = Pose()
        beer_pose.position.x = 0.98  
        beer_pose.position.y = 0.80
        beer_pose.position.z = 0.80
        x.setup()
        x.spwan_model('/.gazebo/models/beer/model.sdf', beer_pose)
        x.start_bag_record()
        x.perform_grasp(254, 254, 254)
        x.stop_bag_record()
        x.delete_object()
        x.perform_grasp(0, 0, 0)
        x.open_hand()

        x.setup()
        x.spwan_model('/.gazebo/models/coke_can/model.sdf', initial_pose)
        x.start_bag_record()
        x.perform_grasp(24, 24, 24)
        x.stop_bag_record()
        x.delete_object()
        x.perform_grasp(0, 0, 0)
        x.open_hand()

        cup_pose = Pose()
        cup_pose.position.x = 0.98  
        cup_pose.position.y = 0.79
        cup_pose.position.z = 0.80 
        x.setup()
        x.spwan_model('/.gazebo/models/plastic_cup/model.sdf', cup_pose)
        x.start_bag_record()
        x.perform_grasp(24, 24, 24)
        x.stop_bag_record()
        x.delete_object()
        x.perform_grasp(0, 0, 0)
        x.open_hand()
        """

        
        
