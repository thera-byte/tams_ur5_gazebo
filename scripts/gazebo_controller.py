#!/usr/bin/env python
#
# script that subscribes to sensor topics on the robotiq fingers and builds tuples from the collision data

import os
import sys
import rospy
import moveit_commander
import numpy as np
import actionlib
import tams_ur5_gazebo.msg 
import time

from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import JointState
#from control_msgs.msg import FollowJointTrajectoryActionGoal
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

# create and fill tuple with collisions and limits
class MoveFingers:
    def __init__(self):

        # move group gripper
        gripper_group_name = "gripper"
        self.gripper_move_group = moveit_commander.MoveGroupCommander( gripper_group_name )
        self.gripper_move_group.set_max_velocity_scaling_factor( 1.0 )
        self.gripper_move_group.set_max_acceleration_scaling_factor( 1.0 )

        self.tuple = [0, 0, 0, 0, 0, 0]
        self.tuple_f1 = [0, 0, 0, 0, 0, 0]
        self.tuple_f2 = [0, 0, 0, 0, 0, 0]

        self.endstate = [0, 0, 0] # middle finger, f1, f2
        self.new_poses = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        
        #start ros node
        #rospy.init_node('listener', anonymous=True)
        #Create a subscribers to all sensors on all fingers
        rospy.Subscriber('/s_model_finger_middle_link_1_sensor', ContactsState, self.callback1, queue_size=10)
        rospy.Subscriber('/s_model_finger_middle_link_2_sensor', ContactsState, self.callback2, queue_size=10)
        rospy.Subscriber('/s_model_finger_middle_link_3_sensor', ContactsState, self.callback3, queue_size=10)
        rospy.Subscriber('/s_model_finger_1_link_1_sensor', ContactsState, self.callback_f1_1, queue_size=10)
        rospy.Subscriber('/s_model_finger_1_link_2_sensor', ContactsState, self.callback_f1_2, queue_size=10)
        rospy.Subscriber('/s_model_finger_1_link_3_sensor', ContactsState, self.callback_f1_3, queue_size=10)
        rospy.Subscriber('/s_model_finger_2_link_1_sensor', ContactsState, self.callback_f2_1, queue_size=10)
        rospy.Subscriber('/s_model_finger_2_link_2_sensor', ContactsState, self.callback_f2_2, queue_size=10)
        rospy.Subscriber('/s_model_finger_2_link_3_sensor', ContactsState, self.callback_f2_3, queue_size=10)
        # subscriber to current jointstates
        rospy.Subscriber('/joint_states', JointState, self.callback4, queue_size=10)
        #subscriber to newly calculated jointstates
        rospy.Subscriber('/new_thetas', Float32MultiArray, self.callback5, queue_size=1)

        

        #print("das is g:", g)

        self._as = actionlib.SimpleActionServer('grasp', tams_ur5_gazebo.msg.graspAction, execute_cb=self.action_callback, auto_start = False)
        self._as.start()

        rospy.spin()
        
    def iterate_movements(self, g, g1, g2):
            #rospy.Timer(rospy.Duration(0.1), self.calculate_new_pose)
        #counter = 0
        
        # nur damit hand schneller aufgeht
        #if g == 0 and g1 == 0 and g2 == 0:
              #  gripper_open_pose = self.make_basic_gripper_pose( 0.10, 0.02, -0.10, 0.10, 0.02, -0.10, 0.10, 0.02, -0.10 )
              #  self.gripper_move_group.go(gripper_open_pose, wait=True) 
        
        #rate = rospy.Rate(0.01)

        # self.timer.start()
        #while not rospy.is_shutdown():
        iterations = max(g, g1, g2)
        print("ITERATIONS", iterations)
        self.counter = 0
        start_time = rospy.Time.now()
        while self.counter <= iterations and not self.endstate == [1,1,1] and (rospy.Time.now() - start_time).to_sec() < 60:
            print("COUNTER:", self.counter)
            self.calculate_new_pose(g, g1, g2)
            self.move_to_pose(g, g1, g2)
            self.counter += 1

        #abfangen falls hand sich aufgehängt hat o.ä. nach bestimmter zeit
        # if schon 60 sekunden
        # dann timeout_cb callen
        #self.timer = rospy.Timer(rospy.Duration(1), self.timeout_cb) 

        if self.counter >= iterations or self.endstate == [1,1,1]:
            print("------------------------------- max g reached - stopping movement of gripper ------------------------------")
            #self.gripper_move_group.stop()  
            #sys.exit()
            
            self.endstate[0] = 0
            self.endstate[1] = 0
            self.endstate[2] = 0
            self.counter = 0
            self._as.set_succeeded(tams_ur5_gazebo.msg.graspActionResult())
        else:
            print('ABORTED')
            self._as.set_aborted(tams_ur5_gazebo.msg.graspActionResult())
        
        
            
 

        #rospy.spin()        
    def action_callback(self, goal:tams_ur5_gazebo.msg.graspActionGoal):
        self.iterate_movements(goal.g, goal.g1, goal.g2)
        
    def timeout_cb(self, timer):
        print('ABORTED - FAILED')
        self._as.set_preempted()
        self._as.is_active()
        self._as.set_aborted(tams_ur5_gazebo.msg.graspActionResult())
    
    def callback1(self, states):
        if states.states:
            self.tuple[0] = 1
        else:
            self.tuple[0] = 0

    def callback_f1_1(self, states):
        if states.states:
            self.tuple_f1[0] = 1
        else:
            self.tuple_f1[0] = 0

    def callback_f2_1(self, states):
        if states.states:
            self.tuple_f2[0] = 1
        else:
            self.tuple_f2[0] = 0

             
    def callback2(self, states):
        if states.states:
            self.tuple[1]= 1   
        else:
            self.tuple[1]= 0  

    def callback_f1_2(self, states):
        if states.states:
            self.tuple_f1[1]= 1   
        else:
            self.tuple_f1[1]= 0

    def callback_f2_2(self, states):
        if states.states:
            self.tuple_f2[1]= 1   
        else:
            self.tuple_f2[1]= 0
        
    def callback3(self, states):
        if states.states:
            self.tuple[2] = 1
        else:
            self.tuple[2] = 0

    def callback_f1_3(self, states):
        if states.states:
            self.tuple_f1[2] = 1
        else:
            self.tuple_f1[2] = 0

    def callback_f2_3(self, states):
        if states.states:
            self.tuple_f2[2] = 1
        else:
            self.tuple_f2[2] = 0

    def callback4(self, data):
        
        self.jointstates = data


    def callback5(self, new_thetas):  
        if new_thetas.data[6] < 0.0495:
            self.tuple[3] = 1
        elif new_thetas.data[6] > 1.20:
            self.tuple[3] = 1
        else:
            self.tuple[3] = 0

        if new_thetas.data[7] < 0.0:
            self.tuple[4] = 1
        elif new_thetas.data[7] > 1.526: #1.5708:
            self.tuple[4] = 1      
        else:
            self.tuple[4] = 0

        if new_thetas.data[8] < -1.20:  #-1.2217
            self.tuple[5] = -1
        elif new_thetas.data[8] > -0.06: #-0.0523: #-0.068:
            self.tuple[5] = 1  
        else:
            self.tuple[5] = 0
            
      
        if new_thetas.data[0] < 0.0495:
            self.tuple_f1[3] = 1
        elif new_thetas.data[0] > 1.20: #1.2218
            self.tuple_f1[3] = 1
        else:
            self.tuple_f1[3] = 0

        if new_thetas.data[1] < 0.0:
            self.tuple_f1[4] = 1
        elif new_thetas.data[1] > 1.526: #1.5708:
            self.tuple_f1[4] = 1      
        else:
            self.tuple_f1[4] = 0

        if new_thetas.data[2] < -1.20:
            self.tuple_f1[5] = -1
        elif new_thetas.data[2] > -0.06: #-0.0523: #-0.068:
            self.tuple_f1[5] = 1  
        else:
            self.tuple_f1[5] = 0   

        if new_thetas.data[3] < 0.0495:
            self.tuple_f2[3] = 1
        elif new_thetas.data[3] > 1.20:
            self.tuple_f2[3] = 1
        else:
            self.tuple_f2[3] = 0

        if new_thetas.data[4] < 0.0:
            self.tuple_f2[4] = 1
        elif new_thetas.data[4] > 1.526: #1.5708:
            self.tuple_f2[4] = 1      
        else:
            self.tuple_f2[4] = 0

        if new_thetas.data[5] < -1.20:  #-1.2217
            self.tuple_f2[5] = -1
        elif new_thetas.data[5] > -0.06: #-0.0523: #-0.068:
            self.tuple_f2[5] = 1  
        else:
            self.tuple_f2[5] = 0 

        #print("CALLBACK 5 called")        


    #hier anfagen delta thetas anfangen auszurechnen
    # x = (theta_1, theta_2, theta_3, g),  u element of [-1,1] describing change in g from one time step to another
    # u ist hier immer fest TO DO ANPASSEN 
    # funktion f_1(x,u)
    def f_1(self):
        theta_1_max =  1.2 #1.2218
        m_1 = theta_1_max / 140
        
        f_1_xu = m_1 * self.delta_g
        return f_1_xu
    
    # funktion f_2(x,u)
    def f_2(self):
        theta_2_max = 1.526 #1.5708
        m_2 = theta_2_max / 100
        
        f_2_xu = m_2 * self.delta_g
        return f_2_xu
    
    # funktion f_3(x,u):  
    def f_3(self, g):
        theta_3_min =  -1.2 #-1.2217
        theta_3_max = -0.06 #-0.0523 #-0.068:
        
        m_3 = theta_3_min + ((theta_3_max - theta_3_min)/(255 - g))  
        f_3_xu = m_3*self.delta_g
        return f_3_xu
    
    def f_3_g1(self, g1):
        theta_3_min =  -1.2 #-1.2217
        theta_3_max = -0.06 #-0.0523 #-0.068:
        
        m_3 = theta_3_min + ((theta_3_max - theta_3_min)/(255 - g1))  
        f_3_xu = m_3*self.delta_g
        return f_3_xu

    def f_3_g2(self, g2):
        theta_3_min =  -1.2 #-1.2217
        theta_3_max = -0.06 #-0.0523 #-0.068:
        
        m_3 = theta_3_min + ((theta_3_max - theta_3_min)/(255 - g2))  
        f_3_xu = m_3*self.delta_g
        return f_3_xu

    # hier tuple wieder aufrufen und checken in welcher phase wir sind und delta_thetas berechnen
    def calc_delta_theta(self, g):
        # voerst alle bewegungen u = 1 FIX LATER
        if g == 0:
            u = -1
        else:
            u = 1
        #if phase 1 :
        print("tuple: ", self.tuple)
        if self.tuple == [0, 0, 0, 0, 0, 0]:
            
            self.delta_g = u
            self.delta_theta_1 = self.f_1()
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = - self.f_1()
            delta_g = u
            print("middle phase 1: delta theta 1:", self.delta_theta_1)
            print("middle phase 1: delta theta 2:", self.delta_theta_2)
            print("middle phase 1: delta theta 3:", self.delta_theta_3)
            #self.endstate[0] = 0

        #if phase 1':
        elif self.tuple == [0, 0, 0, 0, 0, -1]:
            
            self.delta_g = u
            self.delta_theta_1 = self.f_1()
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = 0.00
            print("phase 1': delta theta 1:", self.delta_theta_1)
            print("phase 1': delta theta 2:", self.delta_theta_2)
            print("phase 1': delta theta 3:", self.delta_theta_3)  
            #self.endstate[0] = 0

        #if phase 2 1:
        elif self.tuple == [1, 0, 0, 0, 0, 0]:
            
            self.delta_g = u
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = self.f_2()
            self.delta_theta_3 = - self.f_2()
            print("phase 2 1: delta theta 1:", self.delta_theta_1)
            print("phase 2 1: delta theta 2:", self.delta_theta_2)
            print("phase 2 1: delta theta 3:", self.delta_theta_3) 
            #self.endstate[0] = 0
            

        #if phase 2 1:
        elif self.tuple[0] == 0 and self.tuple[1] == 0 and self.tuple[2] == 0 and self.tuple[3] == 1 and self.tuple[4] == 0 and self.tuple[5] == 0:
        #[0, 0, 0, 1, 0, 0]:
            
            self.delta_g = u
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = self.f_2()
            self.delta_theta_3 = - self.f_2()
            print("phase 2 2: delta theta 1:", self.delta_theta_1)
            print("phase 2 2: delta theta 2:", self.delta_theta_2)
            print("phase 2 2: delta theta 3:", self.delta_theta_3)
            #self.endstate[0] = 0
            

        #if phase 2' 1:
        elif self.tuple == [1, 0, 0, 0, 0, -1]: 
            
            self.delta_g = u
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = self.f_2()
            self.delta_theta_3 = 0.00
            print("phase 2': delta theta 1:", self.delta_theta_1)
            print("phase 2': delta theta 2:", self.delta_theta_2)
            print("phase 2': delta theta 3:", self.delta_theta_3)
            #self.endstate[0] = 0
            
            

        #if phase 2' 2:
        elif self.tuple == [0, 0, 0, 1, 0, -1]:
            
            self.delta_g = u
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = self.f_2()
            self.delta_theta_3 = 0.00
            print("phase 2' 2: delta theta 1:", self.delta_theta_1)
            print("phase 2' 2: delta theta 2:", self.delta_theta_2)
            print("phase 2' 2: delta theta 3:", self.delta_theta_3)
            #self.endstate[0] = 0
            

        #if phase 3:
        elif self.tuple[1] == 1 and self.tuple[2] == 0 and self.tuple[4] == 0 and self.tuple[5] == 0:
            #[., 1, 0, ., 0, 0]:
            
            self.delta_g = u
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = -self.f_3(g)
            print("phase 3 1: delta theta 1:", self.delta_theta_1)
            print("phase 3 1: delta theta 2:", self.delta_theta_2)
            print("phase 3 1: delta theta 3:", self.delta_theta_3)
            #self.endstate[0] = 0

        elif self.tuple[1] == 0 and self.tuple[2] == 0 and self.tuple[4] == 1 and self.tuple[5] == 0: 
            #[., 0, 0, ., 1, 0]:
            
            self.delta_g = u   
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = -self.f_3(g)
            print("phase 3 2: delta theta 1:", self.delta_theta_1)
            print("phase 3 2: delta theta 2:", self.delta_theta_2)
            print("phase 3 2: delta theta 3:", self.delta_theta_3)
            #self.endstate[0] = 0

        # neue phase 3': 
        elif self.tuple[1] == 1 and self.tuple[2] == 0 and self.tuple[4] == 0 and self.tuple[5] == -1:
            #[., 1, 0, ., 0, -1]:
            
            self.delta_g = u
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = -self.f_3(g)
            print("phase 3' 1: delta theta 1:", self.delta_theta_1)
            print("phase 3' 1: delta theta 2:", self.delta_theta_2)
            print("phase 3' 1: delta theta 3:", self.delta_theta_3)
            #self.endstate[0] = 0

            
        #if phase 4:
        # in phase 4 sind alle 3 delta thetas sowie delta g alle = 0
        elif self.tuple[0]== 0 and self.tuple[1] == 0 and self.tuple[2] == 1 and self.tuple[3] == 0 and self.tuple[4] == 0 and self.tuple[5] == 0:
            # [0, 0, 1, 0, 0, 0]
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = 0.00
            self.delta_g = 0.00
            print("phase 4 1: delta theta 1:", self.delta_theta_1)
            print("phase 4 1: delta theta 2:", self.delta_theta_2)
            print("phase 4 1: delta theta 3:", self.delta_theta_3)
            print( "... Middle Finger End State." )
            self.endstate[0] = 1
            #sys.exit()

        elif self.tuple[0]== 1 and self.tuple[1] == 1 and self.tuple[2] == 1 and self.tuple[3] == 0 and self.tuple[4] == 0 and self.tuple[5] == 0:
            # [1, 1, 1, 0, 0, 0]
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = 0.00
            self.delta_g = 0.00
            print("phase 4 1': delta theta 1:", self.delta_theta_1)
            print("phase 4 1': delta theta 2:", self.delta_theta_2)
            print("phase 4 1': delta theta 3:", self.delta_theta_3)
            print( "... Middle Finger End State." )
            self.endstate[0] = 1
            #sys.exit()    

        elif self.tuple[0]== 1 and self.tuple[1] == 1 and self.tuple[2] == 0 and self.tuple[3] == 0 and self.tuple[4] == 0 and self.tuple[5] == 1: 
            # [1, 1, 0, 0, 0, 1]
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = 0.00
            self.delta_g = 0.00
            print("phase 4 2: delta theta 1:", self.delta_theta_1)
            print("phase 4 2: delta theta 2:", self.delta_theta_2)
            print("phase 4 2: delta theta 3:", self.delta_theta_3)
            print( "... Middle Finger End State." )
            self.endstate[0] = 1
            #sys.exit()

        elif self.tuple[0]== 0 and self.tuple[1] == 1 and self.tuple[2] == 0 and self.tuple[3] == 1 and self.tuple[4] == 0 and self.tuple[5] == 1: 
            # [0, 1, 0, 1, 0, 1]
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = 0.00
            self.delta_g = 0.00
            print("phase 4 2': delta theta 1:", self.delta_theta_1)
            print("phase 4 2': delta theta 2:", self.delta_theta_2)
            print("phase 4 2': delta theta 3:", self.delta_theta_3)
            print( "... Middle Finger End State." )
            self.endstate[0] = 1
            #sys.exit()  

        elif self.tuple[0]== 0 and self.tuple[1] == 1 and self.tuple[2] == 0 and self.tuple[3] == 0 and self.tuple[4] == 0 and self.tuple[5] == 1: 
            # [0, 1, 0, 0, 0, 1]
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = 0.00
            self.delta_g = 0.00
            print("phase 4 2'2: delta theta 1:", self.delta_theta_1)
            print("phase 4 2'2: delta theta 2:", self.delta_theta_2)
            print("phase 4 2'2: delta theta 3:", self.delta_theta_3)
            print( "... Middle Finger End State." )
            self.endstate[0] = 1
            #sys.exit()     

        elif self.tuple[1] == 0 and self.tuple[2] == 0 and self.tuple[4] == 1 and self.tuple[5] == -1: 
            #[., 0, 0, ., 1, -1]:
            
            self.delta_g = u   
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = 0.00
            print("phase 4': delta theta 1:", self.delta_theta_1)
            print("phase 4': delta theta 2:", self.delta_theta_2)
            print("phase 4': delta theta 3:", self.delta_theta_3)
            self.endstate[1] = 1         

        # neue Phase 5 
        """ elif self.tuple[2] == 0 and self.tuple[5] == -1: 
            self.delta_theta_1 = 0
            self.delta_theta_2 = 0
            self.delta_theta_3 = 0
            self.delta_g = 0
            print("phase 5: delta theta 1:", self.delta_theta_1)
            print("phase 5: delta theta 2:", self.delta_theta_2)
            print("phase 5: delta theta 3:", self.delta_theta_3) """
           

    def calc_delta_theta_f1(self, g1):
        # voerst alle bewegungen u = 1 FIX LATER
        if g1 == 0:
            u = -1
        else:
            u = 1
        #if phase 1 :
        print("tuple_f1: ", self.tuple_f1)
        if self.tuple_f1 == [0, 0, 0, 0, 0, 0]:
            
            self.delta_g = u
            self.f1_delta_theta_1 = self.f_1()
            self.f1_delta_theta_2 = 0.00
            self.f1_delta_theta_3 = - self.f_1()
            print("f1 phase 1: delta theta 1:", self.f1_delta_theta_1)
            print("f1 phase 1: delta theta 2:", self.f1_delta_theta_2)
            print("f1 phase 1: delta theta 3:", self.f1_delta_theta_3)
            #self.endstate[1] = 0

        #if phase 1':
        elif self.tuple_f1 == [0, 0, 0, 0, 0, -1]:
            
            self.delta_gdelta_g = u
            self.f1_delta_theta_1 = self.f_1()
            self.f1_delta_theta_2 = 0
            self.f1_delta_theta_3 = 0
            print("f1 phase 1': delta theta 1:", self.f1_delta_theta_1)
            print("f1 phase 1': delta theta 2:", self.f1_delta_theta_2)
            print("f1 phase 1': delta theta 3:", self.f1_delta_theta_3)  
            #self.endstate[1] = 0

        #if phase 2 1:
        elif self.tuple_f1 == [1, 0, 0, 0, 0, 0]:
            
            self.delta_g = u
            self.f1_delta_theta_1 = 0.00
            self.f1_delta_theta_2 = self.f_2()
            self.f1_delta_theta_3 = - self.f_2()
            self.delta_g = u
            print("f1 phase 2 1: delta theta 1:", self.f1_delta_theta_1)
            print("f1 phase 2 1: delta theta 2:", self.f1_delta_theta_2)
            print("f1 phase 2 1: delta theta 3:", self.f1_delta_theta_3) 
            #self.endstate[1] = 0
            

        #if phase 2 1:
        elif self.tuple_f1[0] == 0 and self.tuple_f1[1] == 0 and self.tuple_f1[2] == 0 and self.tuple_f1[3] == 1 and self.tuple_f1[4] == 0 and self.tuple_f1[5] == 0:
        #[0, 0, 0, 1, 0, 0]:
            
            self.delta_g = u
            self.f1_delta_theta_1 = 0
            self.f1_delta_theta_2 = self.f_2()
            self.f1_delta_theta_3 = - self.f_2()
            self.delta_g = u
            print("f1 phase 2 2: delta theta 1:", self.f1_delta_theta_1)
            print("f1 phase 2 2: delta theta 2:", self.f1_delta_theta_2)
            print("f1 phase 2 2: delta theta 3:", self.f1_delta_theta_3)
            #self.endstate[1] = 0

        #if phase 2' 1:
        elif self.tuple_f1 == [1, 0, 0, 0, 0, -1]: 
            
            self.delta_g = u
            self.f1_delta_theta_1 = 0
            self.f1_delta_theta_2 = self.f_2()
            self.f1_delta_theta_3 = 0
            print("f1 phase 2': delta theta 1:", self.f1_delta_theta_1)
            print("f1 phase 2': delta theta 2:", self.f1_delta_theta_2)
            print("f1 phase 2': delta theta 3:", self.f1_delta_theta_3)
            #self.endstate[1] = 0
            

        #if phase 2' 2:
        elif self.tuple_f1 == [0, 0, 0, 1, 0, -1]:
            
            self.delta_g = u
            self.f1_delta_theta_1 = 0
            self.f1_delta_theta_2 = self.f_2()
            self.f1_delta_theta_3 = 0
            print("f1 phase 2' 2: delta theta 1:", self.f1_delta_theta_1)
            print("f1 phase 2' 2: delta theta 2:", self.f1_delta_theta_2)
            print("f1 phase 2' 2: delta theta 3:", self.f1_delta_theta_3)
            #self.endstate[1] = 0

        #if phase 3:
        elif self.tuple_f1[1] == 1 and self.tuple_f1[2] == 0 and self.tuple_f1[4] == 0 and self.tuple_f1[5] == 0:
            #[., 1, 0, ., 0, 0]:
            
            self.delta_g = u
            self.f1_delta_theta_1 = 0
            self.f1_delta_theta_2 = 0
            self.f1_delta_theta_3 = -self.f_3_g1(g1)
            print("f1 phase 3 1: delta theta 1:", self.f1_delta_theta_1)
            print("f1 phase 3 1: delta theta 2:", self.f1_delta_theta_2)
            print("f1 phase 3 1: delta theta 3:", self.f1_delta_theta_3)
            #self.endstate[1] = 0

        elif self.tuple_f1[1] == 0 and self.tuple_f1[2] == 0 and self.tuple_f1[4] == 1 and self.tuple_f1[5] == 0:
            #[., 0, 0, ., 1, 0]:
            
            self.delta_g = u    
            self.f1_delta_theta_1 = 0
            self.f1_delta_theta_2 = 0
            self.f1_delta_theta_3 = -self.f_3_g1(g1)
            print("f1 phase 3 2: delta theta 1:", self.f1_delta_theta_1)
            print("f1 phase 3 2: delta theta 2:", self.f1_delta_theta_2)
            print("f1 phase 3 2: delta theta 3:", self.f1_delta_theta_3)
            #self.endstate[1] = 0

        # neue phase 3': 
        elif self.tuple_f1[1] == 1 and self.tuple_f1[2] == 0 and self.tuple_f1[4] == 0 and self.tuple_f1[5] == -1:
            #[., 1, 0, ., 0, -1]:
            
            self.delta_g = u
            self.f1_delta_theta_1 = 0.00
            self.f1_delta_theta_2 = 0.00
            self.f1_delta_theta_3 = self.f_3_g1(g1)
            print("f1 phase 3' 1: delta theta 1:", self.f1_delta_theta_1)
            print("f1 phase 3' 1: delta theta 2:", self.f1_delta_theta_2)
            print("f1 phase 3' 1: delta theta 3:", self.f1_delta_theta_3)
            #self.endstate[1] = 0

   
            
        #if phase 4:
        # in phase 4 sind alle 3 delta thetas sowie delta g alle = 0
        elif self.tuple_f1[2] == 1 and self.tuple_f1[5] == 0:
            #[., ., 1, ., ., 0]:
            
            self.delta_g = u
            self.f1_delta_theta_1 = 0
            self.f1_delta_theta_2 = 0
            self.f1_delta_theta_3 = 0
            print("f1 phase 4 1: delta theta 1:", self.f1_delta_theta_1)
            print("f1 phase 4 1: delta theta 2:", self.f1_delta_theta_2)
            print("f1 phase 4 1: delta theta 3:", self.f1_delta_theta_3)
            print( "... Finger 1 End State." )
            self.endstate[1] = 1
            #sys.exit()

        elif self.tuple_f1[2] == 0 and self.tuple_f1[5] == 1: 
            #[., ., 0, ., ., 1]:
            #[., ., 1, ., ., 0]:
            
            self.delta_g = u
            self.f1_delta_theta_1 = 0
            self.f1_delta_theta_2 = 0
            self.f1_delta_theta_3 = 0
            print("f1 phase 4 2: delta theta 1:", self.f1_delta_theta_1)
            print("f1 phase 4 2: delta theta 2:", self.f1_delta_theta_2)
            print("f1 phase 4 2: delta theta 3:", self.f1_delta_theta_3)
            print( "... Finger 1 End State." )
            self.endstate[1] = 1
            #sys.exit()

        elif self.tuple_f1[1] == 0 and self.tuple_f1[2] == 0 and self.tuple_f1[4] == 1 and self.tuple_f1[5] == -1: 
            #[., 0, 0, ., 1, -1]:
            
            self.delta_g = u   
            self.f1_delta_theta_1 = 0.00
            self.f1_delta_theta_2 = 0.00
            self.f1_delta_theta_3 = 0.00
            print("f1 phase 4': delta theta 1:", self.f1_delta_theta_1)
            print("f1 phase 4': delta theta 2:", self.f1_delta_theta_2)
            print("f1 phase 4': delta theta 3:", self.f1_delta_theta_3)
            self.endstate[1] = 1      

        # neue Phase 5 
        """ elif self.tuple_f1[2] == 0 and self.tuple_f1[5] == -1: 
            self.f1_delta_theta_1 = 0
            self.f1_delta_theta_2 = 0
            self.f1_delta_theta_3 = 0
            self.delta_g = 0
            print("f1 phase 5: delta theta 1:", self.f1_delta_theta_1)
            print("f1 phase 5: delta theta 2:", self.f1_delta_theta_2)
            print("f1 phase 5: delta theta 3:", self.f1_delta_theta_3) """

    def calc_delta_theta_f2(self, g2):
        # voerst alle bewegungen  FIX LATER
        if g2 == 0:
            u = -1
        else:
            u = 1
        #if phase 1 :
        print("tuple_f2: ", self.tuple_f2)
        if self.tuple_f2 == [0, 0, 0, 0, 0, 0]:
            self.f2_delta_theta_1 = self.f_1()
            self.f2_delta_theta_2 = 0
            self.f2_delta_theta_3 = - self.f_1()
            self.delta_g = u
            print("f2 phase 1: delta theta 1:", self.f2_delta_theta_1)
            print("f2 phase 1: delta theta 2:", self.f2_delta_theta_2)
            print("f2 phase 1: delta theta 3:", self.f2_delta_theta_3)
            #self.endstate[2] = 0

        #if phase 1':
        elif self.tuple_f2 == [0, 0, 0, 0, 0, -1]:
            self.f2_delta_theta_1 = self.f_1()
            self.f2_delta_theta_2 = 0
            self.f2_delta_theta_3 = 0
            self.delta_g = u
            print("f2 phase 1': delta theta 1:", self.f2_delta_theta_1)
            print("f2 phase 1': delta theta 2:", self.f2_delta_theta_2)
            print("f2 phase 1': delta theta 3:", self.f2_delta_theta_3)  
            #self.endstate[2] = 0

        #if phase 2 1:
        elif self.tuple_f2 == [1, 0, 0, 0, 0, 0]:
            self.f2_delta_theta_1 = 0
            self.f2_delta_theta_2 = self.f_2()
            self.f2_delta_theta_3 = - self.f_2()
            self.delta_g = u
            print("f2 phase 2 1: delta theta 1:", self.f2_delta_theta_1)
            print("f2 phase 2 1: delta theta 2:", self.f2_delta_theta_2)
            print("f2 phase 2 1: delta theta 3:", self.f2_delta_theta_3) 
            #self.endstate[2] = 0
            

        #if phase 2 1:
        elif self.tuple_f2[0] == 0 and self.tuple_f2[1] == 0 and self.tuple_f2[2] == 0 and self.tuple_f2[3] == 1 and self.tuple_f2[4] == 0 and self.tuple_f2[5] == 0:
        #[0, 0, 0, 1, 0, 0]:
            self.f2_delta_theta_1 = 0
            self.f2_delta_theta_2 = self.f_2()
            self.f2_delta_theta_3 = - self.f_2()
            self.delta_g = u
            print("f2 phase 2 2: delta theta 1:", self.f2_delta_theta_1)
            print("f2 phase 2 2: delta theta 2:", self.f2_delta_theta_2)
            print("f2 phase 2 2: delta theta 3:", self.f2_delta_theta_3)
            #self.endstate[2] = 0
            

        #if phase 2' 1:
        elif self.tuple_f2 == [1, 0, 0, 0, 0, -1]: 
            self.f2_delta_theta_1 = 0
            self.f2_delta_theta_2 = self.f_2()
            self.f2_delta_theta_3 = 0
            self.delta_g = u
            print("f2 phase 2': delta theta 1:", self.f2_delta_theta_1)
            print("f2 phase 2': delta theta 2:", self.f2_delta_theta_2)
            print("f2 phase 2': delta theta 3:", self.f2_delta_theta_3)
            #self.endstate[2] = 0
            
            

        #if phase 2' 2:
        elif self.tuple_f2 == [0, 0, 0, 1, 0, -1]:
            self.f2_delta_theta_1 = 0
            self.f2_delta_theta_2 = self.f_2()
            self.f2_delta_theta_3 = 0
            self.delta_g = u
            print("f2 phase 2' 2: delta theta 1:", self.f2_delta_theta_1)
            print("f2 phase 2' 2: delta theta 2:", self.f2_delta_theta_2)
            print("f2 phase 2' 2: delta theta 3:", self.f2_delta_theta_3)
            #self.endstate[2] = 0
            

        #if phase 3:
        elif self.tuple_f2[1] == 1 and self.tuple_f2[2] == 0 and self.tuple_f2[4] == 0 and self.tuple_f2[5] == 0:
            
            self.delta_g = u
            self.f2_delta_theta_1 = 0
            self.f2_delta_theta_2 = 0
            self.f2_delta_theta_3 = -self.f_3_g2(g2)
            print("f2 phase 3 1: delta theta 1:", self.f2_delta_theta_1)
            print("f2 phase 3 1: delta theta 2:", self.f2_delta_theta_2)
            print("f2 phase 3 1: delta theta 3:", self.f2_delta_theta_3)
            #self.endstate[2] = 0

        elif self.tuple_f2[1] == 0 and self.tuple_f2[2] == 0 and self.tuple_f2[4] == 1 and self.tuple_f2[5] == 0:    
            
            self.delta_g = u
            self.f2_delta_theta_1 = 0
            self.f2_delta_theta_2 = 0
            self.f2_delta_theta_3 = -self.f_3_g2(g2)
            print("f2 phase 3 2: delta theta 1:", self.f2_delta_theta_1)
            print("f2 phase 3 2: delta theta 2:", self.f2_delta_theta_2)
            print("f2 phase 3 2: delta theta 3:", self.f2_delta_theta_3)
            #self.endstate[2] = 0

        # neue phase 3': 
        elif self.tuple_f2[1] == 1 and self.tuple_f2[2] == 0 and self.tuple_f2[4] == 0 and self.tuple_f2[5] == -1:
            #[., 1, 0, ., 0, -1]:
            
            self.delta_g = u
            self.f2_delta_theta_1 = 0.00
            self.f2_delta_theta_2 = 0.00
            self.f2_delta_theta_3 = -self.f_3_g2(g2)
            print("f2 phase 3' 1: delta theta 1:", self.f2_delta_theta_1)
            print("f2 phase 3' 1: delta theta 2:", self.f2_delta_theta_2)
            print("f2 phase 3' 1: delta theta 3:", self.f2_delta_theta_3)
            #self.endstate[2] = 0

        elif self.tuple_f2[1] == 0 and self.tuple_f2[2] == 0 and self.tuple_f2[4] == 1 and self.tuple_f2[5] == -1: 
            #[., 0, 0, ., 1, -1]:
            
            self.delta_g = u   
            self.f2_delta_theta_1 = 0.00
            self.f2_delta_theta_2 = 0.00
            self.f2_delta_theta_3 = -self.f_3_g2(g2)
            print("f2 phase 3' 2: delta theta 1:", self.f2_delta_theta_1)
            print("f2 phase 3' 2: delta theta 2:", self.f2_delta_theta_2)
            print("f2 phase 3' 2: delta theta 3:", self.f2_delta_theta_3) 
            #self.endstate[2] = 0   
            
        #if phase 4:
        # in phase 4 sind alle 3 delta thetas sowie delta g alle = 0
        elif self.tuple_f2[2] == 1 and self.tuple_f2[5] == 0:
            self.f2_delta_theta_1 = 0
            self.f2_delta_theta_2 = 0
            self.f2_delta_theta_3 = 0
            self.delta_g = 0
            print("f2 phase 4 1: delta theta 1:", self.f2_delta_theta_1)
            print("f2 phase 4 1: delta theta 2:", self.f2_delta_theta_2)
            print("f2 phase 4 1: delta theta 3:", self.f2_delta_theta_3)
            print( "... Finger 2 End State." )
            self.endstate[2] = 1
            #sys.exit()

        elif self.tuple_f2[2] == 0 and self.tuple_f2[5] == 1: 
            self.f2_delta_theta_1 = 0
            self.f2_delta_theta_2 = 0
            self.f2_delta_theta_3 = 0
            self.delta_g = 0
            print("f2 phase 4 2: delta theta 1:", self.f2_delta_theta_1)
            print("f2 phase 4 2: delta theta 2:", self.f2_delta_theta_2)
            print("f2 phase 4 2: delta theta 3:", self.f2_delta_theta_3)
            print( "... Finger 2 End State." )
            self.endstate[2] = 1
            #sys.exit()

        elif self.tuple_f2[1] == 0 and self.tuple_f2[2] == 0 and self.tuple_f2[4] == 1 and self.tuple_f2[5] == -1: 
            #[., 0, 0, ., 1, -1]:
            
            self.delta_g = u   
            self.f2_delta_theta_1 = 0.00
            self.f2_delta_theta_2 = 0.00
            self.f2_delta_theta_3 = 0.00
            print("f2 phase 4': delta theta 1:", self.f2_delta_theta_1)
            print("f2 phase 4': delta theta 2:", self.f2_delta_theta_2)
            print("f2 phase 4': delta theta 3:", self.f2_delta_theta_3)
            self.endstate[1] = 1       

        # neue Phase 5 
        """elif self.tuple_f2[2] == 0 and self.tuple_f2[5] == -1: 
            self.f2_delta_theta_1 = 0
            self.f2_delta_theta_2 = 0
            self.f2_delta_theta_3 = 0
            self.delta_g = 0
            print("f2 phase 5: delta theta 1:", self.f2_delta_theta_1)
            print("f2 phase 5: delta theta 2:", self.f2_delta_theta_2)
            print("f2 phase 5: delta theta 3:", self.f2_delta_theta_3)       """       
            

    # hier delta thetas auf actuelle position aufrechnen um neue goal positionen zu erhalten
    #def calculate_new_pose(self, _timer_event):
    def calculate_new_pose(self, g, g1, g2):    
        self.calc_delta_theta(g)
        self.calc_delta_theta_f1(g1)
        self.calc_delta_theta_f2(g2)

        # new positions middle finger
        new_pos_finger_middle_joint_1 = self.delta_theta_1 + self.jointstates.position[6] # position 6 is for 1st joint for middle finger
        print(" middle new theta 1:", new_pos_finger_middle_joint_1)
        print("middle current jointsstate 1:", self.jointstates.position[6])
        new_pos_finger_middle_joint_2 = self.delta_theta_2 + self.jointstates.position[7] # secont joint middle finger
        print("middle new theta 2:", new_pos_finger_middle_joint_2)
        print("middle current jointsstate 2:", self.jointstates.position[7])
        new_pos_finger_middle_joint_3 = self.delta_theta_3 + self.jointstates.position[8]
        print("middle new theta 3:", new_pos_finger_middle_joint_3)
        print("middle current jointsstate 3:", self.jointstates.position[8])

        # new positions finger 1
        new_pos_finger_1_joint_1 = self.f1_delta_theta_1 + self.jointstates.position[0] # position 6 is for 1st joint for 1 finger
        print(" f1 new theta 1:", new_pos_finger_1_joint_1)
        print("f1 current jointsstate 1:", self.jointstates.position[0])
        new_pos_finger_1_joint_2 = self.f1_delta_theta_2 + self.jointstates.position[1] # second joint 1 finger
        print("f1 new theta 2:", new_pos_finger_1_joint_2)
        print("f1 current jointsstate 2:", self.jointstates.position[1])
        new_pos_finger_1_joint_3 = self.f1_delta_theta_3 + self.jointstates.position[2]
        print("f1 new theta 3:", new_pos_finger_1_joint_3)
        print("f1 current jointsstate 3:", self.jointstates.position[2])

        # new positions finger 2
        new_pos_finger_2_joint_1 = self.f2_delta_theta_1 + self.jointstates.position[3] # position 6 is for 1st joint for 1 finger
        print(" f2 new theta 1:", new_pos_finger_2_joint_1)
        print("f2 current jointsstate 1:", self.jointstates.position[3])
        new_pos_finger_2_joint_2 = self.f2_delta_theta_2 + self.jointstates.position[4] # secont joint 1 finger
        print("f2 new theta 2:", new_pos_finger_2_joint_2)
        print("f2 current jointsstate 2:", self.jointstates.position[4])
        new_pos_finger_2_joint_3 = self.f2_delta_theta_3 + self.jointstates.position[5]
        print("f2 new theta 3:", new_pos_finger_2_joint_3)
        print("f2 current jointsstate 3:", self.jointstates.position[5])

        pub = rospy.Publisher('/new_thetas', Float32MultiArray, queue_size=1)
        new_thetas = [new_pos_finger_1_joint_1, new_pos_finger_1_joint_2, new_pos_finger_1_joint_3, 
                      new_pos_finger_2_joint_1, new_pos_finger_2_joint_2, new_pos_finger_2_joint_3, 
                      new_pos_finger_middle_joint_1, new_pos_finger_middle_joint_2, new_pos_finger_middle_joint_3]
        """new_thetas = [self.new_pos_finger_1_joint_1, self.new_pos_finger_1_joint_2, self.new_pos_finger_1_joint_3, 
                      self.new_pos_finger_2_joint_1, self.new_pos_finger_2_joint_2, self.new_pos_finger_2_joint_3, 
                      self.new_pos_finger_middle_joint_1, self.new_pos_finger_middle_joint_2, self.new_pos_finger_middle_joint_3]"""
        data_to_send = Float32MultiArray()
        data_to_send.data = new_thetas
        rospy.loginfo(data_to_send)
        pub.publish(data_to_send)

        self.new_poses = [new_pos_finger_1_joint_1, new_pos_finger_1_joint_2, new_pos_finger_1_joint_3, 
                      new_pos_finger_2_joint_1, new_pos_finger_2_joint_2, new_pos_finger_2_joint_3, 
                      new_pos_finger_middle_joint_1, new_pos_finger_middle_joint_2, new_pos_finger_middle_joint_3]
        


    def move_to_pose(self,g, g1, g2): 
        if self.new_poses[2] >= -0.06:
            self.tuple_f1[5] = 1
        if self.new_poses[5] >= -0.06:
            self.tuple_f2[5] = 1
        if self.new_poses[8] >= -0.06:
            self.tuple[5] = 1
        


        self.calc_delta_theta(g)
        self.calc_delta_theta_f1(g1)
        self.calc_delta_theta_f2(g2)

        new_pos_finger_1_joint_1 = self.new_poses[0]  
        new_pos_finger_1_joint_2 = self.new_poses[1] 
        new_pos_finger_1_joint_3 = self.new_poses[2] 
        new_pos_finger_2_joint_1 = self.new_poses[3] 
        new_pos_finger_2_joint_2 = self.new_poses[4] 
        new_pos_finger_2_joint_3 = self.new_poses[5] 
        new_pos_finger_middle_joint_1 = self.new_poses[6] 
        new_pos_finger_middle_joint_2 = self.new_poses[7] 
        new_pos_finger_middle_joint_3 = self.new_poses[8] 

        if self.counter >= g:
            self.endstate[0] = 1
            print("------ Reached G Value Middle Finger ------")
        if self.counter >= g1:
            self.endstate[1] = 1
            print("------ Reached G Value Finger 1 ------")
        if self.counter >= g2:
            self.endstate[2] = 1  
            print("------ Reached G Value Finger 2 ------")      

        if self.endstate == [1, 0, 0]:
            print("middle finger endstate gets different new positions now", self.endstate)
            print("middle finger new positions:", self.jointstates.position[6], self.jointstates.position[7], self.jointstates.position[8] )
            new_pos_finger_middle_joint_1 = self.jointstates.position[6]
            new_pos_finger_middle_joint_2 = self.jointstates.position[7]
            new_pos_finger_middle_joint_3 = self.jointstates.position[8]
        elif self.endstate == [0, 1, 0]:
            print("finger 1 endstate gets different new positions now", self.endstate)
            print("f1 new positions:", self.jointstates.position[0], self.jointstates.position[1], self.jointstates.position[2] )
            new_pos_finger_1_joint_1 = self.jointstates.position[0]  
            new_pos_finger_1_joint_2 = self.jointstates.position[1] 
            new_pos_finger_1_joint_3 = self.jointstates.position[2]
        elif self.endstate == [0, 0, 1]:
            print("finger 2 endstate gets different new positions now", self.endstate)
            print("f2 new positions:", self.jointstates.position[3], self.jointstates.position[4], self.jointstates.position[5] )
            new_pos_finger_2_joint_1 = self.jointstates.position[3]    
            new_pos_finger_2_joint_2 = self.jointstates.position[4]
            new_pos_finger_2_joint_3 = self.jointstates.position[5]
        if self.endstate == [1, 1, 0]:
            print("middle finger and f1 endstate get different new positions now", self.endstate)
            print("middle finger new positions:", self.jointstates.position[6], self.jointstates.position[7], self.jointstates.position[8] )
            new_pos_finger_middle_joint_1 = self.jointstates.position[6]
            new_pos_finger_middle_joint_2 = self.jointstates.position[7]
            new_pos_finger_middle_joint_3 = self.jointstates.position[8]
            print("f1 new positions:", self, self.jointstates.position[0], self.jointstates.position[1], self.jointstates.position[2] )
            new_pos_finger_1_joint_1 = self.jointstates.position[0]  
            new_pos_finger_1_joint_2 = self.jointstates.position[1] 
            new_pos_finger_1_joint_3 = self.jointstates.position[2]
        elif self.endstate == [0, 1, 1]:
            print("finger 1 and 2 endstate get different new positions now", self.endstate)
            print("f1 new positions:", self.jointstates.position[0], self.jointstates.position[1], self.jointstates.position[2] )
            new_pos_finger_1_joint_1 = self.jointstates.position[0]  
            new_pos_finger_1_joint_2 = self.jointstates.position[1] 
            new_pos_finger_1_joint_3 = self.jointstates.position[2]
            print("f2 new positions:", self, self.jointstates.position[3], self.jointstates.position[4], self.jointstates.position[5] )
            new_pos_finger_2_joint_1 = self.jointstates.position[3]    
            new_pos_finger_2_joint_2 = self.jointstates.position[4]
            new_pos_finger_2_joint_3 = self.jointstates.position[5]
        if self.endstate == [1, 0, 1]:
            print("middle finger and f1 endstate get different new positions now", self.endstate)
            print("middle finger new positions:", self.jointstates.position[6], self.jointstates.position[7], self.jointstates.position[8] )
            new_pos_finger_middle_joint_1 = self.jointstates.position[6]
            new_pos_finger_middle_joint_2 = self.jointstates.position[7]
            new_pos_finger_middle_joint_3 = self.jointstates.position[8]
            print("f2 new positions:", self.jointstates.position[3], self.jointstates.position[4], self.jointstates.position[5] )
            new_pos_finger_2_joint_1 = self.jointstates.position[3]    
            new_pos_finger_2_joint_2 = self.jointstates.position[4]
            new_pos_finger_2_joint_3 = self.jointstates.position[5]
        elif self.endstate == [1, 1, 1]:
            print("ALL FINGERS REACHED AN END STATE ---- exiting now ", self.endstate)
            new_pos_finger_middle_joint_1 = self.jointstates.position[6]
            new_pos_finger_middle_joint_2 = self.jointstates.position[7]
            new_pos_finger_middle_joint_3 = self.jointstates.position[8]
            new_pos_finger_1_joint_1 = self.jointstates.position[0]  
            new_pos_finger_1_joint_2 = self.jointstates.position[1] 
            new_pos_finger_1_joint_3 = self.jointstates.position[2]
            new_pos_finger_2_joint_1 = self.jointstates.position[3]    
            new_pos_finger_2_joint_2 = self.jointstates.position[4]
            new_pos_finger_2_joint_3 = self.jointstates.position[5]

            
            #self.gripper_move_group.stop()  
            #sys.exit()


        

        calculated_pose = self.make_basic_gripper_pose( new_pos_finger_1_joint_1, new_pos_finger_1_joint_2, new_pos_finger_1_joint_3,
                                                       new_pos_finger_2_joint_1, new_pos_finger_2_joint_2, new_pos_finger_2_joint_3, 
                                                       new_pos_finger_middle_joint_1, new_pos_finger_middle_joint_2, new_pos_finger_middle_joint_3 )
        self.gripper_move_group.go(calculated_pose, wait=True)


      #  calculated_pose = self.make_basic_gripper_pose( 1.206809, 0.0129566, -1.151036)
       # self.gripper_move_group.go(calculated_pose, wait=True)
        #rospy.sleep(1)
       # calculated_pose = self.make_basic_gripper_pose( 1.207382, 0.069584, -1.213455)
       # self.gripper_move_group.go(calculated_pose, wait=True)
        #rospy.sleep(1)
       # calculated_pose = self.make_basic_gripper_pose( 1.212175, 1.555280, -1.218616)
       # self.gripper_move_group.go(calculated_pose, wait=True)
       # rospy.sleep(0.5)
       # calculated_pose = self.make_basic_gripper_pose( 0.05, 0.0129566, -0.06)
        #self.gripper_move_group.go(calculated_pose, wait=True)


        
    # takes the finger joint angles (proximal, medial, distal) for the Robotiq 3-finger hand and replicates those angles on all three fingers.
    # Finger abduction (palm) joints are hardcoded to "basic" mode (near zero).
    def make_basic_gripper_pose(self, f1_1, f1_2, f1_3, f2_1, f2_2, f2_3, mf1, mf2, mf3 ):
        js = JointState()
        js.name = [ "s_model_finger_1_joint_1", "s_model_finger_1_joint_2", "s_model_finger_1_joint_3", \
                "s_model_finger_2_joint_1", "s_model_finger_2_joint_2", "s_model_finger_2_joint_3", \
                "s_model_finger_middle_joint_1", "s_model_finger_middle_joint_2", "s_model_finger_middle_joint_3", \
                "s_model_palm_finger_1_joint", "s_model_palm_finger_2_joint" ]
        #js.position = [ mf1, mf2, mf3, mf1, mf2, mf3, mf1, mf2, mf3, -0.016, 0.016 ]
        js.position = [ f1_1, f1_2, f1_3, f2_1, f2_2, f2_3, mf1, mf2, mf3, -0.016, 0.016 ]
        return js 
    
    
    def make_arm_pose(self, joint_angles ):
        # note: rostopic pub /joint_states has elbow lift pan wrist 1 2 3 :-(
     
        js = JointState()
        js.name = [ "ur5_elbow_joint", "ur5_shoulder_lift_joint", "ur5_shoulder_pan_joint", \
                "ur5_wrist_1_joint", "ur5_wrist_2_joint", "ur5_wrist_3_joint" ]
        js.position = joint_angles
        return js
        
    

if __name__ == "__main__":
    rospy.init_node('alles', anonymous=True)
    rospy.sleep(1)

    #g = float(input("Please enter g value for middle finger (0 - 254):" ))
    #print("You entered: ",  g)
    #g1 = float(input("Please enter g value for finger 1 (0 - 254):" ))
    #print("You entered: ",  g1)
    #g2 = float(input("Please enter g value for finger 2 (0 - 254):" ))
    #print("You entered: ",  g2)
    newMovement = MoveFingers() 
    #newMovement.iterate_movements(g, g1, g2)

    #rospy.spin()

    