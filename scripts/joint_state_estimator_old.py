#!/usr/bin/env python
#
# script that subscribes to sensor topics on the robotiq fingers and builds tuples from the collision data

import rospy
import numpy as np
import tams_ur5_gazebo.msg 

from std_msgs.msg import Int16
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import JointState
#from control_msgs.msg import FollowJointTrajectoryActionGoal
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
from copy import deepcopy

# create and fill tuple with collisions and limits
class TupleClass:
    def __init__(self):

        self.js_names = [ "s_model_finger_1_joint_1", "s_model_finger_1_joint_2", "s_model_finger_1_joint_3", \
                "s_model_finger_2_joint_1", "s_model_finger_2_joint_2", "s_model_finger_2_joint_3", \
                "s_model_finger_middle_joint_1", "s_model_finger_middle_joint_2", "s_model_finger_middle_joint_3", \
                "s_model_palm_finger_1_joint", "s_model_palm_finger_2_joint" ]
        self.js_positions = [0.10, 0.02, -0.10, 0.10, 0.02, -0.10, 0.10, 0.02, -0.10] # gripper open position inital werte
        self.js_open_positions = [0.10, 0.02, -0.10, 0.10, 0.02, -0.10, 0.10, 0.02, -0.10, 0, 0] # gripper open position inital werte
        
        # diese variable ist zur synchronisierung von gazebo und diesem skript da, zum publishen der js 
        # das is nur nen fix. wird so nicht mit der echten hand funktionieren. dadurch dass jede iteration mit moveit bewegt wird ist die simulation extram langsam
        # die hand soll sich in sim eigentlich genau so schnell bewegen wie die echte hand und dieses skript die js genau so schnell estimaten
        self.rate_counter = 0
        
        self.pub_estimated = rospy.Publisher('/estimated_joint_states', JointState, queue_size=1)

        self.tuple = [0, 0, 0, 0, 0, 0]
        self.tuple_f1 = [0, 0, 0, 0, 0, 0]
        self.tuple_f2 = [0, 0, 0, 0, 0, 0]

        self.endstate = [0, 0, 0] # middle finger, f1, f2
        self.new_poses = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        # here intitial joint states as gripper open pose
        self.current_jointstates = [0.10, 0.02, -0.10, 0.10, 0.02, -0.10, 0.10, 0.02, -0.10]

        self.goal = [0,0,0]

        self.counter = 0
        self.current_g = 0
        self.current_g1 = 0
        self.current_g2 = 0
        
        
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
        rospy.Subscriber('/joint_states', JointState, self.callback_jointstates, queue_size=1)
        rospy.Subscriber('/grasp/goal', tams_ur5_gazebo.msg.graspActionGoal, self.goal_callback, queue_size=1)
        rospy.Subscriber('g', Int16, self.g_callback, queue_size=1)
        rospy.Subscriber('g1', Int16, self.g1_callback, queue_size=1)
        rospy.Subscriber('g2', Int16, self.g2_callback, queue_size=1)

        # create a publisher for the end position joint states
        self.pub_end_estimated_joint_states = rospy.Publisher('/end_estimated_joint_states', JointState, queue_size=1)

    def callback_jointstates(self, states):
        
        estimated_states = self.update_joint_states(deepcopy(states))
        self.pub_estimated.publish(estimated_states)

    def update_joint_states(self, states):  
        estimated_positions = list(states.position)
        # jointstate nachtricht hier new thetas die berechnet werden
        if self.goal[0] > 0 and self.goal[1] > 0 and self.goal[2] > 0 :
            
            if self.rate_counter == 0 : 
                print('new poses pre calc', self.new_poses)
                #self.calculate_new_pose(self.goal[0], self.goal[1], self.goal[2])
                self.calculate_new_pose(self.current_g, self.current_g1, self.current_g2)
                self.js_positions = self.new_poses
                #print('new poses post calc', self.new_poses )
                #print('js pos:', self.js_positions)
                # TODO braucht man diesen counter wirklich?
                if self.counter <= max(self.goal[0], self.goal[1], self.goal[2]):
                    self.counter += 1
                if self.counter == max(self.goal[0], self.goal[1], self.goal[2]): 
                    self.end_estimated_joint_states = JointState()
                    self.end_estimated_joint_states.position = self.new_poses
                    self.pub_end_estimated_joint_states.publish(self.end_estimated_joint_states)   
            self.rate_counter += 0.5
            if self.rate_counter == 9:    # dieses zahl ändern um update rate zu erhöhen/ niedriger machen
                self.rate_counter = 0

            # neue joint states in nachricht eintragen
            for i in range(len(states.name)):
                for y in range(len(self.js_names)):
                    if states.name[i] == self.js_names[y]:
                        estimated_positions[i] = self.js_positions[y]
            states.position = estimated_positions             

        if self.goal[0] == 0 and self.goal[1] == 0 and self.goal[2] == 0 :
            # fill with open position values for jointstate message
            if self.rate_counter == 0 : 
                #self.calculate_new_pose(self.goal[0], self.goal[1], self.goal[2])
                self.calculate_new_pose(self.current_g, self.current_g1, self.current_g2)
                self.js_positions = self.js_open_positions
                # TODO braucht man diesen counter wirklich?
                if self.counter <= max(self.goal[0], self.goal[1], self.goal[2]):
                    self.counter += 1
            self.rate_counter += 0.5
            if self.rate_counter == 9:    # dieses zahl ändern um update rate zu erhöhen/ niedriger machen
                self.rate_counter = 0

            # neue joint states in nachricht eintragen
            for i in range(len(states.name)):
                for y in range(len(self.js_names)):
                    if states.name[i] == self.js_names[y]:
                        estimated_positions[i] = self.js_positions[y]
            states.position = estimated_positions 

            
        
        return states
    
    def g_callback(self, msg):
        self.current_g = msg.data
        print('g', self.current_g)

    def g1_callback(self, msg):
        self.current_g1 = msg.data 
        print('g1', self.current_g1)  

    def g2_callback(self, msg):
        self.current_g2 = msg.data 
        print('g2', self.current_g2)   


    def goal_callback(self, goal:tams_ur5_gazebo.msg.graspActionGoal):
        self.counter = 0
        #self.iterations = max(goal.g, goal.g1, goal.g2)
        self.goal[0] = goal.goal.g
        self.goal[1] = goal.goal.g1
        self.goal[2] = goal.goal.g2

        rospy.logerr('NEW GOAL ----------------------------------------------------------')
        print(self.goal)

                       

    #def action_callback(self, goal:tams_ur5_gazebo.msg.graspActionGoal):
     #   self.iterate_movements(goal.g, goal.g1, goal.g2)    
           
    
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
        print('G F3_XU:', f_3_xu)
        return f_3_xu
    
    def f_3_g1(self, g1):
        theta_3_min =  -1.2 #-1.2217
        theta_3_max = -0.06 #-0.0523 #-0.068:
        m_3 = theta_3_min + ((theta_3_max - theta_3_min)/(255 - g1))  
        f_3_xu = m_3*self.delta_g
        print('G1 F3_XU:', f_3_xu)
        return f_3_xu

    def f_3_g2(self, g2):
        theta_3_min =  -1.2 #-1.2217
        theta_3_max = -0.06 #-0.0523 #-0.068:
        m_3 = theta_3_min + ((theta_3_max - theta_3_min)/(255 - g2))  
        f_3_xu = m_3*self.delta_g
        print('G2 F3_XU:', f_3_xu)
        return f_3_xu

    # hier tuple wieder aufrufen und checken in welcher phase wir sind und delta_thetas berechnen
    def calc_delta_theta(self, g):
        # voerst alle bewegungen u = 1 FIX LATER
        if g == 0:
            u = -1
        else:
            u = 1
        #if phase 1 :
        #print("tuple: ", self.tuple)
        if self.tuple == [0, 0, 0, 0, 0, 0]:
            
            self.delta_g = u
            self.delta_theta_1 = self.f_1()
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = - self.f_1()
            print("phase 1")
            

        #if phase 1':
        elif self.tuple == [0, 0, 0, 0, 0, -1]:
        
            self.delta_g = u
            self.delta_theta_1 = self.f_1()
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = 0.00
            print("phase 1'")

        #if phase 2 1:
        elif self.tuple == [1, 0, 0, 0, 0, 0]:
            
            self.delta_g = u
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = self.f_2()
            self.delta_theta_3 = - self.f_2()
            print("phase 2 1")
            

        #if phase 2 1:
        elif self.tuple[0] == 0 and self.tuple[1] == 0 and self.tuple[2] == 0 and self.tuple[3] == 1 and self.tuple[4] == 0 and self.tuple[5] == 0:
        #[0, 0, 0, 1, 0, 0]:
            
            self.delta_g = u
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = self.f_2()
            self.delta_theta_3 = - self.f_2()
            print("phase 2 1 1")
            

        #if phase 2' 1:
        elif self.tuple == [1, 0, 0, 0, 0, -1]: 
            
            self.delta_g = u
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = self.f_2()
            self.delta_theta_3 = 0.00
            print("phase 2' 1")
            
            

        #if phase 2' 2:
        elif self.tuple == [0, 0, 0, 1, 0, -1]:
            
            self.delta_g = u
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = self.f_2()
            self.delta_theta_3 = 0.00
            print("phase 2' 2")
            

        #if phase 3:
        elif self.tuple[1] == 1 and self.tuple[2] == 0 and self.tuple[4] == 0 and self.tuple[5] == 0:
            #[., 1, 0, ., 0, 0]:
            
            self.delta_g = u
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = 0.00
            gr = g/2
            self.delta_theta_3 = -self.f_3(gr)
            print("phase 3")

        elif self.tuple[1] == 0 and self.tuple[2] == 0 and self.tuple[3] == 1 and self.tuple[4] == 1 and self.tuple[5] == 0: 
            #[., 0, 0, ., 1, 0]:
            
            self.delta_g = u   
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = 0 #-self.f_3(g)
            self.endstate[1] = 1
            print("phase 3 1")

        # neue phase 3': 
        elif self.tuple[1] == 1 and self.tuple[2] == 0 and self.tuple[4] == 0 and self.tuple[5] == -1:
            #[., 1, 0, ., 0, -1]:
            
            self.delta_g = u
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = self.f_3(g)
            print("phase 3'")

            
        #if phase 4:
        # in phase 4 sind alle 3 delta thetas sowie delta g alle = 0
        elif self.tuple[0]== 0 and self.tuple[1] == 0 and self.tuple[2] == 1 and self.tuple[3] == 0 and self.tuple[4] == 0 and self.tuple[5] == 0:
            # [0, 0, 1, 0, 0, 0]
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = 0.00
            self.delta_g = 0.00
            print("phase 4")
            print( "... Middle Finger End State." )
            self.endstate[0] = 1
            #sys.exit()

        elif self.tuple[0]== 1 and self.tuple[1] == 1 and self.tuple[2] == 1 and self.tuple[3] == 0 and self.tuple[4] == 0 and self.tuple[5] == 0:
            # [1, 1, 1, 0, 0, 0]
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = 0.00
            self.delta_g = 0.00
            print("phase 4 1")
            print( "... Middle Finger End State." )
            self.endstate[0] = 1
            #sys.exit()    

        elif self.tuple[0]== 1 and self.tuple[1] == 1 and self.tuple[2] == 0 and self.tuple[3] == 0 and self.tuple[4] == 0 and self.tuple[5] == 1: 
            # [1, 1, 0, 0, 0, 1]
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = 0.00
            self.delta_g = 0.00
            print("phase 4 2")
            print( "... Middle Finger End State." )
            self.endstate[0] = 1
            #sys.exit()

        elif self.tuple[0]== 0 and self.tuple[1] == 1 and self.tuple[2] == 0 and self.tuple[3] == 1 and self.tuple[4] == 0 and self.tuple[5] == 1: 
            # [0, 1, 0, 1, 0, 1]
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = 0.00
            self.delta_g = 0.00
            print("phase 4 3")
            print( "... Middle Finger End State." )
            self.endstate[0] = 1
            #sys.exit()  

            """elif self.tuple[0]== 0 and self.tuple[1] == 1 and self.tuple[2] == 0 and self.tuple[3] == 0 and self.tuple[4] == 0 and self.tuple[5] == 1: 
            # [0, 1, 0, 0, 0, 1]
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = 0.00
            self.delta_g = 0.00
            print("phase 4 4")
            print( "... Middle Finger End State." )
            self.endstate[0] = 1
            #sys.exit()     """

        elif self.tuple[1] == 0 and self.tuple[2] == 0 and self.tuple[4] == 1 and self.tuple[5] == -1: 
            #[., 0, 0, ., 1, -1]:
            
            self.delta_g = u   
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = 0.00
            print("phase 4 5")
            self.endstate[1] = 1     

        elif self.tuple[1] == 0 and self.tuple[2] == 0 and self.tuple[3] == 1 and self.tuple[4] == 1 and self.tuple[5] == -1: 
            #[., 0, 0, 1, 1, -1]:
            
            self.delta_g = u   
            self.delta_theta_1 = 0.00
            self.delta_theta_2 = 0.00
            self.delta_theta_3 = 0.00
            print("phase 4 6")
            self.endstate[1] = 1         

        # neue Phase 5 
        """ elif self.tuple[2] == 0 and self.tuple[5] == -1: 
            self.delta_theta_1 = 0
            self.delta_theta_2 = 0
            self.delta_theta_3 = 0
            self.delta_g = 0
             """
           

    def calc_delta_theta_f1(self, g1):
        # voerst alle bewegungen u = 1 FIX LATER
        if g1 == 0:
            u = -1
        else:
            u = 1
        #if phase 1 :
        #print("tuple_f1: ", self.tuple_f1)
        if self.tuple_f1 == [0, 0, 0, 0, 0, 0]:
            
            self.delta_g = u
            self.f1_delta_theta_1 = self.f_1()
            self.f1_delta_theta_2 = 0.00
            self.f1_delta_theta_3 = - self.f_1()
            print("f1 phase 1")

        #if phase 1':
        elif self.tuple_f1 == [0, 0, 0, 0, 0, -1]:
            
            self.delta_gdelta_g = u
            self.f1_delta_theta_1 = self.f_1()
            self.f1_delta_theta_2 = 0
            self.f1_delta_theta_3 = 0
            print("f1 phase 1'")

        #if phase 2 1:
        elif self.tuple_f1 == [1, 0, 0, 0, 0, 0]:
            
            self.delta_g = u
            self.f1_delta_theta_1 = 0.00
            self.f1_delta_theta_2 = self.f_2()
            self.f1_delta_theta_3 = - self.f_2()
            self.delta_g = u
            print("f1 phase 2 1")
            

        #if phase 2 1:
        elif self.tuple_f1[0] == 0 and self.tuple_f1[1] == 0 and self.tuple_f1[2] == 0 and self.tuple_f1[3] == 1 and self.tuple_f1[4] == 0 and self.tuple_f1[5] == 0:
        #[0, 0, 0, 1, 0, 0]:
            
            self.delta_g = u
            self.f1_delta_theta_1 = 0
            self.f1_delta_theta_2 = self.f_2()
            self.f1_delta_theta_3 = - self.f_2()
            self.delta_g = u
            print("f1 phase 2 1 1")

        #if phase 2' 1:
        elif self.tuple_f1 == [1, 0, 0, 0, 0, -1]: 
            
            self.delta_g = u
            self.f1_delta_theta_1 = 0
            self.f1_delta_theta_2 = self.f_2()
            self.f1_delta_theta_3 = 0
            print("f1 phase 2# 1")
            

        #if phase 2' 2:
        elif self.tuple_f1 == [0, 0, 0, 1, 0, -1]:
            
            self.delta_g = u
            self.f1_delta_theta_1 = 0
            self.f1_delta_theta_2 = self.f_2()
            self.f1_delta_theta_3 = 0
            print("f1 phase 2 2")

        #if phase 3:
        elif self.tuple_f1[1] == 1 and self.tuple_f1[2] == 0 and self.tuple_f1[4] == 0 and self.tuple_f1[5] == 0:
            #[., 1, 0, ., 0, 0]:
            
            self.delta_g = u
            self.f1_delta_theta_1 = 0
            self.f1_delta_theta_2 = 0
            self.f1_delta_theta_3 = -self.f_3_g1(g1)
            print("f1 phase 3")

        elif self.tuple_f1[1] == 0 and self.tuple_f1[2] == 0 and self.tuple_f1[3] == 1 and self.tuple_f1[4] == 1 and self.tuple_f1[5] == 0:
            #[., 0, 0, ., 1, 0]:
            
            self.delta_g = u    
            self.f1_delta_theta_1 = 0
            self.f1_delta_theta_2 = 0
            self.f1_delta_theta_3 = 0 #-self.f_3_g1(g1)
            self.endstate[1] = 1
            print("f1 phase 3 1 ")

        # neue phase 3': 
        elif self.tuple_f1[1] == 1 and self.tuple_f1[2] == 0 and self.tuple_f1[4] == 0 and self.tuple_f1[5] == -1:
            #[., 1, 0, ., 0, -1]:
            
            self.delta_g = u
            self.f1_delta_theta_1 = 0.00
            self.f1_delta_theta_2 = 0.00
            self.f1_delta_theta_3 = self.f_3_g1(g1)
            print("f1 phase 3'")

   
            
        #if phase 4:
        # in phase 4 sind alle 3 delta thetas sowie delta g alle = 0
        elif self.tuple_f1[0]== 0 and self.tuple_f1[1] == 0 and self.tuple_f1[2] == 1 and self.tuple_f1[3] == 0 and self.tuple_f1[4] == 0 and self.tuple_f1[5] == 0:
            # [0, 0, 1, 0, 0, 0]
            self.f1_delta_theta_1 = 0.00
            self.f1_delta_theta_2 = 0.00
            self.f1_delta_theta_3 = 0.00
            self.delta_g = 0.00
            print("phase 4")
            print( "... Middle Finger End State." )
            self.endstate[0] = 1
            #sys.exit()

        elif self.tuple_f1[0]== 1 and self.tuple_f1[1] == 1 and self.tuple_f1[2] == 1 and self.tuple_f1[3] == 0 and self.tuple_f1[4] == 0 and self.tuple_f1[5] == 0:
            # [1, 1, 1, 0, 0, 0]
            self.f1_delta_theta_1 = 0.00
            self.f1_delta_theta_2 = 0.00
            self.f1_delta_theta_3 = 0.00
            self.delta_g = 0.00
            print("phase 4 1")
            print( "... Middle Finger End State." )
            self.endstate[0] = 1
            #sys.exit()    

        elif self.tuple_f1[0]== 1 and self.tuple_f1[1] == 1 and self.tuple_f1[2] == 0 and self.tuple_f1[3] == 0 and self.tuple_f1[4] == 0 and self.tuple_f1[5] == 1: 
            # [1, 1, 0, 0, 0, 1]
            self.f1_delta_theta_1 = 0.00
            self.f1_delta_theta_2 = 0.00
            self.f1_delta_theta_3 = 0.00
            self.delta_g = 0.00
            print("phase 4 2")
            print( "... Middle Finger End State." )
            self.endstate[0] = 1
            #sys.exit()

        elif self.tuple_f1[0]== 0 and self.tuple_f1[1] == 1 and self.tuple_f1[2] == 0 and self.tuple_f1[3] == 1 and self.tuple_f1[4] == 0 and self.tuple_f1[5] == 1: 
            # [0, 1, 0, 1, 0, 1]
            self.f1_delta_theta_1 = 0.00
            self.f1_delta_theta_2 = 0.00
            self.f1_delta_theta_3 = 0.00
            self.delta_g = 0.00
            print("phase 4 3")
            print( "... Middle Finger End State." )
            self.endstate[0] = 1
            #sys.exit()  

            """elif self.tuple_f1[0]== 0 and self.tuple_f1[1] == 1 and self.tuple_f1[2] == 0 and self.tuple_f1[3] == 0 and self.tuple_f1[4] == 0 and self.tuple_f1[5] == 1: 
            # [0, 1, 0, 0, 0, 1]
            self.f1_delta_theta_1 = 0.00
            self.f1_delta_theta_2 = 0.00
            self.f1_delta_theta_3 = 0.00
            self.delta_g = 0.00
            print("phase 4 4")
            print( "... Middle Finger End State." )
            self.endstate[0] = 1
            #sys.exit()     """

        elif self.tuple_f1[1] == 0 and self.tuple_f1[2] == 0 and self.tuple_f1[4] == 1 and self.tuple_f1[5] == -1: 
            #[., 0, 0, ., 1, -1]:
            
            self.delta_g = u   
            self.f1_delta_theta_1 = 0.00
            self.f1_delta_theta_2 = 0.00
            self.f1_delta_theta_3 = 0.00
            print("phase 4 5")
            self.endstate[1] = 1     

        elif self.tuple_f1[1] == 0 and self.tuple_f1[2] == 0 and self.tuple_f1[3] == 1 and self.tuple_f1[4] == 1 and self.tuple_f1[5] == -1: 
            #[., 0, 0, 1, 1, -1]:
            
            self.delta_g = u   
            self.f1_delta_theta_1 = 0.00
            self.f1_delta_theta_2 = 0.00
            self.f1_delta_theta_3 = 0.00
            print("phase 4 6")
            self.endstate[1] = 1  

        # neue Phase 5 
        """ elif self.tuple_f1[2] == 0 and self.tuple_f1[5] == -1: 
            self.f1_delta_theta_1 = 0
            self.f1_delta_theta_2 = 0
            self.f1_delta_theta_3 = 0
            self.delta_g = 0
             """

    def calc_delta_theta_f2(self, g2):
        # voerst alle bewegungen u = 1 FIX LATER
        if g2 == 0:
            u = -1
        else:
            u = 1
        #if phase 1 :
        #print("tuple_f2: ", self.tuple_f2)
        if self.tuple_f2 == [0, 0, 0, 0, 0, 0]:
            self.f2_delta_theta_1 = self.f_1()
            self.f2_delta_theta_2 = 0
            self.f2_delta_theta_3 = - self.f_1()
            self.delta_g = u
            print("f2 phase 1")

        #if phase 1':
        elif self.tuple_f2 == [0, 0, 0, 0, 0, -1]:
            self.f2_delta_theta_1 = self.f_1()
            self.f2_delta_theta_2 = 0
            self.f2_delta_theta_3 = 0
            self.delta_g = u
            print("f2 phase 1'")

        #if phase 2 1:
        elif self.tuple_f2 == [1, 0, 0, 0, 0, 0]:
            self.f2_delta_theta_1 = 0
            self.f2_delta_theta_2 = self.f_2()
            self.f2_delta_theta_3 = - self.f_2()
            self.delta_g = u
            print("f2 phase 2 1")
            

        #if phase 2 1:
        elif self.tuple_f2[0] == 0 and self.tuple_f2[1] == 0 and self.tuple_f2[2] == 0 and self.tuple_f2[3] == 1 and self.tuple_f2[4] == 0 and self.tuple_f2[5] == 0:
        #[0, 0, 0, 1, 0, 0]:
            self.f2_delta_theta_1 = 0
            self.f2_delta_theta_2 = self.f_2()
            self.f2_delta_theta_3 = - self.f_2()
            self.delta_g = u
            print("f2 phase 2 1 1")
            

        #if phase 2' 1:
        elif self.tuple_f2 == [1, 0, 0, 0, 0, -1]: 
            self.f2_delta_theta_1 = 0
            self.f2_delta_theta_2 = self.f_2()
            self.f2_delta_theta_3 = 0
            self.delta_g = u
            print("f2 phase 2# 1 ")
            
            

        #if phase 2' 2:
        elif self.tuple_f2 == [0, 0, 0, 1, 0, -1]:
            self.f2_delta_theta_1 = 0
            self.f2_delta_theta_2 = self.f_2()
            self.f2_delta_theta_3 = 0
            self.delta_g = u
            print("f2 phase 2' 2")
            

        #if phase 3:
        elif self.tuple_f2[1] == 1 and self.tuple_f2[2] == 0 and self.tuple_f2[4] == 0 and self.tuple_f2[5] == 0:
            
            self.delta_g = u
            self.f2_delta_theta_1 = 0
            self.f2_delta_theta_2 = 0
            print('-1 wert pre calling F3_XU function:', self.new_pos_finger_2_joint_3)
            self.f2_delta_theta_3 = -self.f_3_g2(g2)
            print("f2 phase 3")

        elif self.tuple_f2[1] == 0 and self.tuple_f2[2] == 0 and self.tuple_f2[3] == 1 and self.tuple_f2[4] == 1 and self.tuple_f2[5] == 0:    
            
            self.delta_g = u
            self.f2_delta_theta_1 = 0
            self.f2_delta_theta_2 = 0
            print('0 wert pre calling F3_XU function:', self.new_pos_finger_2_joint_3)
            self.f2_delta_theta_3 = 0 #-self.f_3_g2(g2)
            self.endstate[1] = 1
            print("f2 phase 3 1")

        # neue phase 3': 
        elif self.tuple_f2[1] == 1 and self.tuple_f2[2] == 0 and self.tuple_f2[4] == 0 and self.tuple_f2[5] == -1:
            #[., 1, 0, ., 0, -1]:
            
            self.delta_g = u
            self.f2_delta_theta_1 = 0.00
            self.f2_delta_theta_2 = 0.00
            print('1 wert pre calling F3_XU function:', self.new_pos_finger_2_joint_3)
            self.f2_delta_theta_3 = self.f_3_g2(g2)
            print("f2 phase 3'")

        elif self.tuple_f2[1] == 0 and self.tuple_f2[2] == 0 and self.tuple_f2[4] == 1 and self.tuple_f2[5] == -1: 
            #[., 0, 0, ., 1, -1]:
            
            self.delta_g = u   
            self.f2_delta_theta_1 = 0.00
            self.f2_delta_theta_2 = 0.00
            print('2 wert pre calling F3_XU function:', self.new_pos_finger_2_joint_3)
            self.f2_delta_theta_3 = -self.f_3_g2(g2)
            print("f2 phase 3' 1")  
            
        #if phase 4:
        # in phase 4 sind alle 3 delta thetas sowie delta g alle = 0
        elif self.tuple_f2[0]== 0 and self.tuple_f2[1] == 0 and self.tuple_f2[2] == 1 and self.tuple_f2[3] == 0 and self.tuple_f2[4] == 0 and self.tuple_f2[5] == 0:
            # [0, 0, 1, 0, 0, 0]
            self.f2_delta_theta_1 = 0.00
            self.f2_delta_theta_2 = 0.00
            self.f2_delta_theta_3 = 0.00
            self.delta_g = 0.00
            print("phase 4")
            print( "... Middle Finger End State." )
            self.endstate[0] = 1
            #sys.exit()

        elif self.tuple_f2[0]== 1 and self.tuple_f2[1] == 1 and self.tuple_f2[2] == 1 and self.tuple_f2[3] == 0 and self.tuple_f2[4] == 0 and self.tuple_f2[5] == 0:
            # [1, 1, 1, 0, 0, 0]
            self.f2_delta_theta_1 = 0.00
            self.f2_delta_theta_2 = 0.00
            self.f2_delta_theta_3 = 0.00
            self.delta_g = 0.00
            print("phase 4 1")
            print( "... Middle Finger End State." )
            self.endstate[0] = 1
            #sys.exit()    

        elif self.tuple_f2[0]== 1 and self.tuple_f2[1] == 1 and self.tuple_f2[2] == 0 and self.tuple_f2[3] == 0 and self.tuple_f2[4] == 0 and self.tuple_f2[5] == 1: 
            # [1, 1, 0, 0, 0, 1]
            self.f2_delta_theta_1 = 0.00
            self.f2_delta_theta_2 = 0.00
            self.f2_delta_theta_3 = 0.00
            self.delta_g = 0.00
            print("phase 4 2")
            print( "... Middle Finger End State." )
            self.endstate[0] = 1
            #sys.exit()

        elif self.tuple_f2[0]== 0 and self.tuple_f2[1] == 1 and self.tuple_f2[2] == 0 and self.tuple_f2[3] == 1 and self.tuple_f2[4] == 0 and self.tuple_f2[5] == 1: 
            # [0, 1, 0, 1, 0, 1]
            self.f2_delta_theta_1 = 0.00
            self.f2_delta_theta_2 = 0.00
            self.f2_delta_theta_3 = 0.00
            self.delta_g = 0.00
            print("phase 4 3")
            print( "... Middle Finger End State." )
            self.endstate[0] = 1
            #sys.exit()  

            """elif self.tuple_f2[0]== 0 and self.tuple_f2[1] == 1 and self.tuple_f2[2] == 0 and self.tuple_f2[3] == 0 and self.tuple_f2[4] == 0 and self.tuple_f2[5] == 1: 
            # [0, 1, 0, 0, 0, 1]
            self.f2_delta_theta_1 = 0.00
            self.f2_delta_theta_2 = 0.00
            self.f2_delta_theta_3 = 0.00
            self.delta_g = 0.00
            print("phase 4 4")
            print( "... Middle Finger End State." )
            self.endstate[0] = 1
            #sys.exit()     """

        elif self.tuple_f2[1] == 0 and self.tuple_f2[2] == 0 and self.tuple_f2[4] == 1 and self.tuple_f2[5] == -1: 
            #[., 0, 0, ., 1, -1]:
            
            self.delta_g = u   
            self.f2_delta_theta_1 = 0.00
            self.f2_delta_theta_2 = 0.00
            self.f2_delta_theta_3 = 0.00
            print("phase 4 5")
            self.endstate[1] = 1     

        elif self.tuple_f2[1] == 0 and self.tuple_f2[2] == 0 and self.tuple_f2[3] == 1 and self.tuple_f2[4] == 1 and self.tuple_f2[5] == -1: 
            #[., 0, 0, 1, 1, -1]:
            
            self.delta_g = u   
            self.f2_delta_theta_1 = 0.00
            self.f2_delta_theta_2 = 0.00
            self.f2_delta_theta_3 = 0.00
            print("phase 4 6")
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
        self.new_pos_finger_middle_joint_1 = self.delta_theta_1 + self.current_jointstates[6] # position 6 is for 1st joint for middle finger
        self.new_pos_finger_middle_joint_2 = self.delta_theta_2 + self.current_jointstates[7] # secont joint middle finger
        self.new_pos_finger_middle_joint_3 = self.delta_theta_3 + self.current_jointstates[8]
        #if self.new_pos_finger_middle_joint_3 > -0.06:
         #   self.new_pos_finger_middle_joint_3 = -0.06

        # new positions finger 1
        self.new_pos_finger_1_joint_1 = self.f1_delta_theta_1 + self.current_jointstates[0] # position 6 is for 1st joint for 1 finger
        self.new_pos_finger_1_joint_2 = self.f1_delta_theta_2 + self.current_jointstates[1] # second joint 1 finger
        self.new_pos_finger_1_joint_3 = self.f1_delta_theta_3 + self.current_jointstates[2]
        #if self.new_pos_finger_1_joint_3 > -0.06:
         #   self.new_pos_finger_1_joint_3 = -0.06

        # new positions finger 2
        self.new_pos_finger_2_joint_1 = self.f2_delta_theta_1 + self.current_jointstates[3] # position 6 is for 1st joint for 1 finger
        self.new_pos_finger_2_joint_2 = self.f2_delta_theta_2 + self.current_jointstates[4] # secont joint 1 finger
        self.new_pos_finger_2_joint_3 = self.f2_delta_theta_3 + self.current_jointstates[5]
        #if self.new_pos_finger_2_joint_3 > -0.06:
         #   self.new_pos_finger_2_joint_3 = -0.06

        
        new_thetas = [self.new_pos_finger_1_joint_1, self.new_pos_finger_1_joint_2, self.new_pos_finger_1_joint_3, 
                      self.new_pos_finger_2_joint_1, self.new_pos_finger_2_joint_2, self.new_pos_finger_2_joint_3, 
                      self.new_pos_finger_middle_joint_1, self.new_pos_finger_middle_joint_2, self.new_pos_finger_middle_joint_3]

        data_to_send = Float32MultiArray()
        data_to_send.data = new_thetas
        self.callback5(data_to_send)

        self.new_poses = [self.new_pos_finger_1_joint_1, self.new_pos_finger_1_joint_2, self.new_pos_finger_1_joint_3, 
                      self.new_pos_finger_2_joint_1, self.new_pos_finger_2_joint_2, self.new_pos_finger_2_joint_3, 
                      self.new_pos_finger_middle_joint_1, self.new_pos_finger_middle_joint_2, self.new_pos_finger_middle_joint_3, 0, 0]
        
        
        

        
        if self.counter <= g1:
           self.current_jointstates[0] = self.new_poses[0]
           self.current_jointstates[1] = self.new_poses[1]     
           self.current_jointstates[2] = self.new_poses[2]          
        if self.counter <= g2:
           self.current_jointstates[3] = self.new_poses[3]
           self.current_jointstates[4] = self.new_poses[4]     
           self.current_jointstates[5] = self.new_poses[5] 
        if self.counter <= g:        
           self.current_jointstates[6] = self.new_poses[6]
           self.current_jointstates[7] = self.new_poses[7]     
           self.current_jointstates[8] = self.new_poses[8] 

           # this is needed or else the hand will continue in previous last closing position after opening  
        if g1 == 0:
           self.current_jointstates[0] = 0.10
           self.current_jointstates[1] = 0.02
           self.current_jointstates[2] = -0.10         
        if g2 == 0:
           self.current_jointstates[3] = 0.10
           self.current_jointstates[4] = 0.02
           self.current_jointstates[5] = -0.10
        if g == 0:   
           self.current_jointstates[6] = 0.10
           self.current_jointstates[7] = 0.02   
           self.current_jointstates[8] = -0.10  

#[0.10, 0.02, -0.10, 0.10, 0.02, -0.10, 0.10, 0.02, -0.10]
    
if __name__ == "__main__":
    rospy.init_node('alles', anonymous=True)
    rospy.sleep(0.2) 

    newTuple = TupleClass()

    rospy.spin()

   
    