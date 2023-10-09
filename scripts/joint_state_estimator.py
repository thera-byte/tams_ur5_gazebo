#!/usr/bin/env python
#
# script that subscribes to sensor topics on the robotiq fingers and builds tuples from the collision data

import rospy
import tams_ur5_gazebo.msg 

from std_msgs.msg import Int16
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from copy import deepcopy
from tams_ur5_gazebo_lib import theta_calculator

# create and fill tuple with collisions and limits
class TupleClass:
    def __init__(self):
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

        self.end_js_publish = True

        self.theta_c = theta_calculator.ThetaCalculator()

        self.js_names = [ "s_model_finger_1_joint_1", "s_model_finger_1_joint_2", "s_model_finger_1_joint_3", \
                "s_model_finger_2_joint_1", "s_model_finger_2_joint_2", "s_model_finger_2_joint_3", \
                "s_model_finger_middle_joint_1", "s_model_finger_middle_joint_2", "s_model_finger_middle_joint_3", \
                "s_model_palm_finger_1_joint", "s_model_palm_finger_2_joint" ]
        self.js_positions = [0.10, 0.02, -0.10, 0.10, 0.02, -0.10, 0.10, 0.02, -0.10, 0, 0] # gripper open position inital werte
        self.js_open_positions = [0.10, 0.02, -0.10, 0.10, 0.02, -0.10, 0.10, 0.02, -0.10, 0, 0] # gripper open position inital werte
        
        # diese variable ist zur synchronisierung von gazebo und diesem skript da, zum publishen der js 
        # das is nur nen fix. wird so nicht mit der echten hand funktionieren. dadurch dass jede iteration mit moveit bewegt wird ist die simulation extram langsam
        # die hand soll sich in sim eigentlich genau so schnell bewegen wie die echte hand und dieses skript die js genau so schnell estimaten
        self.rate_counter = 0
        
        self.pub_estimated = rospy.Publisher('/estimated_joint_states', JointState, queue_size=1)

        
        
        
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
                print('js pos:', self.js_positions)
                # TODO braucht man diesen counter wirklich?
                """
                if self.counter <= max(self.goal[0], self.goal[1], self.goal[2]):
                    self.counter += 1
                    print("COUNTER:", self.counter)
                if self.counter == max(self.goal[0], self.goal[1], self.goal[2]): 
                    self.end_estimated_joint_states = JointState()
                    self.end_estimated_joint_states.position = self.new_poses
                    self.pub_end_estimated_joint_states.publish(self.end_estimated_joint_states) """  
            self.rate_counter += 0.5
            if self.rate_counter == 8:    # dieses zahl ändern um update rate zu erhöhen/ niedriger machen
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
                #if self.counter <= max(self.goal[0], self.goal[1], self.goal[2]):
                    #self.counter += 1
                # damit endstate reseted bleibt beim öffnen    
                self.endstate = [0,0,0]    
            self.rate_counter += 0.5
            if self.rate_counter == 8:    # dieses zahl ändern um update rate zu erhöhen/ niedriger machen
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
        self.end_js_publish = True
        self.goal[0] = goal.goal.g
        self.goal[1] = goal.goal.g1
        self.goal[2] = goal.goal.g2

        rospy.logerr('NEW GOAL ----------------------------------------------------------')
        print(self.goal)        
    
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

    def check_joint_1(self, joint_state):
        if joint_state <= 0.0495:
            return 0.0495, 1
        elif joint_state >= 1.20:
            return 1.20, 1
        return joint_state, 0

    def check_joint_2(self, joint_state):
        if joint_state <= 0.0:
            return 0.0, 1
        elif joint_state >= 1.526:
            return 1.526, 1
        return joint_state, 0 

    def check_joint_3(self, joint_state):
        if joint_state <= -1.20:
            return -1.20, -1
        elif joint_state >= -0.06:
            return -0.06, 1
        return joint_state, 0
    
    # hier delta thetas auf actuelle position aufrechnen um neue goal positionen zu erhalten
    #def calculate_new_pose(self, _timer_event):
    def calculate_new_pose(self, g, g1, g2):    
        print("Middle Finger")
        self.delta_theta_1, self.delta_theta_2, self.delta_theta_3, self.delta_g, self.endstate[0]  = self.theta_c.calc_delta(g, self.tuple, self.endstate[0])
        #print("delta_theta_2", self.delta_theta_2)
        print("Finger 1")
        self.f1_delta_theta_1, self.f1_delta_theta_2, self.f1_delta_theta_3, self.delta_g, self.endstate[1]  = self.theta_c.calc_delta(g1, self.tuple_f1, self.endstate[1])
        #print("f1_delta_theta_2", self.f1_delta_theta_2)
        print("Finger 2")
        self.f2_delta_theta_1, self.f2_delta_theta_2, self.f2_delta_theta_3, self.delta_g, self.endstate[2]  = self.theta_c.calc_delta(g2, self.tuple_f2, self.endstate[2])
        #print("f2_delta_theta_2", self.f2_delta_2)
        # new positions middle finger
        new_pos_finger_middle_joint_1, self.tuple[3] = self.check_joint_1(self.delta_theta_1 + self.current_jointstates[6]) # position 6 is for 1st joint for middle finger
        new_pos_finger_middle_joint_2, self.tuple[4] = self.check_joint_2(self.delta_theta_2 + self.current_jointstates[7]) # secont joint middle finger
        new_pos_finger_middle_joint_3, self.tuple[5] = self.check_joint_3(self.delta_theta_3 + self.current_jointstates[8])

        # new positions finger 1
        new_pos_finger_1_joint_1, self.tuple_f1[3] = self.check_joint_1(self.f1_delta_theta_1 + self.current_jointstates[0]) # position 6 is for 1st joint for 1 finger
        new_pos_finger_1_joint_2, self.tuple_f1[4] = self.check_joint_2(self.f1_delta_theta_2 + self.current_jointstates[1]) # second joint 1 finger
        new_pos_finger_1_joint_3, self.tuple_f1[5] = self.check_joint_3(self.f1_delta_theta_3 + self.current_jointstates[2])

        # new positions finger 2
        new_pos_finger_2_joint_1, self.tuple_f2[3] = self.check_joint_1(self.f2_delta_theta_1 + self.current_jointstates[3]) # position 6 is for 1st joint for 1 finger
        new_pos_finger_2_joint_2, self.tuple_f2[4] = self.check_joint_2(self.f2_delta_theta_2 + self.current_jointstates[4]) # secont joint 1 finger
        new_pos_finger_2_joint_3, self.tuple_f2[5] = self.check_joint_3(self.f2_delta_theta_3 + self.current_jointstates[5])
        
        self.new_poses = [new_pos_finger_1_joint_1, new_pos_finger_1_joint_2, new_pos_finger_1_joint_3, 
                      new_pos_finger_2_joint_1, new_pos_finger_2_joint_2, new_pos_finger_2_joint_3, 
                      new_pos_finger_middle_joint_1, new_pos_finger_middle_joint_2, new_pos_finger_middle_joint_3, 0, 0]
        

        
        #if self.counter <= g1:
        if self.current_g1 <= self.goal[0]:   
           self.current_jointstates[0] = self.new_poses[0]
           self.current_jointstates[1] = self.new_poses[1]     
           self.current_jointstates[2] = self.new_poses[2]          
        #if self.counter <= g2:
        if self.current_g2 <= self.goal[1]:  
           self.current_jointstates[3] = self.new_poses[3]
           self.current_jointstates[4] = self.new_poses[4]     
           self.current_jointstates[5] = self.new_poses[5] 
        #if self.counter <= g:
        if self.current_g <= self.goal[2]:          
           self.current_jointstates[6] = self.new_poses[6]
           self.current_jointstates[7] = self.new_poses[7]     
           self.current_jointstates[8] = self.new_poses[8] 

        #if self.current_g <= max(self.goal[0], self.goal[1], self.goal[2]):
         #   self.counter += 1
          #  print("COUNTER:", self.counter)
        if (max(self.current_g, self.current_g1, self.current_g2) == max(self.goal[0], self.goal[1], self.goal[2]) and max(self.goal[0], self.goal[1], self.goal[2]) > 0 ) \
            or (max(self.goal[0], self.goal[1], self.goal[2]) > 0 and self.endstate == [1,1,1]): 
            print("CURRENT Gs: -------------------------------------------------", self.current_g, self.current_g1, self.current_g2)
            print("GOALs: ............................................", self.goal[0], self.goal[1], self.goal[2])
            print("ENDSTATE: __________________________________________", self.endstate)
            #if max(self.goal[0], self.goal[1], self.goal[2]) > 0: 
            #if max(self.current_g, self.current_g1, self.current_g2) > 0:
            print("END JS PUB", self.end_js_publish)
            if self.end_js_publish == True:
                self.end_estimated_joint_states = JointState()
                self.end_estimated_joint_states.position = self.new_poses
                self.pub_end_estimated_joint_states.publish(self.end_estimated_joint_states)
                self.end_js_publish = False
                self.endstate = [0,0,0]

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

        
    
if __name__ == "__main__":
    rospy.init_node('alles', anonymous=True)
    rospy.sleep(0.2) 

    newTuple = TupleClass()

    rospy.spin()

   
    