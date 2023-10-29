#!/usr/bin/env python
#
# script that subscribes to sensor topics on the robotiq fingers and builds tuples from the collision data

import rospy
import moveit_commander
import actionlib
import tams_ur5_gazebo.msg 

from std_msgs.msg import Int16
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import JointState
from tams_ur5_gazebo_lib import theta_calculator

# create and fill tuple with collisions and limits
class MoveFingers:
    def __init__(self):
        self.theta_c = theta_calculator.ThetaCalculator() 

        # move group gripper
        gripper_group_name = "gripper"
        self.gripper_move_group = moveit_commander.MoveGroupCommander( gripper_group_name )
        self.gripper_move_group.set_max_velocity_scaling_factor( 1.0 )
        self.gripper_move_group.set_max_acceleration_scaling_factor( 1.0 )

        self.tuple = [0, 0, 0, 0, 0, 0]
        self.tuple_f1 = [0, 0, 0, 0, 0, 0]
        self.tuple_f2 = [0, 0, 0, 0, 0, 0]

        self.endstate = [0, 0, 0] # middle finger, f1, f2
        
        # create a publisher for g values
        self.pub_g = rospy.Publisher('/g', Int16, queue_size=1 )
        self.pub_g1 = rospy.Publisher('/g1', Int16, queue_size=1 )
        self.pub_g2 = rospy.Publisher('/g2', Int16, queue_size=1 )
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
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback, queue_size=10)
        # publisher for end joint states
        self.pub_end_joint_states = rospy.Publisher('/end_joint_states', JointState, queue_size=1)
        # publisher for endstates - which indicate when a finger is done moving
        self.pub_endstate_movement_middle = rospy.Publisher('/endstate_movement_middle', Int16, queue_size=10)
        self.pub_endstate_movement_finger_1 = rospy.Publisher('/endstate_movement_finger_1', Int16, queue_size=10)
        self.pub_endstate_movement_finger_2 = rospy.Publisher('/endstate_movement_finger_2', Int16, queue_size=10)

        self._as = actionlib.SimpleActionServer('grasp', tams_ur5_gazebo.msg.graspAction, execute_cb=self.action_callback, auto_start = False)
        self._as.start()

        
    def iterate_movements(self, g, g1, g2):
        iterations = max(g, g1, g2)
        print("ITERATIONS", iterations)
        self.counter = 0
        g_msg = Int16()
        current_m = 0
        current_m1 = 0
        current_m2 = 0
        start_time = rospy.Time.now()
        while self.counter <= iterations and not self.endstate == [1,1,1] and (rospy.Time.now() - start_time).to_sec() < 120:
            rospy.logerr('--------------------------------------------')
            print("COUNTER:", self.counter)
            new_poses = self.calculate_new_pose(g, g1, g2)
            self.move_to_pose(g, g1, g2, new_poses)
            self.counter += 1

            # g werte publishen da unklar wie die g werte auf der echten hand gepublisht werden
            
            if self.endstate[0]== 0:
                current_m = self.counter
            g_msg.data = int(current_m) 
            self.pub_g.publish(g_msg)

            if self.endstate[1]== 0:
                current_m1 = self.counter
            g_msg.data = int(current_m1)  
            self.pub_g1.publish(g_msg)

            if self.endstate[2]== 0:
                current_m2 = self.counter
            g_msg.data = int(current_m2)  
            self.pub_g2.publish(g_msg)

            if self.endstate[0] == 1 and max(g, g1, g2) > 0:
                print("ENDSTAAAAATE G", self.endstate[0])
                self.pub_endstate_movement_middle.publish(self.endstate[0])
            #elif self.endstate[0] == 0:
             #   self.pub_endstate_movement_middle.publish(self.endstate[0])

            if self.endstate[1] == 1 and max(g, g1, g2) > 0:
                print("ENDSTAAAAATE G1", self.endstate[1])
                self.pub_endstate_movement_finger_1.publish(self.endstate[1])
            #elif self.endstate[1] == 0:
             #   self.pub_endstate_movement_finger_1.publish(self.endstate[1])

            if self.endstate[2] == 1 and max(g, g1, g2) > 0:
                print("ENDSTAAAAATE G2", self.endstate[2])
                self.pub_endstate_movement_finger_2.publish(self.endstate[2])
            #elif self.endstate[2] == 0:
             #   self.pub_endstate_movement_finger_2.publish(self.endstate[2])            


        if self.counter >= iterations or self.endstate == [1,1,1]:
            print("------------------------------- max g oder all endstates reached - stopping movement of gripper ------------------------------")
            
        
            if max(g, g1, g2) == 0:
                self.endstate = [0,0,0]

                # publish endstates for estimator to stop movement
                self.pub_endstate_movement_middle.publish(self.endstate[0])
                self.pub_endstate_movement_finger_1.publish(self.endstate[1])
                self.pub_endstate_movement_finger_2.publish(self.endstate[2])


            # publish the end joint states
            if max(g, g1, g2) > 0:
                self.end_joint_states = JointState()
                self.end_joint_states.position = new_poses
                self.pub_end_joint_states.publish(self.end_joint_states)

                # publish endstates for estimator to stop movement
                self.pub_endstate_movement_middle.publish(self.endstate[0])
                self.pub_endstate_movement_finger_1.publish(self.endstate[1])
                self.pub_endstate_movement_finger_2.publish(self.endstate[2])

                

            self.tuple = [0,0,0,0,0,0]
            self.tuple_f1 = [0,0,0,0,0,0]
            self.tuple_f2 = [0,0,0,0,0,0]
            self.endstate[0] = 0
            self.endstate[1] = 0
            self.endstate[2] = 0
            self.counter = 0      

            

            self._as.set_succeeded(tams_ur5_gazebo.msg.graspActionResult())
        else:
            print('ABORTED')
            self._as.set_aborted(tams_ur5_gazebo.msg.graspActionResult())
       
    def action_callback(self, goal:tams_ur5_gazebo.msg.graspActionGoal):
        self.iterate_movements(goal.g, goal.g1, goal.g2)
    
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

    def joint_state_callback(self, data):   
        self.jointstates = data


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
        

    # hier tuple wieder aufrufen und checken in welcher phase wir sind und delta_thetas berechnen
    # hier delta thetas auf actuelle position aufrechnen um neue goal positionen zu erhalten
    #def calculate_new_pose(self, _timer_event):
    def calculate_new_pose(self, g, g1, g2):    

        print("Middle Finger")
        self.delta_theta_1, self.delta_theta_2, self.delta_theta_3, self.delta_g, self.endstate[0]  = self.theta_c.calc_delta(g, self.tuple, self.endstate[0])
        print("Finger 1")
        self.f1_delta_theta_1, self.f1_delta_theta_2, self.f1_delta_theta_3, self.delta_g, self.endstate[1]  = self.theta_c.calc_delta(g1, self.tuple_f1, self.endstate[1])
        print("Finger 2")
        self.f2_delta_theta_1, self.f2_delta_theta_2, self.f2_delta_theta_3, self.delta_g, self.endstate[2]  = self.theta_c.calc_delta(g2, self.tuple_f2, self.endstate[2])
        # new positions middle finger
        new_pos_finger_middle_joint_1, self.tuple[3] = self.check_joint_1(self.delta_theta_1 + self.jointstates.position[6]) # position 6 is for 1st joint for middle finger
        new_pos_finger_middle_joint_2, self.tuple[4] = self.check_joint_2(self.delta_theta_2 + self.jointstates.position[7]) # secont joint middle finger
        new_pos_finger_middle_joint_3, self.tuple[5] = self.check_joint_3(self.delta_theta_3 + self.jointstates.position[8])

        # new positions finger 1
        new_pos_finger_1_joint_1, self.tuple_f1[3] = self.check_joint_1(self.f1_delta_theta_1 + self.jointstates.position[0]) # position 6 is for 1st joint for 1 finger
        new_pos_finger_1_joint_2, self.tuple_f1[4] = self.check_joint_2(self.f1_delta_theta_2 + self.jointstates.position[1]) # second joint 1 finger
        new_pos_finger_1_joint_3, self.tuple_f1[5] = self.check_joint_3(self.f1_delta_theta_3 + self.jointstates.position[2])

        # new positions finger 2
        new_pos_finger_2_joint_1, self.tuple_f2[3] = self.check_joint_1(self.f2_delta_theta_1 + self.jointstates.position[3]) # position 6 is for 1st joint for 1 finger
        new_pos_finger_2_joint_2, self.tuple_f2[4] = self.check_joint_2(self.f2_delta_theta_2 + self.jointstates.position[4]) # secont joint 1 finger
        new_pos_finger_2_joint_3, self.tuple_f2[5] = self.check_joint_3(self.f2_delta_theta_3 + self.jointstates.position[5])
        
        return [new_pos_finger_1_joint_1, new_pos_finger_1_joint_2, new_pos_finger_1_joint_3, 
                      new_pos_finger_2_joint_1, new_pos_finger_2_joint_2, new_pos_finger_2_joint_3, 
                      new_pos_finger_middle_joint_1, new_pos_finger_middle_joint_2, new_pos_finger_middle_joint_3]
        

    def move_to_pose(self,g, g1, g2, new_poses): 
        """if new_poses[2] >= -0.06:
            self.tuple_f1[5] = 1
        if new_poses[5] >= -0.06:
            self.tuple_f2[5] = 1
        if new_poses[8] >= -0.06:
            self.tuple[5] = 1 """

        new_pos_finger_1_joint_1 = new_poses[0]  
        new_pos_finger_1_joint_2 = new_poses[1] 
        new_pos_finger_1_joint_3 = new_poses[2] 
        new_pos_finger_2_joint_1 = new_poses[3] 
        new_pos_finger_2_joint_2 = new_poses[4] 
        new_pos_finger_2_joint_3 = new_poses[5] 
        new_pos_finger_middle_joint_1 = new_poses[6] 
        new_pos_finger_middle_joint_2 = new_poses[7] 
        new_pos_finger_middle_joint_3 = new_poses[8] 

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
            print("f1 new positions:", self.jointstates.position[0], self.jointstates.position[1], self.jointstates.position[2] )
            new_pos_finger_1_joint_1 = self.jointstates.position[0]  
            new_pos_finger_1_joint_2 = self.jointstates.position[1] 
            new_pos_finger_1_joint_3 = self.jointstates.position[2]
        elif self.endstate == [0, 1, 1]:
            print("finger 1 and 2 endstate get different new positions now", self.endstate)
            print("f1 new positions:", self.jointstates.position[0], self.jointstates.position[1], self.jointstates.position[2] )
            new_pos_finger_1_joint_1 = self.jointstates.position[0]  
            new_pos_finger_1_joint_2 = self.jointstates.position[1] 
            new_pos_finger_1_joint_3 = self.jointstates.position[2]
            print("f2 new positions:", self.jointstates.position[3], self.jointstates.position[4], self.jointstates.position[5] )
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


        calculated_pose = self.make_basic_gripper_pose( new_pos_finger_1_joint_1, new_pos_finger_1_joint_2, new_pos_finger_1_joint_3,
                                                       new_pos_finger_2_joint_1, new_pos_finger_2_joint_2, new_pos_finger_2_joint_3, 
                                                       new_pos_finger_middle_joint_1, new_pos_finger_middle_joint_2, new_pos_finger_middle_joint_3 )
        self.gripper_move_group.go(calculated_pose, wait=True)


        
    # takes the finger joint angles (proximal, medial, distal) for the Robotiq 3-finger hand and replicates those angles on all three fingers.
    # Finger abduction (palm) joints are hardcoded to "basic" mode (near zero).
    def make_basic_gripper_pose(self, f1_1, f1_2, f1_3, f2_1, f2_2, f2_3, mf1, mf2, mf3 ):
        js = JointState()
        js.name = [ "s_model_finger_1_joint_1", "s_model_finger_1_joint_2", "s_model_finger_1_joint_3", \
                "s_model_finger_2_joint_1", "s_model_finger_2_joint_2", "s_model_finger_2_joint_3", \
                "s_model_finger_middle_joint_1", "s_model_finger_middle_joint_2", "s_model_finger_middle_joint_3", \
                "s_model_palm_finger_1_joint", "s_model_palm_finger_2_joint" ]
        #js.position = [ mf1, mf2, mf3, mf1, mf2, mf3, mf1, mf2, mf3, -0.016, 0.016 ]
        #js.position = [ f1_1, f1_2, f1_3, f2_1, f2_2, f2_3, mf1, mf2, mf3, -0.016, 0.016 ] # for basic mode
        js.position = [ f1_1, f1_2, f1_3, f2_1, f2_2, f2_3, mf1, mf2, mf3, -0.15, 0.15 ] # for pinch mode

        return js 
    
    
    def make_arm_pose(self, joint_angles ):
        # note: rostopic pub /joint_states has elbow lift pan wrist 1 2 3 :-(
        js = JointState()
        js.name = [ "ur5_elbow_joint", "ur5_shoulder_lift_joint", "ur5_shoulder_pan_joint", \
                "ur5_wrist_1_joint", "ur5_wrist_2_joint", "ur5_wrist_3_joint" ]
        js.position = joint_angles
        return js
        
    

if __name__ == "__main__":
    rospy.init_node('gazebo_robotiq_controller', anonymous=True)
    rospy.sleep(1)

    newMovement = MoveFingers() 
    
    rospy.spin()

    