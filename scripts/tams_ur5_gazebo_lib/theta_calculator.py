#!/usr/bin/env python
#

class ThetaCalculator:
    def __init__(self):
                print("Create Theta Class")

    #hier anfagen delta thetas anfangen auszurechnen
    # x = (theta_1, theta_2, theta_3, g),  u element of [-1,1] describing change in g from one time step to another
    # funktion f_1(x,u)
    def f_1(self, delta_g):
        theta_1_max =  1.2 #1.2218
        m_1 = theta_1_max / 140
        
        f_1_xu = m_1 * delta_g
        return f_1_xu
    
    # funktion f_2(x,u)
    def f_2(self, delta_g):
        theta_2_max = 1.526 #1.5708
        m_2 = theta_2_max / 100
        
        f_2_xu = m_2 * delta_g
        return f_2_xu
    
    # funktion f_3(x,u):  
    def f_3(self, g, delta_g):
        theta_3_min =  -1.2 #-1.2217
        theta_3_max = -0.06 #-0.0523 #-0.068:
        
        m_3 = theta_3_min + ((theta_3_max - theta_3_min)/(255 - g))  
        f_3_xu = m_3*delta_g
        return f_3_xu
    
    def f_3_g1(self, g1, delta_g):
        theta_3_min =  -1.2 #-1.2217
        theta_3_max = -0.06 #-0.0523 #-0.068:
        
        m_3 = theta_3_min + ((theta_3_max - theta_3_min)/(255 - g1))  
        f_3_xu = m_3*delta_g
        return f_3_xu

    def f_3_g2(self, g2, delta_g):
        theta_3_min =  -1.2 #-1.2217
        theta_3_max = -0.06 #-0.0523 #-0.068:
        
        m_3 = theta_3_min + ((theta_3_max - theta_3_min)/(255 - g2))  
        f_3_xu = m_3*delta_g
        return f_3_xu            


    def calc_delta(self, g, tuple, endstate):
            
        print("tuple:", tuple)
        if g == 0:
            u = -1
        else:
            u = 1
 
        # 1     ###########################################################
        if tuple == [0, 0, 0, 0, 0, 0]:
            print("phase 1")
            delta_g = u
            delta_theta_1 = self.f_1(delta_g)
            delta_theta_2 = 0.00
            delta_theta_3 = - self.f_1(delta_g)
            #endstate[0] = 0

        # 1'     ###########################################################
        elif tuple == [0, 0, 0, 0, 0, -1]: 
            print("phase 1'")
            delta_g = u
            delta_theta_1 = self.f_1(delta_g)
            delta_theta_2 = 0.00
            delta_theta_3 = 0.00


        # 2     ###########################################################
        elif tuple == [1, 0, 0, 0, 0, 0] or tuple == [1, 0, 0, 1, 0, 0] or tuple == [0, 0, 0, 1, 0, 0]:
            print("phase 2")
            delta_g = u
            delta_theta_1 = 0.00
            delta_theta_2 = self.f_2(delta_g)
            delta_theta_3 = - self.f_2(delta_g)

        # 2'     ###########################################################
        elif tuple == [1, 0, 0, 0, 0, -1] or tuple == [0, 0, 0, 1, 0, -1] or tuple == [1,0,0,1,0,-1]:
            print("phase 2'")
            delta_g = u
            delta_theta_1 = 0.00
            delta_theta_2 = self.f_2(delta_g)
            delta_theta_3 = 0.00

            
        # 3     ###########################################################
        elif ((tuple[1] == 1 and tuple[2] == 0 and tuple[4] == 0 and tuple[5] == 0) 
            or (tuple[1] == 0 and tuple[2] == 0 and tuple[3] == 1 and tuple[4] == 1 and tuple[5] == 0 )): 
            #[., 1, 0, ., 0, 0]:
            print("phase 3")
            delta_g = u
            delta_theta_1 = 0.00
            delta_theta_2 = 0.00
            delta_theta_3 = -self.f_3(g, delta_g)

        
        # 3'     ###########################################################
        elif ((tuple[1] == 1 and tuple[2] == 0 and tuple[4] == 0 and tuple[5] == -1) 
              or (tuple[1] == 0 and tuple[2] == 0 and tuple[3] == 0 and tuple[4] == 1 and tuple[5] == -1)
              or (tuple[1] == 1 and tuple[2] == 0 and tuple[4] == 0 and tuple[5] == 0)) :
            #[., 1, 0, ., 0, -1] or [.,0,0,0,1,-1] or [.,1,0,.,0,0]
            print("phase 3'")
            delta_g = u
            delta_theta_1 = 0.00
            delta_theta_2 = 0.00
            delta_theta_3 = self.f_3(g, delta_g)
            

        # 4     ###########################################################
        elif ((tuple[2] == 1 and tuple[5] == 0) 
              or (tuple[2] == 0 and tuple[5] == 1) 
              or tuple == [0, 0, 0, 1, 1, -1] ):
            #tuple == [0, 0, 1, 0, 0, 0]:
            print("phase 4")
            delta_theta_1 = 0.00
            delta_theta_2 = 0.00
            delta_theta_3 = 0.00
            delta_g = 0.00
            print( "... Middle Finger End State." )
            endstate = 1

        return delta_theta_1, delta_theta_2, delta_theta_3, delta_g, endstate
    
    