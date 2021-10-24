#!/usr/bin/env python

# =========================================================== #
# [Author] Moses C. Nah
# [Email]  mosesnah@mit.edu
# [Date]   2021.10.23
# [Description]
#   - The most basic code for running Baxter
# =========================================================== #


import rospy
import time
import baxter_interface
import numpy        as np
import std_msgs.msg as msg 
import threading

from   std_msgs.msg       import UInt16, Empty
from   baxter_interface   import CHECK_VERSION
from   numpy.linalg       import inv, pinv
from   baxter_pykdl       import baxter_kinematics
from   tf.transformations import ( quaternion_from_euler , quaternion_matrix   , 
                                   quaternion_inverse    , quaternion_multiply )


# [CONSTANTS] 
RIGHT = 0
LEFT  = 1
BOTH  = 2


MIN_POSE_RIGHT = { 
   'right_s0':  0.291072854501257, 
   'right_s1': -0.639669988548217,
   'right_e0':  0.656160282017985, 
   'right_e1':  1.958509970932702, 
   'right_w0': -0.042951462060791,
   'right_w1': -1.183849673050568,
   'right_w2': -0.575626290654001
}

MIN_POSE_LEFT = {  
   'left_s0': -0.291072854501257, 
   'left_s1': -0.639669988548217,
   'left_e0': -0.656160282017985, 
   'left_e1':  1.958509970932702, 
   'left_w0': -0.042951462060791,
   'left_w1': -1.183849673050568,
   'left_w2':  0.575626290654001
}


GRASP_POSE_RIGHT = {
    'right_s0':  0.63621853177547540, 
    'right_s1': -0.30219421521342650, 
    'right_e0':  0.45099035163831164, 
    'right_e1':  1.78095169473496530,
    'right_w0': -0.03604854851530722, 
    'right_w1': -1.44922834935474460, 
    'right_w2': -2.00567988016017830 
}

GRASP_POSE_LEFT = {
    'left_s0': -0.63621853177547540, 
    'left_s1': -0.30219421521342650, 
    'left_e0': -0.45099035163831164, 
    'left_e1':  1.78095169473496530,
    'left_w0': -0.03604854851530722, 
    'left_w1': -1.44922834935474460, 
    'left_w2':  2.00567988016017830, 
}

STRAIGHTEN_UP_POSE_RIGHT = {
    'right_s0':  0.8893253617765686, 
    'right_s1': -0.8755195346855998,
    'right_e0':  0.3834951969713534, 
    'right_e1': -0.0494708804093045,    
    'right_w0': -0.2132233295160725, 
    'right_w1':  0.1399757468945440, 
    'right_w2': -1.5876701154614032
}

STRAIGHTEN_UP_POSE_LEFT = {
    'left_s0': -0.8893253617765686, 
    'left_s1': -0.8755195346855998,
    'left_e0': -0.3834951969713534, 
    'left_e1': -0.0494708804093045,   
    'left_w0': -0.2132233295160725, 
    'left_w1':  0.1399757468945440, 
    'left_w2':  1.5876701154614032
}

POSE_2_RIGHT = {    
    'right_s0':  0.6691991187150117, 
    'right_s1': -0.6189612479117644, 
    'right_e0':  0.4594272459716814, 
    'right_e1':  0.5464806556841787,
    'right_w0': -3.0491703111192310, 
    'right_w1': -1.5700293364007210, 
    'right_w2': -1.6352235198858511
}


POSE_2_LEFT = {    
    'left_s0': -0.6691991187150117, 
    'left_s1': -0.6189612479117644, 
    'left_e0': -0.4594272459716814, 
    'left_e1':  0.5464806556841787,
    'left_w0': -3.0491703111192310, 
    'left_w1': -1.5700293364007210, 
    'left_w2':  1.6352235198858511
}

REST_POSE_RIGHT = {
    'right_s0':  0.6711165946998685, 
    'right_s1': -0.2834029505618302, 
    'right_e0':  0.1441941940612289, 
    'right_e1':  1.3997574689454400,  
    'right_w0': -3.0480198255283173, 
    'right_w1': -0.4329660773806580, 
    'right_w2': -1.8047283969471892, 
}

REST_POSE_LEFT = {
    'left_s0': -0.6711165946998685, 
    'left_s1': -0.2834029505618302, 
    'left_e0': -0.1441941940612289, 
    'left_e1':  1.3997574689454400,  
    'left_w0': -3.0480198255283173, 
    'left_w1': -0.4329660773806580, 
    'left_w2':  1.8047283969471892, 
}

# STRAIGHTEN_WIDE_POSE_RIGHT = {
    

# }

# STRAIGHTEN_WIDE_POSE_LEFT  = {
    

# }

# Flip the sign for s0, e0, and w2 for the joint posture
# However, this is not always the case
# GRASP_POSE_LEFT = {
#     'left_s0': -0.45099035163831164, 
#     'left_s1': -0.16375244910676792
#     'left_e0': -0.6768690226544388, 
#     'left_e1':  1.7226604247953197, 
#     'left_w0':  0.11466506389443468, 
#     'left_w1': -1.26438366441455230, 
#     'left_w2': -0.8091748656095558, 
# }





# [BACKUP]
# import math
# from tf.transformations import (euler_from_quaternion, quaternion_from_euler)
 
class BaxterControl( object ):

    def __init__( self, arm_type = LEFT ):
        
        self.arm_type = arm_type

        self.record_data = True # True or False

        # Robot Control Objects
        if   self.arm_type == RIGHT:
            self.arm  = baxter_interface.limb.Limb( "right" )
            self.kin  = baxter_kinematics(          "right" )
            self.grip = baxter_interface.Gripper(   "right" )

        elif self.arm_type == LEFT:
            self.arm  = baxter_interface.limb.Limb( "left" )
            self.kin  = baxter_kinematics(          "left" )
            self.grip = baxter_interface.Gripper(   "left" )

        self.arm_names = self.arm.joint_names( )
        self.grip.set_holding_force( 0 )


        # Enable Robot
        rospy.loginfo( "Getting robot state... " )
        self.rs = baxter_interface.RobotEnable( CHECK_VERSION )
        self.init_state = self.rs.state().enabled
        rospy.loginfo("Enabling robot... ")
        self.rs.enable()


        if self.record_data:
            time_now = time.strftime('%Y_%m_%d-%H_%M')
            self.file_Q  = open('Q_MotionCon_'  + time_now + '.txt', 'w')             
              
    def move2grasp_pose( self ):
        print( 'moving to pre-position..' )

        self.arm.set_joint_position_speed( 0.5 )    # Value range [0.0, 1.0], default 0.3


        start_time = rospy.Time.now()
        time = 0.0
        end_time = 20
        update_rate = 1.0/60
        update_time = 0

        while time < end_time :
            time = (rospy.Time.now() - start_time).to_sec()


            if self.arm_type == RIGHT:

                if   time < 5:
                    posture = REST_POSE_RIGHT
                    self.arm.move_to_joint_positions( posture )
                
                elif time >10:

                    posture = GRASP_POSE_RIGHT
                    self.arm.move_to_joint_positions( posture )

 
            if self.record_data and time - update_time >= update_rate:
                self.print_q( time )
                update_time = time

            elif self.arm_type == LEFT:
                posture = REST_POSE_LEFT
                self.arm.move_to_joint_positions( posture )


    def print_q( self, t ):
        q_current_dict = self.arm.joint_angles()
        
        if self.arm_type == RIGHT :
            self.file_Q.write( str( t ) + " right " + ' ' + str(q_current_dict['right_s0']) + ' ' + str(q_current_dict['right_s1']) + ' ' + str(q_current_dict['right_e0']) + ' ' + str(q_current_dict['right_e1']) + ' '+ str(q_current_dict['right_w0']) + ' ' + str(q_current_dict['right_w1']) + ' ' + str(q_current_dict['right_w2']) + '\n')
        elif self.arm_type == LEFT :
            self.file_Q.write( str( t ) + " left " ' ' + str(q_current_dict['left_s0']) + ' ' + str(q_current_dict['left_s1']) + ' ' + str(q_current_dict['left_e0']) + ' ' + str(q_current_dict['left_e1']) + ' '+ str(q_current_dict['left_w0']) + ' ' + str(q_current_dict['left_w1']) + ' ' + str(q_current_dict['left_w2']) + '\n' )


    def grip_command( self ):
        self.grip.close()
   
        rospy.sleep(2)
        self.grip.open()


    def clean_shutdown(self):

        rospy.sleep(2)
        self.arm.exit_control_mode()


def main():
    
    print("Initializing Controller Node... ")
    
    rospy.init_node("MY_BAXTER_CONTROL")
    
    ctrl = BaxterControl( arm_type = RIGHT )
    rospy.on_shutdown( ctrl.clean_shutdown )
    ctrl.move2grasp_pose( )
    # ctrl.print_joints( )
    # ctrl.grip_command( )
    print("Done")


if __name__ == '__main__':
    main()
