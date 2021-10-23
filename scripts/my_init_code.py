#!/usr/bin/env python

# =========================================================== #
# [Author] Moses C. Nah
# [Email]  mosesnah@mit.edu
# [Date]   2021.10.23
# [Description]
#   - The most basic code for running Baxter
# =========================================================== #


import rospy
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


# [BACKUP]
# import math
# from tf.transformations import (euler_from_quaternion, quaternion_from_euler)
 
class BaxterControl( object ):

    def __init__( self, arm_type = LEFT ):
        
        self.arm_type = arm_type

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


    def move2prepose( self ):
        print( 'moving to pre-position..' )

        if self.arm_type == RIGHT:
            prepose = {'right_s0':  0.291072854501257, 
                       'right_s1': -0.639669988548217,
                       'right_e0':  0.656160282017985, 
                       'right_e1':  1.958509970932702, 
                       'right_w0': -0.042951462060791,
                       'right_w1': -1.183849673050568,
                       'right_w2': -0.575626290654001}

        elif self.arm_type == LEFT:
            prepose = { 'left_s0': -0.291072854501257, 
                        'left_s1': -0.639669988548217,
                        'left_e0': -0.656160282017985, 
                        'left_e1':  1.958509970932702, 
                        'left_w0': -0.042951462060791,
                        'left_w1': -1.183849673050568,
                        'left_w2':  0.575626290654001}

    
        self.arm.set_joint_position_speed( 0.8 )    # Value range [0.0, 1.0], default 0.3
        self.arm.move_to_joint_positions( prepose )


    def print_joints( self ):

        start_time = rospy.Time.now()   
        time = 0.0
        T    = 3.0
        while time < T :
            time = ( rospy.Time.now( ) - start_time ).to_sec()
            print( self.arm.joint_angles() )


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
    ctrl.move2prepose( )
    # ctrl.print_joints( )
    ctrl.grip_command( )
    print("Done")


if __name__ == '__main__':
    main()
