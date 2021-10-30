#!/usr/bin/env python

# =========================================================== #
# [Author] Moses C. Nah
# [Email]  mosesnah@mit.edu
# [Date]   2021.10.23
# [Description]
#   - The most basic code for running Baxter
#   - This code is specifically for reading the joint datas from the Baxter Robot.
# =========================================================== #


import rospy
import baxter_interface
import baxter_external_devices          # For keyboard interrupt
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

        # Controller Rates
        self.joint_publish_rate = 1000.0 # Hz
        self.controller_rate    = 520.0 # Hz

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




    def print_joints( self ):

        DONE = False
        while not DONE :
            c = baxter_external_devices.getch()     # Get the char input from the Keyboard

            if c:

                #catch Esc or ctrl-c
                if c in ['\x1b', '\x03']:
                    DONE = True
                    rospy.signal_shutdown("Reading Joint Data finished.")

                if c == "p":
                    print(" Printing Joint Data")
                    print( self.arm.joint_angles() )


    def clean_shutdown(self):
        rospy.sleep( 1 )
        self.arm.exit_control_mode()

        if not self.init_state and self.rs.state().enabled:
            # print( "Disabling robot..." )
            self.rs.disable()

def main():
    
    print("Initializing Controller Node... ")
    rospy.init_node("MY_BAXTER_CONTROL")
    
    ctrl = BaxterControl( arm_type = RIGHT )
    rospy.on_shutdown( ctrl.clean_shutdown )

    ctrl.print_joints( )

if __name__ == '__main__':
    main()
