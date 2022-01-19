#!/usr/bin/python3

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
from   baxter_pykdl       import baxter_kinematics


# Local Library, under moses/scripts
from my_constants import Constants as C


# [BACKUP]
# import math
# from tf.transformations import (euler_from_quaternion, quaternion_from_euler)

class BaxterControl( object ):

    def __init__( self, arm_type = C.RIGHT ):

        self.arm_type = arm_type

        # Controller Rates
        self.joint_publish_rate = 1000.0 # Hz
        self.controller_rate    = 520.0 # Hz

        # Robot Control Objects
        if   self.arm_type == C.RIGHT:
            self.arm  = baxter_interface.limb.Limb( "right" )
            self.kin  = baxter_kinematics(          "right" )
            self.grip = baxter_interface.Gripper(   "right" )

        elif self.arm_type == C.LEFT:
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
                    # The most convenient form for the print is simply printing out the right angle joints without "right" prefix,
                    # in the following order:
                    # Joint angle
                    print( "=" * 100 )
                    for name in C.JOINT_NAMES:
                        val = self.arm.joint_angles( ).get( "right_" + name )
                        print( "'{0}' : {1:.10f},".format( name, val ) )

                    print( "=" * 100 )



    def clean_shutdown(self):
        rospy.sleep( 1 )
        self.arm.exit_control_mode()

        if not self.init_state and self.rs.state().enabled:
            # print( "Disabling robot..." )
            self.rs.disable()

def main():

    print("Initializing Controller Node... ")
    rospy.init_node("MY_BAXTER_CONTROL")

    ctrl = BaxterControl( arm_type = C.RIGHT )
    rospy.on_shutdown( ctrl.clean_shutdown )

    ctrl.print_joints( )

if __name__ == '__main__':
    main()
