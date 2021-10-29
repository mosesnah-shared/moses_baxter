#!/usr/bin/env python

# Source code from following:
# [REF] https://github.com/tony1994513/Impedance-control/blob/master/impedcontol.py

import baxter_interface
import argparse
import rospy
import geometry_msgs
import numpy as np


from dynamic_reconfigure.server import Server
from std_msgs.msg               import Empty 
from baxter_interface           import CHECK_VERSION
from baxter_pykdl               import baxter_kinematics


# Local Library, under moses/scripts
from my_constants import Constants as C

class JointImpedanceControl( object ):
    """
    Virtual Joint Springs class for torque example.
    @param limb: limb on which to run joint springs example
    @param reconfig_server: dynamic reconfigure server
    JointSprings class contains methods for the joint torque example allowing
    moving the limb to a neutral location, entering torque mode, and attaching
    virtual springs.
    """
    def __init__( self, limb = 'right' ):#, reconfig_server):
        # self._dyn = reconfig_server
        self._rKin = baxter_kinematics('right')

        # control parameters
        self._rate = 1000.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout

        # create our limb instance
        self._limb = baxter_interface.Limb(limb)

        # initialize parameters
        self.Kq = dict()
        self.Bq = dict()
        self._start_angles = dict()

        self._output = np.array([0,0,0,0,0,0])

        # self._update_parameters()    
        self.Kq = { 'right_s0': 10.0, 
                    'right_s1': 15.0, 
                    'right_w0': 3.0, 
                    'right_w1': 2.0, 
                    'right_w2': 1.5, 
                    'right_e0': 5.0,  
                    'right_e1': 5.0 }   
        self.Bq = { 'right_s0': 0.1, 
                    'right_s1': 0.1, 
                    'right_w0': 0.1, 
                    'right_w1': 0.1,
                    'right_w2': 0.1, 
                    'right_e0': 0.1, 
                    'right_e1': 0.1 } 

        self.robot_init( )


    def robot_init( self ):
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

    def _update_torques(self):
        """
            Calculates the current angular difference between the start position
            and the current joint positions applying the joint torque spring forces
            as defined on the dynamic reconfigure server.
        """
        cmd = dict()
        
        cur_pos = self._limb.joint_angles()
        cur_vel = self._limb.joint_velocities()

        for joint in self._start_angles.keys():
           
            cmd[ joint ]  =   self.Kq[ joint ] * ( self._start_angles[ joint ] - cur_pos[ joint ] )   # spring portion
            cmd[ joint ] += - self.Bq[ joint ] * cur_vel[ joint ]                                     # damping portion
        
        self._output = np.array( list( cmd.values( ) ) ) 

        Cmd = { 'right_s0':self._output[ 0 ], 
                'right_s1':self._output[ 1 ], 
                'right_w0':self._output[ 2 ], 
                'right_w1':self._output[ 3 ],
                'right_w2':self._output[ 4 ], 
                'right_e0':self._output[ 5 ], 
                'right_e1':self._output[ 6 ] }

        self._limb.set_joint_torques( Cmd )

    def start_impedance(self):
        """
            Switches to joint torque mode and attached joint springs to current joint positions.
        """

        self._start_angles = self._limb.joint_angles()

        # set control rate
        control_rate = rospy.Rate(self._rate)

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and disable
        self._limb.set_command_timeout( ( 1.0 / self._rate) * self._missed_cmds)

        # loop at specified rate commanding new joint torques
        while not rospy.is_shutdown():
            if not self._rs.state().enabled:
                rospy.logerr("impedance example failed to meet, specified control rate timeout.")
                break

            self._update_torques()
            control_rate.sleep()

    def move2pose( self, joint_angles ):        
        self._limb.move_to_joint_positions( joint_angles )
        rospy.sleep( 1.0 )

    def clean_shutdown(self):
        """
            Switches out of joint torque mode to exit cleanly
        """

        print( "\nExiting example..." )

        self._limb.exit_control_mode()
        
        if not self._init_state and self._rs.state().enabled:
            print( "Disabling robot..." )
            self._rs.disable()


def main():

    print("Initializing node... ")
    rospy.init_node("impedance_control_right" )
    imp_ctrl = JointImpedanceControl( limb = 'right' )

    rospy.on_shutdown( imp_ctrl.clean_shutdown )

    starting_joint_angles = {'right_w0': -0.3305728597893067, 
                             'right_w1': -1.0783884938834458, 
                             'right_w2': -1.8177672336442152, 
                             'right_e0':  0.3079466431679968,
                             'right_e1':  1.5964905049917444,
                             'right_s0':  0.6089903727905093, 
                             'right_s1': -0.3451456772742181}

    imp_ctrl.move2pose( starting_joint_angles )
    imp_ctrl.start_impedance()

if __name__ == "__main__":
    main()