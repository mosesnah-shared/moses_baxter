#!/usr/bin/env python

# Source code from following:
# [REF] https://github.com/tony1994513/Impedance-control/blob/master/impedcontol.py

import baxter_interface
import argparse
import time
import rospy
import numpy as np
import sys

from std_msgs.msg               import Empty 
from baxter_interface           import CHECK_VERSION
from baxter_pykdl               import baxter_kinematics
from sensor_msgs.msg            import JointState


# Local Library, under moses/scripts
from my_constants import Constants as C
from my_utils     import Logger

class Logger(object):
    def __init__(self, record_data = False):
        self.record_data = record_data
        self.terminal    = sys.stdout

        if record_data == True:  
            time_now         = time.strftime( '%Y_%m_%d-%H_%M' )                 # Redirecting the stdout to file
            self.log = open( C.SAVE_DIR + 'baxter_'  + time_now + '.txt' , 'w')
        else:
            self.log = None
   
    def write(self, message ):
        self.terminal.write( message )

        if self.log is not None:
            self.log.write( message )  

    def flush(self):
        # this flush method is needed for python 3 compatibility.
        # this handles the flush command by doing nothing.
        # you might want to specify some extra behavior here.
        pass    

def callback( data, start_time ):
    # For the JointState data, we have the following data
    # [1] position
    # [2] velocity
    # [3] effort
    # [4] name
    # [*] etc.
    # [REF] https://stackoverflow.com/questions/57271100/how-to-feed-the-data-obtained-from-rospy-subscriber-data-into-a-variable
    # We are interested at the joint position data, hence check whether the given data are joint data
    # Can add more inputs if you want, by simply concatenating them
    # [REF] https://answers.ros.org/question/332126/passing-multiple-arguments-to-subscriber-callback-function-in-python/
    # We will check this via finding a specific string
    if 'torso_t0' in data.name:


        time = ( rospy.Time.now() - start_time).to_sec()    
        for name in C.LEFT_JOINT_NAMES:
            # Find index of the name
            try:
                idx = data.name.index( name )
                rospy.loginfo( "[time] [%.4f] [name] [%s] [value] [%.5f]", time, name, data.position[ idx ]  ) 
                # if file is not None:
                    # file.write( "time = {}, name = {},  value = {}".format( time, name, data.position[ idx ] ) )
            except:
                NotImplementedError( )

        time = ( rospy.Time.now() - start_time).to_sec()    
        for name in C.RIGHT_JOINT_NAMES:
            # Find index of the name
            try:
                idx = data.name.index( name )
                rospy.loginfo( "[time] [%.4f] [name] [%s] [value] [%.5f]", time, name, data.position[ idx ]  ) 
                # if file is not None:
                #     file.write( "time = {%.4f}, name = {%s},  value = {%.5f}".format( time, name, data.position[ idx ] ) )
            except:
                NotImplementedError( )


class JointImpedanceControl( object ):
    def __init__( self, record_data = False ):#, reconfig_server):
        

        self.record_data = record_data           # Boolean, data saved 
                                                 # Just define the whole arms in the first place to simplify the code
        self.arms        = range( 2 )            # Since we have both the left/right arms
        self.kins        = range( 2 )            # Since we have both the left/right arms
        self.grips       = range( 2 )            # Since we have both the left/right arms

        self.start_time  = rospy.Time.now()
        self.T_max       = 30                   # The maximum run time for the robot. 

        # Saving the limb objects
        for idx, name in enumerate( [ "right", "left" ] ):
            self.arms[  idx ] = baxter_interface.limb.Limb( name )
            self.kins[  idx ] = baxter_kinematics(          name )
            self.grips[ idx ] = baxter_interface.Gripper(   name )

            # Initialize the grippers holding force
            self.grips[ idx ].set_holding_force( 0 )



        # self._rKin = baxter_kinematics('right')

                                    # control parameters
        self.rate        = 1000.0  # Hz
        self.missed_cmds = 20.0    # Missed cycles before triggering timeout


        # self._update_parameters()    
        self.Kq = { 'right_s0': 10.0, 
                    'right_s1': 15.0, 
                    'right_w0': 3.0, 
                    'right_w1': 2.0, 
                    'right_w2': 1.5, 
                    'right_e0': 5.0,  
                    'right_e1': 5.0 } 


        self.Bq = { 'right_s0': 2, 
                    'right_s1': 3.0, 
                    'right_w0': 0.6, 
                    'right_w1': 0.4,
                    'right_w2': 0.3, 
                    'right_e0': 1.0, 
                    'right_e1': 1.0 } 

        self.robot_init( )
        sys.stdout = Logger( record_data = record_data ) 


    def robot_init( self ):
        print("Getting robot state... ")
        
        self.rs         = baxter_interface.RobotEnable( CHECK_VERSION )
        self.init_state = self.rs.state().enabled
        
        print("Enabling robot... ")
        self.rs.enable()
        
        print("Running. Ctrl-c to quit")

    def update_torques(self):
        """
            Calculates the current angular difference between the start position
            and the current joint positions applying the joint torque spring forces
            as defined on the dynamic reconfigure server.
        """
        tau = dict()
        
        q  = self.arms[ C.RIGHT ].joint_angles()
        dq = self.arms[ C.RIGHT ].joint_velocities()

        for joint in C.RIGHT_JOINT_NAMES:
           
            tau[ joint ]  =   self.Kq[ joint ] * ( self.q0[ joint ] - q[ joint ] )   # spring portion
            tau[ joint ] += - self.Bq[ joint ] * dq[ joint ]                                     # damping portion
        
        self.arms[ C.RIGHT ].set_joint_torques( tau )

    def start_impedance(self):
        """
            Switches to joint torque mode and attached joint springs to current joint positions.
        """

        # Maintaining the given posture
        self.q0 = self.arms[ C.RIGHT ].joint_angles()

        # set control rate
        control_rate = rospy.Rate( self.rate )

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and disable
        self.arms[ C.RIGHT ].set_command_timeout( ( 1.0 / self.rate) * self.missed_cmds)

        # loop at specified rate commanding new joint torques
        while not rospy.is_shutdown() and ( rospy.Time.now() - self.start_time ).to_sec( ) <= self.T_max: 
            if not self.rs.state().enabled:
                rospy.logerr("impedance example failed to meet, specified control rate timeout.")
                break

            self.update_torques()
            control_rate.sleep()

    def move2pose( self, which_arm, pose, joint_speed = 0.1 ):

        # If which_arm is neither 0 nor 1, assert
        assert which_arm in [ C.RIGHT, C.LEFT] 

        if   which_arm == C.RIGHT:
            # Need to check whether the names contain right on the pose
            assert all( "right" in s for s in pose.keys( ) )

        elif which_arm == C.LEFT:

            # Need to check whether the names contain left on the pose
            assert all( "left" in s for s in pose.keys( ) )

        self.arms[ which_arm ].set_joint_position_speed( joint_speed )
        self.arms[ which_arm ].move_to_joint_positions( pose )     

    def joint_state_listener( self ):
        
        # rospy.init_node( 'my_listener', anonymous = False )
        rospy.Subscriber( "robot/joint_states", JointState, callback, self.start_time )

    def clean_shutdown(self):
        """
            Switches out of joint torque mode to exit cleanly
        """

        # print( "\nExiting example..." )

        for i in range( 2 ):
            self.arms[ i ].exit_control_mode()
        
        if not self.init_state and self.rs.state().enabled:
            # print( "Disabling robot..." )
            self.rs.disable()


def main():

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser  = argparse.ArgumentParser( formatter_class = arg_fmt )
    parser.add_argument('-s', '--save_data',    
                        dest = 'record_data',   action = 'store_true',
                        help = 'Save the Data')

    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("impedance_control_right" )
    my_baxter = JointImpedanceControl( args.record_data )

    rospy.on_shutdown( my_baxter.clean_shutdown )

    my_baxter.joint_state_listener( )

    my_baxter.move2pose( C.RIGHT, C.GRASP_POSE_RIGHT_WIDER, 0.3 )
    my_baxter.start_impedance()

if __name__ == "__main__":
    main()