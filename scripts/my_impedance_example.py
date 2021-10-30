#!/usr/bin/env python

# Source code from following:
# [REF] https://github.com/tony1994513/Impedance-control/blob/master/impedcontol.py


import argparse
import time
import rospy
import numpy as np
import sys

# BAXTER Libraries
import baxter_interface
import baxter_external_devices          # For keyboard interrupt
from   baxter_interface           import CHECK_VERSION
from   baxter_pykdl               import baxter_kinematics

# Ros related Libraries
from std_msgs.msg                 import Empty 
from sensor_msgs.msg              import JointState

# Local Library, under moses/scripts
from my_constants import Constants as C
from my_utils     import Logger


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
        self.T_MAX       = 30                    # The maximum run time for the robot, current

        # Saving the limb objects
        for idx, name in enumerate( [ "right", "left" ] ):
            self.arms[  idx ] = baxter_interface.limb.Limb( name )
            self.kins[  idx ] = baxter_kinematics(          name )
            self.grips[ idx ] = baxter_interface.Gripper(   name )

            self.grips[ idx ].set_holding_force( 100 )  # Initialize the grippers holding force
            self.grips[ idx ].open(  block = False )    # Open the gripper too


                                   # control parameters
        self.rate        = 1000.0  # Hz
        self.missed_cmds = 20.0    # Missed cycles before triggering timeout

        # Impedance Parameters for Case 1 
        self.Kq = C.JOINT_IMP1_Kq
        self.Bq = C.JOINT_IMP1_Bq

        self.robot_init( )
        sys.stdout = Logger( record_data = record_data ) 


    def robot_init( self ):
        print("Getting robot state... ")
        
        self.rs         = baxter_interface.RobotEnable( CHECK_VERSION )
        self.init_state = self.rs.state().enabled
        
        print("Enabling robot... ")
        self.rs.enable()
        
        print("Running. Ctrl-c to quit")


    def get_reference_traj( self, pose1, pose2, D, t  ):
        """ 
            Get the reference trajectory, the basis function is the minimum jerk trajectory
        """

        q0  = dict( )   # Making  q0 as a dictionary
        dq0 = dict( )   # Making dq0 as a dictionary 


        # Iterate through the joints
        for joint in C.RIGHT_JOINT_NAMES:


            tt = t/D if t<=D else 1     # Normalized time as tt, tau is actually the notation but tau is reserved for torque
                                        # If duration bigger than time t than set tt as 1

            q0[  joint ] = pose1[ joint ] + ( pose2[ joint ] - pose1[ joint ] ) * (  10 * tt ** 3 - 15 * tt ** 4 +  6 * tt ** 5 )
            dq0[ joint ] =          1.0/D * ( pose2[ joint ] - pose1[ joint ] ) * (  30 * tt ** 2 - 60 * tt ** 3 + 30 * tt ** 4 )
     
        return q0, dq0 

    def joint_impedance( self, pose1, pose2, D, t_total ):
        """
            start_impedance joint controller, from pose1 to pose2 reference trajectory
            t_total is for the time the controller runs
        """

        # set control rate
        control_rate = rospy.Rate( self.rate )

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and disable
        self.arms[ C.RIGHT ].set_command_timeout( ( 1.0 / self.rate) * self.missed_cmds)

        ts = rospy.Time.now()
        t  = 0                  # Elapsed time

        # loop at specified rate commanding new joint torques
        while not rospy.is_shutdown() and t <= t_total: 
            if not self.rs.state().enabled:
                rospy.logerr("impedance example failed to meet, specified control rate timeout.")
                break

            t = ( rospy.Time.now( ) - ts ).to_sec( )        # The elapsed time of the simulatoin

            q0, dq0 = self.get_reference_traj( pose1, pose2, D, t )

            # self.update_torques()
            tau = dict()
            
            q   = self.arms[ C.RIGHT ].joint_angles()
            dq  = self.arms[ C.RIGHT ].joint_velocities()

            for joint in C.RIGHT_JOINT_NAMES:
               
                tau[ joint ]  =   self.Kq[ joint ] * (  q0[ joint ] -  q[ joint ] )   # The Stiffness portion
                tau[ joint ] +=   self.Bq[ joint ] * ( dq0[ joint ] - dq[ joint ] )   # The Damping   portion
                
                if self.record_data:
                    rospy.loginfo( "[time] [%.4f] [name] [%s] [value] [%.5f]", t, joint + "_q0"  ,  q0[ joint ]  ) 
                    rospy.loginfo( "[time] [%.4f] [name] [%s] [value] [%.5f]", t, joint + "_q"   ,   q[ joint ]  ) 
                    rospy.loginfo( "[time] [%.4f] [name] [%s] [value] [%.5f]", t, joint + "_tau" , tau[ joint ]  ) 


            self.arms[ C.RIGHT ].set_joint_torques( tau )

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


    def control_gripper( self , mode = "timer"):
        """
            There are two modes to do the controller of the gripper, simple time wait or keyboard
        """
        if   mode == "timer":

            rospy.sleep( 5 )

            self.grips[ C.RIGHT ].close(  block = False )   
            self.grips[ C.LEFT  ].close(  block = False )

            rospy.sleep( 5 )

        elif mode == "keyboard":

            # Control the gripper via keyboard command
            DONE = False

            left_open  = True
            right_open = True

            while not DONE :
                c = baxter_external_devices.getch()     # Get the char input from the Keyboard
                if c:

                    #catch Esc or ctrl-c
                    if c in ['\x1b', '\x03']:
                        DONE = True
                        rospy.signal_shutdown("Reading Joint Data finished.")

                    if c == "l":
                        if left_open == True: 
                            self.grips[ C.LEFT  ].open(   block = False )

                        else:
                            self.grips[ C.LEFT  ].close(  block = False )

                        # Flip the boolean value
                        left_open = not left_open

                    if c == "r":
                        if right_open == True:
                            self.grips[ C.RIGHT ].open(   block = False )

                        else:
                            self.grips[ C.RIGHT ].close(  block = False )

                        # Flip the boolean value
                        right_open = not right_open

                    if c== "d":
                        DONE = True
                        rospy.loginfo( "Gripper Done"  ) 
             


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

    parser.add_argument('-g', '--gripper_on',    
                        dest = 'gripper_on',    action = 'store_true',
                        help = 'Turn on Gripper')


    args = parser.parse_args( rospy.myargv( )[ 1: ] ) 

    print("Initializing node... ")
    rospy.init_node("impedance_control_right" )
    my_baxter = JointImpedanceControl( args.record_data )

    rospy.on_shutdown( my_baxter.clean_shutdown )

    # my_baxter.joint_state_listener( )

    if args.gripper_on:
        # If gripper on, move the joints to the grasp_posture
        my_baxter.move2pose( C.RIGHT, C.GRASP_POSE_RIGHT_WIDER, joint_speed = 0.3 )
        my_baxter.move2pose( C.LEFT , C.GRASP_POSE_LEFT_WIDER , joint_speed = 0.3 )

        # Wait gripper to be on, using keyboard interrupt 

        # [Moses C. Nah] [Log] [2021.10.30]
        # Need improvement for the gripper code but still cannot find the reason
        # The problem is the gripper always need calibration, and even though it worked, we need to run launch file.
        # There will be a way to automate this process

        my_baxter.control_gripper( mode = "keyboard" )


    else:
        # #
        # my_baxter.move2pose( C.RIGHT, C.REST_POSE_RIGHT ,        joint_speed = 0.3 )
        # rospy.sleep( 1.0 )
        # my_baxter.move2pose( C.RIGHT, C.MID_POSE_RIGHT  ,        joint_speed = 0.3 )
        # rospy.sleep( 1.0 )
        # my_baxter.move2pose( C.RIGHT, C.REST_POSE_RIGHT ,        joint_speed = 0.3 )
        # rospy.sleep( 1.0 )

        # my_baxter.move2pose( C.LEFT, C.REST_POSE_LEFT ,        joint_speed = 0.3 )
        # rospy.sleep( 1.0 )
        # my_baxter.move2pose( C.LEFT, C.MID_POSE_LEFT  ,        joint_speed = 0.3 )
        # rospy.sleep( 1.0 )
        # my_baxter.move2pose( C.LEFT, C.REST_POSE_LEFT ,        joint_speed = 0.3 )
        # rospy.sleep( 1.0 )

        # my_baxter.move2pose( C.RIGHT, C.REST_POSE_RIGHT ,  joint_speed = 0.3 )
        D    = 1
        toff = 1
        my_baxter.move2pose( C.RIGHT, C.REST_POSE_RIGHT ,  joint_speed = 0.3 )
        my_baxter.control_gripper( mode = "timer" )

        rospy.sleep( 1 )
        my_baxter.joint_impedance(  C.REST_POSE_RIGHT  ,  C.MID_POSE_RIGHT  , D, D + 1 )
        my_baxter.joint_impedance(  C.MID_POSE_RIGHT   ,  C.REST_POSE_RIGHT , D, D + 4 )



if __name__ == "__main__":
    main()