#!/usr/bin/python3

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
from my_utils     import GripperConnect #Logger,

# Local Library, customized messages
from moses_baxter.msg import my_msg


class JointImpedanceControl( object ):
    def __init__( self, publish_data = False ):#, reconfig_server):


        self.publish_data = publish_data           # Boolean, data saved
                                                 # Just define the whole arms in the first place to simplify the code
        self.arms        = list( range( 2 ) )    # Since we have both the left/right arms
        self.kins        = list( range( 2 ) )    # Since we have both the left/right arms
        self.grips       = list( range( 2 ) )    # Since we have both the left/right arms

        self.start_time  = rospy.Time.now()


        self.pub = rospy.Publisher( 'my_baxter' , my_msg ) # The publisher of my message

        self.msg    = my_msg()
        self.msg.on = True

        # Saving the limb objects
        for idx, name in enumerate( [ "right", "left" ] ):
            self.arms[  idx ] = baxter_interface.limb.Limb( name )
            self.kins[  idx ] = baxter_kinematics(          name )
            self.grips[ idx ] = baxter_interface.Gripper(   name )

            # Initialize the gripper
            # [MOSES] [2021.10.31]
            # There might be a way to initialize the gripper rather than running ''roslaunch moses gripper_setup.launch''.
            # Hence, leaving the comments here for future task. Refer to ``gripper_cuff_control.py'' and ``gripper_setup.launch''.
            # self.grips[ idx ].on_type_changed.connect( self.check_calibration )

            self.grips[ idx ].set_holding_force( 100 )  # Initialize the grippers holding force
            self.grips[ idx ].open(  block = False )    # Open the gripper too


                                   # control parameters
        self.rate        = 100.0   # Hz
        self.missed_cmds = 20.0    # Missed cycles before triggering timeout

        # Impedance Parameters for Case 1
        # self.Kq = C.JOINT_IMP1_Kq
        # self.Bq = C.JOINT_IMP1_Bq

        self.Kq = C.JOINT_IMP2_Kq
        self.Bq = C.JOINT_IMP2_Bq

        self.robot_init( )


    def robot_init( self ):
        print("Getting robot state... ")

        self.rs         = baxter_interface.RobotEnable( CHECK_VERSION )
        self.init_state = self.rs.state().enabled

        print("Enabling robot... ")

        # Initializing the gripper
        # [Moses C. Nah] You need to separately save the output to use the grippers
        self.grip_ctrls = [ GripperConnect( arm ) for arm in C.LIMB_NAMES ]

        # Open the grippers
        self.grips[ C.RIGHT ].open(  block = False )
        self.grips[ C.LEFT  ].open(  block = False )

        self.rs.enable()

        print("Running. Ctrl-c to quit")

    # [BACKUP] For initializing the cuff
    # def check_calibration( self, value ):
    #     # if self._gripper.calibrated():
    #     return True
    #     # else:
    #     #     return False

    def get_reference_traj( self, which_arm, pose1, pose2, D, t  ):
        """
            Get the reference trajectory, the basis function is the minimum jerk trajectory
        """
        assert which_arm in [ C.RIGHT, C.LEFT ]

        q0  = dict( )                                   # Making  q0 as a dictionary
        dq0 = dict( )                                   # Making dq0 as a dictionary

        pose1 = self.pose_gen( which_arm, pose1 )       # Regenerating the pose
        pose2 = self.pose_gen( which_arm, pose2 )       # Regenerating the pose

        limb  = C.LIMB_NAMES[ which_arm ]

        # Iterate through the joints
        for joint in C.JOINT_NAMES:

            joint_name = limb + "_" + joint             # [Example] "right_s0"

            tt = t/D if t<=D else 1     # Normalized time as tt, tau is actually the notation but tau is reserved for torque
                                        # If duration bigger than time t than set tt as 1

            q0[  joint_name ] = pose1[ joint_name ] + ( pose2[ joint_name ] - pose1[ joint_name ] ) * (  10 * tt ** 3 - 15 * tt ** 4 +  6 * tt ** 5 )
            dq0[ joint_name ] =               1.0/D * ( pose2[ joint_name ] - pose1[ joint_name ] ) * (  30 * tt ** 2 - 60 * tt ** 3 + 30 * tt ** 4 )

        return q0, dq0

    def joint_impedance( self, which_arm, poses, Ds = None, toffs = None):

        control_rate = rospy.Rate( self.rate )             # set control rate
        assert which_arm in [ C.RIGHT, C.LEFT, C.BOTH ]    # If which_arm is C.BOTH, then we are moving both the left and right arm

        # Making it as a list
        Ds    = [ Ds    ] if isinstance(    Ds , float ) or isinstance(    Ds , int ) else    Ds
        toffs = [ toffs ] if isinstance( toffs , float ) or isinstance( toffs , int ) else toffs


        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot will timeout and disable
        # Regardless of which_arm, setting both the left and right arm set_command
        self.arms[ C.RIGHT ].set_command_timeout( ( 1.0 / self.rate) * self.missed_cmds)
        self.arms[ C.LEFT  ].set_command_timeout( ( 1.0 / self.rate) * self.missed_cmds)


        N  = len( poses )
        assert N > 1                    # N should be bigger than 1, since we need at least initial - final posture of the impedance controller

        tau_R = dict( )
        tau_L = dict( )

        for i in range( N - 1 ):        # Iterating along the poses of the impedance controller

            ts = rospy.Time.now()
            t  = 0                      # Elapsed time

            while not rospy.is_shutdown() and t <= Ds[ i ] + toffs[ i ]:

                if not self.rs.state().enabled:
                    rospy.logerr("impedance example failed to meet, specified control rate timeout.")
                    break

                t = ( rospy.Time.now( ) - ts ).to_sec( )        # The elapsed time of the simulatoin

                q0_R, dq0_R = self.get_reference_traj( C.RIGHT, poses[ i ], poses[ i + 1 ], Ds[ i ], t )
                q0_L, dq0_L = self.get_reference_traj( C.LEFT , poses[ i ], poses[ i + 1 ], Ds[ i ], t )

                q_R   = self.arms[ C.RIGHT ].joint_angles()
                dq_R  = self.arms[ C.RIGHT ].joint_velocities()

                q_L   = self.arms[ C.LEFT  ].joint_angles()
                dq_L  = self.arms[ C.LEFT  ].joint_velocities()

                for j, joint in enumerate( C.JOINT_NAMES ):

                    right_name = "right_" + joint             # [Example] "right_s0"
                    left_name  =  "left_" + joint             # [Example] "left_s0"

                    tau_R[ right_name ]  =  self.Kq[ joint ] * (  q0_R[ right_name ] -  q_R[ right_name ] )   # The Stiffness portion
                    tau_R[ right_name ] +=  self.Bq[ joint ] * ( dq0_R[ right_name ] - dq_R[ right_name ] )   # The Damping   portion

                    tau_L[ left_name ]   =  self.Kq[ joint ] * (  q0_L[  left_name ] -  q_L[  left_name ] )   # The Stiffness portion
                    tau_L[ left_name ]  +=  self.Bq[ joint ] * ( dq0_L[  left_name ] - dq_L[  left_name ] )   # The Damping   portion

                    if self.publish_data:
                        self.msg.q0_L[ j ]  =  q0_L[  left_name ]
                        self.msg.q_L[ j ]   =   q_L[  left_name ]
                        self.msg.dq_L[ j ]  =  dq_L[  left_name ]
                        self.msg.tau_L[ j ] = tau_L[  left_name ]

                        self.msg.q0_R[ j ]  =  q0_R[ right_name ]
                        self.msg.q_R[ j ]   =   q_R[ right_name ]
                        self.msg.dq_R[ j ]  =  dq_R[ right_name ]
                        self.msg.tau_R[ j ] = tau_R[ right_name ]


                    if   which_arm == C.RIGHT:

                        self.arms[ C.RIGHT ].set_joint_torques( tau_R )

                    elif which_arm == C.LEFT:

                        self.arms[ C.LEFT  ].set_joint_torques( tau_L )

                    elif which_arm == C.BOTH:

                        self.arms[ C.RIGHT ].set_joint_torques( tau_R )
                        self.arms[ C.LEFT  ].set_joint_torques( tau_L )

                    else:
                        NotImplementedError( )

                # Publish the message
                self.msg.stamp = ( rospy.Time.now() - ts).to_sec( )
                self.pub.publish( self.msg )
                control_rate.sleep()


    def pose_gen( self, which_arm, old_pose ):

        assert which_arm in [ C.RIGHT, C.LEFT ]

        new_pose = dict( )

        if   which_arm == C.RIGHT:
            for name in C.JOINT_NAMES:
                new_pose[ "right_" + name ] = old_pose[ name ]

        elif which_arm == C.LEFT:
            for name in C.JOINT_NAMES:
                if name in C.JOINT_TO_FLIP:
                    new_pose[ "left_" + name ] = -old_pose[ name ]
                else:
                    new_pose[ "left_" + name ] =  old_pose[ name ]

        else:
            NotImplementedError( )

        return new_pose

    def move2pose( self, which_arm, poses, wait_time = 1, joint_speed = 0.1 ):

        # If which_arm is neither 0 nor 1, assert
        # Iterate along the poses

        # If single dictionary element, then make it as a list
        poses = [ poses ] if isinstance( poses , dict ) else poses

        for idx, pose in enumerate( poses ):
            pose = self.pose_gen( which_arm, pose )

            self.arms[ which_arm ].set_joint_position_speed( joint_speed )
            self.arms[ which_arm ].move_to_joint_positions( pose )

            rospy.sleep( wait_time )

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
            self.msg.on = False
            self.pub.publish( self.msg )

            self.rs.disable()


def main():

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser  = argparse.ArgumentParser( formatter_class = arg_fmt )
    parser.add_argument('-s', '--publish_data',
                        dest = 'publish_data',   action = 'store_true',
                        help = 'Save the Data')

    parser.add_argument('-g', '--gripper_on',
                        dest = 'gripper_on',    action = 'store_true',
                        help = 'Turn on Gripper')

    args = parser.parse_args( rospy.myargv( )[ 1: ] )

    print( "Initializing node... " )
    rospy.init_node( "impedance_control_right" )
    my_baxter = JointImpedanceControl( args.publish_data )

    rospy.on_shutdown( my_baxter.clean_shutdown )

    if args.gripper_on:

        #  Can put arrays of pose
        my_baxter.move2pose( C.RIGHT, [ C.REST_POSE, C.GRASP_POSE_WIDER, C.WIDE_POSE, C.REST_POSE], wait_time = 1, joint_speed = 0.3 )
        # my_baxter.move2pose( C.LEFT , [ C.REST_POSE, C.GRASP_POSE_WIDER, C.WIDE_POSE, C.REST_POSE], wait_time = 1, joint_speed = 0.3 )


        # [Moses C. Nah] [Log] [2021.10.30]
        # Need improvement for the gripper code but still cannot find the reason
        # The problem is the gripper always need calibration, and even though it worked, we need to run launch file.
        # There will be a way to automate this process
        # my_baxter.control_gripper( mode = "keyboard" )


    else:
        # rospy.sleep( 1
        # my_baxter.move2pose( C.LEFT , C.GRASP_POSE, wait_time = 2, joint_speed = 0.1 )
        my_baxter.move2pose( C.RIGHT, C.GRASP_POSE, wait_time = 2, joint_speed = 0.2 )
        my_baxter.move2pose( C.LEFT,  C.GRASP_POSE, wait_time = 2, joint_speed = 0.2 )

        my_baxter.control_gripper( mode = "timer" )

        # my_baxter.move2pose( C.LEFT , C.GRASP_POSE, wait_time = 2, joint_speed = 0.2 )
        my_baxter.joint_impedance(  C.BOTH, [ C.GRASP_POSE, C.MID_POSE, C.FINAL_POSE  ] , Ds = [1.0, 1.0], toffs = [0.1, 2.0]  )
        # my_baxter.joint_impedance(  C.BOTH, [ C.GR`ASP_POSE, C.MID_POSE  ] , Ds = [1.0], toffs = [2.0]  )






if __name__ == "__main__":
    main()
