#!/usr/bin/python3

"""
# =========================================================== #
# [Author            ] Moses C. Nah
# [Email             ] mosesnah@mit.edu
# [Date Created      ] 2022.01.26
# [Last Modification ] 2022.04.17
# =========================================================== #
"""

import sys
import pdb
import time
import rospy
import nlopt
import argparse
import numpy             as np
import matplotlib.pyplot as plt

# BAXTER Libraries
import baxter_interface
import baxter_external_devices    # For keyboard interrupt
from   baxter_interface           import CHECK_VERSION
from   baxter_pykdl               import baxter_kinematics

# Ros related Libraries
from std_msgs.msg                 import Empty
from sensor_msgs.msg              import JointState

# Local Library, under moses/scripts
from my_constants import Constants as C
from my_utils     import GripperConnect, Logger

# Local Library, customized messages
from moses_baxter.msg import my_msg

def pose_right2left( pose: dict ):
    """
        Changing the dictionary key's prefix name from "right_" to "left_", and flipping the sign too
        The reason why we don't have left2right is because we will follow the "right-hand" convention,
        meaning, all the constants in "my_constants.py" are saved as "right" hand informatino

        Arguments:
            [1] pose ( dict ): 7-element dictionary with keys "right_" + s0, s1, e0, e1, w0, w1, w2
    """
    assert all( [ c in pose.keys( ) for c in C.JOINT_NAMES[ "right" ] ] )   # check whether the given dictionary has all the "right_" + s0, s1, e0, e1, w0, w1 and w2 on the keys.

    new_pose = dict( )

    for right_name in C.JOINT_NAMES[ "right" ]:
        left_name = C.RIGHT2LEFT[ right_name ]
        new_pose[ left_name ] = C.LEFT_JOINT_SIGN[ left_name ] * pose[ right_name ]

    return new_pose



class Controller( object ):
    """
        Primitive Controller Object
    """

    def __init__( self, robot ):

        self.robot = robot

    def gen_dict( self, which_arm: str, arr: np.ndarray ):

        assert which_arm in [ "right", "left" ]
        assert len( arr ) == 7

        new = dict( )

        for i, joint_name in enumerate( C.JOINT_NAMES[ which_arm ] ):
            new[ joint_name ] = arr[ i ]

        return new

    def poses_delta( self, which_arm: str, pose1: dict, pose2: dict ):
        """
            conduct pose2 - pose1 for each element
        """
        assert which_arm in [ "right", "left" ]

        assert all( [ c in pose1.keys( ) for c in C.JOINT_NAMES[ which_arm ] ] )
        assert all( [ c in pose2.keys( ) for c in C.JOINT_NAMES[ which_arm ] ] )

        return { key: pose2[ key ] - pose1[ key ] for key in pose1.keys( ) }


class PrintJointController( Controller ):
    """
        Printing out the joint values by moving around the limbs.

        This controller is useful when we want to design the movements.
    """
    def __init__( self, robot ):

        super().__init__( robot )
        self.type  = "print_joint_controller"

    def run( self ):

        DONE = False
        while not DONE:
            typed_letter = baxter_external_devices.getch()

            if typed_letter:

                if typed_letter in ['\x1b', '\x03']:   #catch Esc or ctrl-c
                    DONE = True
                    rospy.signal_shutdown( "[LOG] EXITTING PRINT JOINT MODE.")

                if typed_letter == "p":

                    print( "=" * 100 )
                    for limb_name in [ "right", "left" ]:
                        joint_angles = self.robot.get_arm_pose( limb_name )

                        for joint_name in C.JOINT_NAMES[ limb_name ]:
                            print( "'{0}' : {1:.10f},".format( joint_name, joint_angles[ joint_name ] ) )
                    print( "=" * 100 )

class JointImpedanceController( Controller ):
    """
        1st-order joint-space impedance controller
        The equation is as follows:
            tau = Kq( q0 - q ) + Bq( dq0 - dq ) + tau_G

        tau_G, the gravity compensation torque is conducted by Baxter alone
    """

    def __init__( self, robot, is_save_data = False ):

        super().__init__( robot )
        self.type          = "joint_impedance_controller"
        self.is_save_data  = is_save_data

        a   = 0.2 # The ratio between stiffness and dampling
        BqR = { key : a * val for key, val in C.JOINT_IMP_Kq_R.items() }
        BqL = { key : a * val for key, val in C.JOINT_IMP_Kq_L.items() }

        # Since the left and right moves can be conducted independently, also saving the values independently
        self.Kq      = { "right": C.JOINT_IMP_Kq_R , "left": C.JOINT_IMP_Kq_L }
        self.Bq      = { "right": BqR, "left": BqL }
        self.moves   = { "right": [ ], "left": [ ] }
        self.n_moves = { "right":   0, "left":   0 }


        if self.is_save_data:
            self.pub    = rospy.Publisher( 'my_baxter' , my_msg ) # The publisher of my message
            self.msg    = my_msg()
            self.msg.on = False

    def min_jerk_traj( self, t: float, ti: float, tf: float, pi: float, pf: float, D: float ):
        """
            Returning the 1D position and velocity data at time t of the minimum-jerk-trajectory ( current time )
            Time should start at t = 0
            Note that the minimum-jerk-trajectory remains at the initial (respectively, final) posture before (after) the movement.

            Arguments
            ---------
                [1] t : current time
                [2] ti: start of the movement
                [3] tf: end   of the movement
                [4] pi: initial ( reference ) posture
                [5] pf: final   ( reference ) posture
                [6]  D: duration

        """

        assert  t >=  0 and ti >= 0 and tf >= 0 and D >= 0
        assert tf >= ti

        if   t <= ti:
            pos = pi
            vel = 0

        elif ti < t <= tf:
            tau = ( t - ti ) / D                                                # Normalized time
            pos =    pi + ( pf - pi ) * ( 10 * tau ** 3 - 15 * tau ** 4 +  6 * tau ** 5 )
            vel = 1 / D * ( pf - pi ) * ( 30 * tau ** 2 - 60 * tau ** 3 + 30 * tau ** 4 )

        else:
            pos = pf
            vel = 0

        return pos, vel

    def reset( self ):
        self.moves   = { "right" : [ ], "left" : [ ] }
        self.n_moves = { "right" :   0, "left" :   0 }

    def add_movement( self, which_arm: str, pose_init: dict, pose_final: dict, duration: float, toff: float ):
        """
            Adding the ZTT (Zero-torque-trajectory) information to the movement.
            ZTT uses the minimum-jerk-trajectory (quintic 5-th order polynomial) as the basis function.

            Arguments:

                [1] which_arm  (str)   : either "right" or "left"
                [2] pose_init  (dict)  : initial "right" posture of the ZTT (Zero-torque-trajectory), note that if which_arm is "left", then should change pose from right2left
                [3] pose_final (dict)  : final   "right" posture of the ZTT (Zero-torque-trajectory), note that if which_arm is "left", then should change pose from right2left
                [4] duration   (float) : the    duration of the ZTT (Zero-torque-trajectory)
                [5] toff       (float) : time offset with respect to the previous movements, can be negative value since it can overlap with the previous one
                                         If the movement is the first one (i.e., self.n_l_moves == 0 ), automatically set toff as zero.

        """

        assert which_arm in [ "right", "left" ]
        assert duration >= 0
        assert all( [ c in pose_init.keys(  ) for c in C.JOINT_NAMES[ which_arm ] ] )
        assert all( [ c in pose_final.keys( ) for c in C.JOINT_NAMES[ which_arm ] ] )

        move = dict( )

        if self.n_moves[ which_arm ] == 0:
            """
                For the first movement, toff and ti must be set as 0.
            """

            pi   = pose_init
            pf   = pose_final
            toff = 0
            ti   = 0
            tf   = duration
            D    = duration

        else:
            """
                For movements after the 2nd, we must save the poses_delta
            """
            idx_prev = self.n_moves[ which_arm ] - 1                            # Index of the previous movement
            tf_prev  = self.moves[ which_arm ][ idx_prev ][ "tf" ]              # Getting the final time of the previous movement

            tmpi = pose_init
            tmpf = pose_final

            pi   = self.gen_dict( which_arm, np.zeros( 7 ) )                    # Zero initial posture
            pf   = self.poses_delta( which_arm, tmpi, tmpf )                               # Doing tmpf - tmpi
            toff = toff
            ti   = tf_prev + toff             if tf_prev + toff            >= 0 else 0
            tf   = tf_prev + toff + duration  if tf_prev + toff + duration >= 0 else duration
            D    = duration


        move[ "toff" ] = toff
        move[ "ti"   ] = ti
        move[ "tf"   ] = tf
        move[ "pi"   ] = pi
        move[ "pf"   ] = pf
        move[ "D"    ] = D

        self.moves[   which_arm ].append( move )
        self.n_moves[ which_arm ] += 1


    def get_reference_traj( self, which_arm: str, t: float ):

        assert which_arm in [ "right", "left" ]
        assert t >= 0
        assert self.n_moves[ which_arm ] >= 1                                   # Should have more than or equal to 1 movement to get the reference trajectory

        q0  = self.gen_dict( which_arm, np.zeros( 7 ) )
        dq0 = self.gen_dict( which_arm, np.zeros( 7 ) )

        for joint_name in C.JOINT_NAMES[ which_arm ]:
            for move in self.moves[ which_arm ]:

                ti  = move[ "ti" ]
                tf  = move[ "tf" ]
                pi  = move[ "pi" ][ joint_name ]
                pf  = move[ "pf" ][ joint_name ]
                D   = move[ "D"  ]

                p, v = self.min_jerk_traj( t, ti, tf, pi, pf, D )
                q0[  joint_name ] += p
                dq0[ joint_name ] += v

        return q0, dq0

    def run( self ):

        assert self.n_moves[ "right" ] >= 1 or self.n_moves[ "left" ] >= 1

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot will timeout and disable
        # Regardless of which_arm, setting both the left and right arm set_command
        control_rate = rospy.Rate( self.robot.rate )             # set control rate

        self.robot.arms[ "right" ].set_command_timeout( ( 1.0 / self.robot.rate ) * self.robot.missed_cmds )
        self.robot.arms[ "left"  ].set_command_timeout( ( 1.0 / self.robot.rate ) * self.robot.missed_cmds )

        # The initial time for the simulation
        ts = rospy.Time.now( )
        t  = 0
        while not rospy.is_shutdown( ) and t <= 8:

            if not self.robot.rs.state( ).enabled:
                rospy.logerr( "Impedance example failed to meet, specified control rate timeout." )
                break

            # The elapsed time
            t = ( rospy.Time.now( ) - ts ).to_sec( )

            q0_R, dq0_R = self.get_reference_traj( which_arm = "right",  t = t )
            q0_L, dq0_L = self.get_reference_traj( which_arm = "left" ,  t = t )

            q   = { "right": None , "left": None  }
            dq  = { "right": None , "left": None  }
            q0  = { "right": q0_R , "left": q0_L  }
            dq0 = { "right": dq0_R, "left": dq0_L }
            tau = { "right": self.gen_dict( "right", np.zeros( 7 ) ) ,
                     "left": self.gen_dict( "left" , np.zeros( 7 ) ) }

            if self.is_save_data:
                self.msg.on = True

            for which_arm in [ "right" , "left" ]:

                # Need to check whether the movement exists
                if self.n_moves[ which_arm ] == 0:
                    continue

                # Get current position and velocities of the joints
                q[ which_arm  ] = self.robot.get_arm_pose(     which_arm )
                dq[ which_arm ] = self.robot.get_arm_velocity( which_arm )

                for joint_name in C.JOINT_NAMES[ which_arm ]:
                    tau[ which_arm ][ joint_name ]  =  self.Kq[ which_arm ][ joint_name ] * (  q0[ which_arm ][ joint_name ] -  q[ which_arm ][ joint_name ] )
                    tau[ which_arm ][ joint_name ] +=  self.Bq[ which_arm ][ joint_name ] * ( dq0[ which_arm ][ joint_name ] - dq[ which_arm ][ joint_name ] )

                if self.is_save_data:
                    # Sadly, we need to use multiplelines to publish the data.
                    # 2D array will be useful, but we use 1D array which is
                    self.msg.stamp      = t

                    if   which_arm == "left":
                        for j, joint_name in enumerate( C.JOINT_NAMES[ "left" ] ):
                            self.msg.q0_L[ j  ] =   q0[ "left"  ][ joint_name ]
                            self.msg.dq0_L[ j ] =  dq0[ "left"  ][ joint_name ]
                            self.msg.q_L[ j   ] =    q[ "left"  ][ joint_name ]
                            self.msg.dq_L[ j  ] =   dq[ "left"  ][ joint_name ]
                            self.msg.tau_L[ j ] =  tau[ "left"  ][ joint_name ]

                    elif which_arm == "right":
                        for j, joint_name in enumerate( C.JOINT_NAMES[ "right" ] ):
                            self.msg.q0_R[ j ]  =   q0[ "right" ][ joint_name ]
                            self.msg.dq0_R[ j ] =  dq0[ "right" ][ joint_name ]
                            self.msg.q_R[ j ]   =    q[ "right" ][ joint_name ]
                            self.msg.dq_R[ j ]  =   dq[ "right" ][ joint_name ]
                            self.msg.tau_R[ j ] =  tau[ "right" ][ joint_name ]
                    else:
                        pass

            self.robot.arms[ "right" ].set_joint_torques( tau[ "right" ] )
            self.robot.arms[ "left"  ].set_joint_torques( tau[ "left"  ] )

            # [Moses C. Nah]
            # DO NOT ERASE!! Erasing it will lead to unstable controller
            control_rate.sleep()

            if self.is_save_data:
                self.pub.publish( self.msg )

        # Turn off message if done.
        if self.is_save_data:
            self.msg.on = False

    def move2pose( self, pose: dict, duration: float, toff:float ) :
        """
            Symmetrically move both arms to the specific posture
            The initial posture for this function is simply the curent posture, hence no need to specify the posture
        """
        assert duration >= 0
        assert toff     >= 0
        assert all( [ c in pose.keys(  ) for c in C.JOINT_NAMES[ "right" ] ] )

        # First, get the current position of right and left arm
        pose_R = self.robot.get_arm_pose( "right" )
        pose_L = self.robot.get_arm_pose( "left"  )

        pose2go_R = pose
        pose2go_L = pose_right2left( pose )

        self.add_movement( which_arm = "right" , pose_init = pose_R, pose_final = pose2go_R, duration = duration, toff = 0 )
        self.add_movement( which_arm = "left"  , pose_init = pose_L, pose_final = pose2go_L, duration = duration, toff = 0 )

        self.run( )

        rospy.sleep( toff )

        self.reset( ) # Once the movement is initiated, reset the whole data


class JointPositionController( Controller ):
    """
        Pure position controller
        Using innate Baxter function
            [1] set_joint_position_speed( joint_speed )
            [2] move_to_joint_positions( pose )
        to conduct the movements

    """

    def __init__( self, robot ):

        super().__init__( robot )

        self.type    = "joint_position_controller"
        self.moves   = []                               # Elements should be dictionary Type
        self.n_moves = 0

    def add_movement( self, which_arm: str, pose2go: dict, joint_vel: float, toff: float ):
        """
            Adding details of the movement to be conducted
            The problem of this controller is that we cannot move both limbs simultaneously.
            Hence each right/left limb will move separately.

            Arguments:
                [1] which_arm (str)   : Either "right" or "left"
                [2] pose2go   (dict)  : Keys "which_arm" + s0, s1, e0, e1, w0, w1, w2 with corresponding values.
                                        Note that the keys between "which_arm" and "pose2go" should match each other.
                [3] joint_vel (float) : Ranged (0,1) and argument for "set_joint_position_speed" method
                [4] toff      (float) : time offset with respect to the previous movements, must be nonnegative
        """

        assert which_arm in [ "right", "left" ]
        assert all( [ c in pose2go.keys( ) for c in C.JOINT_NAMES[ which_arm ] ] )   # check whether the given dictionary has all the "right_" + s0, s1, e0, e1, w0, w1 and w2 on the keys.
        assert joint_vel >= 0 and joint_vel <= 1
        assert      toff >= 0

        # Generating the movement details
        move = dict( )
        move[ "which_arm" ] = which_arm
        move[ "pose"      ] = pose2go
        move[ "joint_vel" ] = joint_vel
        move[ "toff"      ] = toff

        # Adding the movement details
        self.moves.append( move )
        self.n_moves += 1

    def reset( self ):
        # Emptying out all the movements
        self.moves   = []
        self.n_moves = 0

    def run( self ):
        """
            Once finish adding the movements, initiate the movements
        """

        assert self.n_moves >= 1

        # Initiating the movements step by step\
        for i, move in enumerate( self.moves ) :

            print( "[LOG] INITIATING THE {0:}-th movement".format( i + 1 ) )

            which_arm = move[ "which_arm" ]
            pose      = move[ "pose"      ]
            vel       = move[ "joint_vel" ]
            toff      = move[ "toff"      ]

            self.robot.arms[ which_arm ].set_joint_position_speed( vel )
            self.robot.arms[ which_arm ].move_to_joint_positions( pose )
            rospy.sleep( toff )


    def run_example( self, type = "default" ):
        """
            Just some simple repertoire for this controller
        """

        if   type == "default":

            # All the constants are saved in "right" hand, hence should parse the movements a bit
            POSE1_R = C.GRASP_POSE
            POSE1_L = pose_right2left( C.GRASP_POSE )

            self.add_movement( which_arm = "right" , pose2go = POSE1_R, joint_vel = 0.2, toff = 5 )
            self.add_movement( which_arm = "left"  , pose2go = POSE1_L, joint_vel = 0.2, toff = 5 )

            POSE2_R = C.LIFT_POSE
            POSE2_L = pose_right2left( C.LIFT_POSE )

            self.add_movement( which_arm = "left"  , pose2go = POSE2_L , joint_vel = 0.2, toff = 5 )
            self.add_movement( which_arm = "right" , pose2go = POSE2_R , joint_vel = 0.2, toff = 5 )

            POSE3_R = C.FINAL_POSE
            POSE3_L = pose_right2left( C.FINAL_POSE )

            self.add_movement( which_arm = "right" , pose2go = POSE3_R, joint_vel = 0.2, toff = 5 )
            self.add_movement( which_arm = "left"  , pose2go = POSE3_L, joint_vel = 0.2, toff = 5 )

            self.run( )

        elif type == "swing":
            pass

        else:
            pass

        self.reset( )


class Baxter( object ):

    # ================================================================ #
    # ======================== INIT  FUNCTIONS ======================= #
    # ================================================================ #

    def __init__( self, args ):

        self.args         = args
        self.arms         = dict( )
        self.kins         = dict( )
        self.grips        = dict( )

        self.start_time   = rospy.Time.now()

        self.pub    = rospy.Publisher( 'my_baxter' , my_msg )
        self.msg    = my_msg()
        self.msg.on = True

        # Saving the limb objects
        for limb_name in [ "right", "left" ]:
            self.arms[  limb_name ] = baxter_interface.limb.Limb( limb_name )
            self.kins[  limb_name ] = baxter_kinematics(          limb_name )
            self.grips[ limb_name ] = baxter_interface.Gripper(   limb_name )

            # Initialize the gripper
            # [MOSES] [2021.10.31]
            # There might be a way to initialize the gripper rather than running ''roslaunch moses gripper_setup.launch''.
            # Hence, leaving the comments here for future task. Refer to ``gripper_cuff_control.py'' and ``gripper_setup.launch''.
            # self.grips[ idx ].on_type_changed.connect( self.check_calibration )

            # self.grips[ idx ].set_holding_force( 100 )  # Initialize the grippers holding force
            # self.grips[ idx ].open(  block = False )    # Open the gripper too

                                   # control parameters
        self.rate        = 100.0   # Hz
        self.missed_cmds = 20.0    # Missed cycles before triggering timeout

        self.robot_init( )


    def robot_init( self ):
        print( "[LOG] [INIT IN PROGRESS] ....." )

        self.rs         = baxter_interface.RobotEnable( CHECK_VERSION )         # Getting Robot State
        self.init_state = self.rs.state().enabled                               # Enabling Robot

        # Initializing the gripper
        # [Moses C. Nah] You need to separately save the output to use the grippers
        #                Hence, adding this seemingly-redundant Code
        self.grip_ctrls = [ GripperConnect( arm ) for arm in [ "left", "right" ] ]

        # [BACKUP] When you want to control the grippers
        # Check whether the gripper is opened (100) or closed( 0 )
        # threshold value is simply 50

        # self.open_gripper( )
        self.rs.enable()

        print( "[LOG] [INIT COMPLETE] Ctrl-c to quit" )

    def clean_shutdown( self ):

        print( "[LOG] [SHUTTING DOWN ROBOT]" )

        for limb_name in [ "left", "right" ]:
            self.arms[ limb_name ].exit_control_mode( )

        if not self.init_state and self.rs.state().enabled:

            self.rs.disable()

    # ---------------------------------------------------------------- #
    # ------------------------ INIT  FUNCTIONS ----------------------- #
    # ---------------------------------------------------------------- #

    # ================================================================ #
    # ======================== BASIC FUNCTIONS ======================= #
    # ================================================================ #

    def get_arm_pose( self, which_arm: str ):
        assert which_arm in [ "right", "left" ]
        return self.arms[ which_arm  ].joint_angles( )

    def get_arm_velocity( self, which_arm: str ):
        assert which_arm in [ "right", "left" ]
        return self.arms[ which_arm  ].joint_velocities( )

    def get_gripper_pos( self, which_arm: str ) :
        assert which_arm in [ "right", "left" ]
        return self.grips[ which_arm ].position( )

    def open_gripper( self  ):
        self.grips[ "right" ].open(  block = False )
        self.grips[  "left" ].open(  block = False )

    def close_gripper( self ):
        self.grips[ "right" ].close(  block = False )
        self.grips[  "left" ].close(  block = False )


    # ---------------------------------------------------------------- #
    # ------------------------ BASIC FUNCTIONS ----------------------- #
    # ---------------------------------------------------------------- #


def main():

    parser  = argparse.ArgumentParser( formatter_class = argparse.RawTextHelpFormatter )
    parser.add_argument('-s', '--save_data',
                        dest = 'save_data',   action = 'store_true',
                        help = 'Save the Data')

    parser.add_argument('-o', '--run_optimization',
                        dest = 'is_run_optimization',    action = 'store_true',
                        help = 'Running the optimization of the whole process')

    parser.add_argument('-c', '--controller',
                        dest = 'ctrl_type',    action = 'store', type = str,
                        help = C.CONTROLLER_DESCRIPTIONS )


    args = parser.parse_args( rospy.myargv( )[ 1: ] )

    print( "Initializing node... " )
    rospy.init_node( "impedance_control_right" )
    my_baxter = Baxter( args )
    rospy.on_shutdown( my_baxter.clean_shutdown )


    # ==================================================================================================== #
    # Running the simulation
    # ==================================================================================================== #

    if args.is_run_optimization:

        my_log = Logger( record_data = True )

        # Find the input parameters (input_pars) that are aimed to be optimized
        # Possible options (written in integer values) are as follows
        # [REF] https://nlopt.readthedocs.io/en/latest/NLopt_Algorithms/

        # Define the controller
        my_ctrl = JointImpedanceController( my_baxter, is_save_data = False )
        print( "[LOG] OPTIMIZATION MODE" )


        #   idx are                 0                   1               2               3                  4
        idx_opt   = [ nlopt.GN_DIRECT_L, nlopt.GN_DIRECT_L_RAND, nlopt.GN_DIRECT, nlopt.GN_CRS2_LM, nlopt.GN_ESCH  ]
        idx       = 3


        #  Upper/Lower Bound   s1    e1     w1   D1   D2    a
        lb    = np.array( [ -0.65, 0.31, -0.76, 0.8, 0.5, -0.6 ] )
        ub    = np.array( [ -0.40, 0.73, -0.25, 1.5, 1.5,  0.5 ] )
        n_opt = 6

        algorithm = idx_opt[ idx ]                                              # Selecting the algorithm to be executed
        opt       = nlopt.opt( algorithm, n_opt )                               # Defining the class for optimization

        opt.set_lower_bounds( lb )
        opt.set_upper_bounds( ub )
        opt.set_maxeval( 40 )

        init = ( lb + ub ) * 0.5 + 0.05 * lb                                    # Setting an arbitrary non-zero initial step


        # Uncomment the following 3 sentences in case if the tablecloth is not equipped.
        # my_baxter.move2pose( C.RIGHT, C.GRASP_POSE, wait_time = 2, joint_speed = 0.2 )
        # my_baxter.move2pose( C.LEFT,  C.GRASP_POSE, wait_time = 2, joint_speed = 0.2 )
        # my_baxter.control_gripper( mode = "timer" )

        tmp_t = 98 # If the coverage is higher than this value, simply set it as 100 and stop optimizaiton

        def nlopt_objective( pars, grad ):                                      # Defining the objective function that we are aimed to optimize.

            # Run Baxter - Code implementation here
            # Manipulating the cloth
            # [STEP #1] Move to initial posture
            # [STEP #2] Initiate movement
            # Go to the initial posture
            my_ctrl.move2pose( C.GRASP_POSE, duration = 5, toff = 0.1 )


            POSE1_R = C.GRASP_POSE
            POSE1_L = pose_right2left( C.GRASP_POSE  )

            # s0, s1, e0, e1, w0, w1, w2 number
            POSE2_R = my_ctrl.gen_dict( "right", np.array( [ C.GRASP_POSE[ "right_s0" ],
                                                                              pars[ 0 ],
                                                             C.GRASP_POSE[ "right_e0" ],
                                                                              pars[ 1 ],
                                                             C.GRASP_POSE[ "right_w0" ],
                                                                              pars[ 2 ],
                                                             C.GRASP_POSE[ "right_w2" ] ] ) )
            POSE2_L = pose_right2left( POSE2_R  )

            POSE3_R = C.FINAL_POSE
            POSE3_L = pose_right2left( C.FINAL_POSE  )


            my_ctrl.add_movement( which_arm = "right", pose_init = POSE1_R, pose_final = POSE2_R, duration = pars[ 3 ], toff = 0.0                   )
            my_ctrl.add_movement( which_arm = "right", pose_init = POSE2_R, pose_final = POSE3_R, duration = pars[ 4 ], toff = pars[ 3 ] * pars[ 5 ] )

            my_ctrl.add_movement( which_arm = "left" , pose_init = POSE1_L, pose_final = POSE2_L, duration = pars[ 3 ], toff = 0.0                   )
            my_ctrl.add_movement( which_arm = "left" , pose_init = POSE2_L, pose_final = POSE3_L, duration = pars[ 4 ], toff = pars[ 3 ] * pars[ 5 ] )

            my_log.write( "[Iteration] " + str( opt.get_numevals( ) + 1) + " [parameters] " + str( pars ) )

            my_ctrl.run( )
            rospy.sleep( 3 )

            # Get Baxter's tablecloth performance
            obj = rospy.get_param( 'my_obj_func' )
            if obj >= tmp_t:
                obj = 100.0

            my_log.write( " [obj] " + str( obj ) + "\n" )
            my_ctrl.reset( )

            # my_ctrl.move2pose(  C.LIFT_POSE, duration = 5, toff = 0.1 )



            return 100.0 - obj # Inverting the value


        # input( "Ready for optimization, press any key to continue" )

        opt.set_min_objective( nlopt_objective )
        opt.set_stopval( -1    )                                                # If value is within 98~100% (i.e., 0~2%)
        xopt = opt.optimize( init )                                             # Start at the mid-point of the lower and upper bound


    # ============================================================================= #
    # ============================= NORMAL EXECUTION ============================== #
    # ============================================================================= #

    elif not args.is_run_optimization:

        # =============================================================== #
        # ================= JOINT POSITION CONTROLLER =================== #
        # =============================================================== #
        if   args.ctrl_type == "joint_position_controller":
            my_ctrl = JointPositionController( my_baxter )
            my_ctrl.run_example( type = "default" )


        # =============================================================== #
        # ================= JOINT IMPEDANCE CONTROLLER ================== #
        # =============================================================== #

        elif args.ctrl_type == "joint_impedance_controller":
            my_ctrl = JointImpedanceController( my_baxter, is_save_data = args.save_data )
            my_ctrl.move2pose( C.GRASP_POSE, duration = 5, toff = 1 )

            # rospy.sleep( 5 )
            # my_baxter.close_gripper()
            # input( "Ready for optimization, press any key to continue" )
            # my_ctrl.move2pose( C.LIFT_POSE , duration = 5, toff = 1 )

            # # Design the movements in detail
            POSE1_R = C.GRASP_POSE
            POSE1_L = pose_right2left( C.GRASP_POSE  )

            POSE2_R = C.MID_POSE
            POSE2_L = pose_right2left( C.MID_POSE    )

            POSE3_R = C.FINAL_POSE
            POSE3_L = pose_right2left( C.FINAL_POSE  )
            #
            my_ctrl.add_movement( which_arm = "right", pose_init = POSE1_R, pose_final = POSE2_R, duration = 0.87, toff =  0.0 )
            my_ctrl.add_movement( which_arm = "left" , pose_init = POSE1_L, pose_final = POSE2_L, duration = 0.87, toff =  0.0 )

            my_ctrl.add_movement( which_arm = "right", pose_init = POSE2_R, pose_final = POSE3_R, duration = 0.80, toff = 0.87 * 0.45 )
            my_ctrl.add_movement( which_arm = "left" , pose_init = POSE2_L, pose_final = POSE3_L, duration = 0.80, toff = 0.87 * 0.45 )
            #
            my_ctrl.run( )

        # =============================================================== #
        # ================== PRINT JOINT CONTROLLER ===================== #
        # =============================================================== #

        elif args.ctrl_type == "print_joint_controller":
            my_ctrl = PrintJointController( my_baxter )
            my_ctrl.run( )


if __name__ == "__main__":
    main()
