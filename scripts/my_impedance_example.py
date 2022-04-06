#!/usr/bin/python3

# Source code from following:
# [REF] https://github.com/tony1994513/Impedance-control/blob/master/impedcontol.py


import argparse
import time
import rospy
import numpy as np
import sys
import nlopt

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

class Controller( object ):
    """
        Primitive Controller Object
    """

    def __init__( self, robot ):

        self.robot = robot

        # Internal variables, DO NOT CHANGE!
        # Using kinematic symmetry
        self._r_sign = { "s0": +1, "s1": +1, "e0": +1, "e1": +1, "w0": +1, "w1": +1, "w2": +1 }       # Whether it is plus or minus of the value of the joint
        self._l_sign = { "s0": -1, "s1": +1, "e0": -1, "e1": +1, "w0": -1, "w1": +1, "w2": -1 }
        self._signs  = {  C.RIGHT: self._r_sign , C.LEFT: self._l_sign   }
        self._type   = {  C.RIGHT: "right"      , C.LEFT: "left"         }

class JointImpedanceController( Controller ):
    """
        First-order Joint Impedance Controller
    """

    def __init__( self, robot, Kq, Bq ):

        self.type  = "joint_impedance_controller"

        # Impedance Parameters for the robot,
        self.Kq = Kq
        self.Bq = Bq

        # The number of moves and its details
        # Since the left and right moves can be conducted independently, also saving it independently
        self.moves   = { C.RIGHT : [ ], C.LEFT : [ ] }
        self.n_moves = { C.RIGHT :   0, C.LEFT :   0 }


    def get_reference_traj( self, t, ti, tf, pi, pf, D ):
        """
            Returning the position and velocity data at time t ( current time )
            Time should start at t = 0

            Arguments
            ---------
                [1] t : current time
                [2] ti: start of the movement
                [3] tf: end   of the movement
                [4] pi: initial ( reference ) posture
                [5] pf: final   ( reference ) posture
                [6]  D: duration

        """

        # All the time variables must be higher than ti
        assert  t >= 0 and ti >= 0 and tf >= 0 and D >= 0
        assert tf >= ti

        if   t <= ti:
            pos = pi
            vel = 0

        elif ti < t <= tf:
            tau = ( t - ti ) / D    # Normalized time
            pos =    pi + ( pf - pi ) * ( 10 * tau ** 3 - 15 * tau ** 4 +  6 * tau ** 5 )
            vel = 1 / D * ( pf - pi ) * ( 30 * tau ** 2 - 60 * tau ** 3 + 30 * tau ** 4 )

        else:
            pos = pf
            vel = 0

        return pos, vel

    def add_movement( self , which_arm, pose1, pose2, duration, t_off ):
        """
            Adding the ZTT (Zero-torque-trajectory) information to the movement.
            ZTT uses the minimum-jerk-trajectory (quintic 5-th order polynomial) as the basis function.

            Arguments
            ---------
                [1] which_arm: either right (C.RIGHT) or left (C.LEFT)
                [2] pose1    : initial posture of the ZTT (Zero-torque-trajectory)
                [3] pose2    : final   posture of the ZTT (Zero-torque-trajectory)
                [4] duration : the    duration of the ZTT (Zero-torque-trajectory)
                [5] t_off    : time offset with respect to the previous movements, can be negative value since it can overlap with the previous one
                               If the movement is the first one (i.e., self.n_l_moves == 0 ), automatically set t_off as zero.

        """

        assert which_arm in [ C.RIGHT, C.LEFT ]
        assert duration >= 0

        # Defining the initial position, final position
        # When adding movement, if t_off is negative we need to take account for
        assert all( [ c in pose1.keys( ) for c in C.JOINT_NAMES ] )             # check whether the given dictionary has all the s0, s1, e0, e1, w0, w1 and w2 on the keys.
        assert all( [ c in pose2.keys( ) for c in C.JOINT_NAMES ] )             # check whether the given dictionary has all the s0, s1, e0, e1, w0, w1 and w2 on the keys.

        # Generating the pose and adding it to the "move" dictionary with key "pose"
        pi        = dict( )
        pf        = dict( )
        limb_name = self._type[  which_arm  ] # Either "right" or "left"
        sign      = self._signs[ which_arm  ] # refer to the variables declared in "__init__"

        for joint_name in C.JOINT_NAMES:
            pi[ limb_name + "_" + joint_name ] = pose1[ joint_name ] * sign[ joint_name ]
            pf[ limb_name + "_" + joint_name ] = pose2[ joint_name ] * sign[ joint_name ]


        move = dict( )

        move[ "pi" ] = pi
        move[ "pf" ] = pf
        move[ "D"  ] = duration

        if self.n_r_moves == 0:     # If this is the first movement
            move[ "t_off" ] = 0
            move[ "ti"    ] = 0
            move[ "tf"    ] = D

        else:
            idx_prev = self.n_r_moves - 1                                       # Index of the previous movement
            tf_prev  = self.moves[ which_arm ][ idx_prev ][ "tf" ]              # Getting the final time of the previous movement

            move[ "t_off" ] = t_off
            move[ "ti"    ] = tf_prev + t_off
            move[ "tf"    ] = tf_prev + t_off + D


        self.moves[   which_arm ].append( move )
        self.n_moves[ which_arm ] += 1

    def _prepare_movement( self ):
        # Prepare the movement to be conducted
        # It will be good to save the movement functions

        # Using the simple superposition principle of each movements
        # If ti and tf is defined, just define the function piecewise.

        # Prepare the movements for right movements
        for move in self.l_moves:
            # Calculate the maximum time size for the movement
            # Since there are 7 joints, we need to have 7 x N array for the calculation

        # Prepare the movements for left movements
        for move in self.r_moves:
            # Calculate the maximum time size for the movement
            # Since there are 7 joints, we need to have 7 x N array for the calculation


    def reset( self ):
        # Emptying out all the movements
        self.moves   = { C.RIGHT : [ ], C.LEFT : [ ] }
        self.n_moves = { C.RIGHT :   0, C.LEFT :   0 }

    def run( self ):

        assert self.n_r_moves >= 1 or self.n_l_moves >= 1


        self._prepare_movement( )

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot will timeout and disable
        # Regardless of which_arm, setting both the left and right arm set_command
        control_rate = rospy.Rate( self.rate )             # set control rate
        assert which_arm in [ C.RIGHT, C.LEFT, C.BOTH ]    # If which_arm is C.BOTH, then we are moving both the left and right arm

        self.robot.arms[ C.RIGHT ].set_command_timeout( ( 1.0 / self.rate ) * self.missed_cmds )
        self.robot.arms[ C.LEFT  ].set_command_timeout( ( 1.0 / self.rate ) * self.missed_cmds )



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

        self.type  = "joint_position_controller"

        self.moves   = []                                                       # Please check "add_movement" method to check the form of the movement.
        self.n_moves = 0


    def add_movement( self, which_arm, pose, joint_vel, t_off ):
        """
            The problem of this controller is that we cannot move both limbs simultaneously.
            Hence each right/left limb will move separately.

            Arguments
            ---------
                [1] which_arm : either right (C.RIGHT) or left (C.LEFT)
                [2] pose      : dictionary, s0, s1, e0, e1, w0, w1, w2 with corresponding values
                [3] joint_vel :
                [4] t_off     : time offset with respect to the previous movements, must be nonnegative
        """

        assert which_arm in [ C.RIGHT, C.LEFT ]
        assert all( [ c in pose.keys( ) for c in C.JOINT_NAMES ] )              # check whether the given dictionary has all the s0, s1, e0, e1, w0, w1 and w2 on the keys.
        assert joint_vel >= 0 and joint_vel <= 1
        assert     t_off >= 0


        # Generating the pose and adding it to the "move" dictionary with key "pose"
        new_pose  = dict( )
        limb_name = self._type[  which_arm  ] # Either "right" or "left"
        sign      = self._signs[ which_arm  ] # refer to the variables declared in "__init__"

        for joint_name in C.JOINT_NAMES:
            new_pose[ limb_name + "_" + joint_name ] = pose[ joint_name ] * sign[ joint_name ]        # Making the code economic using the kinematic symmetry of the robot

        # Now adding the details to the movement
        move = dict( )
        move[ "which_arm" ] = which_arm
        move[ "pose"      ] = new_pose
        move[ "joint_vel" ] = joint_vel
        move[ "t_off"     ] = t_off

        # Once done generating the movement dictionary, append it to internal dictionary and add the number of movements.
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
            j_vel     = move[ "joint_vel" ]
            t_off     = move[ "t_off"     ]

            self.robot.arms[ which_arm ].set_joint_position_speed( j_vel )
            self.robot.arms[ which_arm ].move_to_joint_positions( pose )

            rospy.sleep( t_off )


class Baxter( object ):

    # ================================================================ #
    # ======================== INIT  FUNCTIONS ======================= #
    # ================================================================ #

    def __init__( self, args ):

        self.args         = args
        self.publish_data = args.publish_data     # Boolean, data saved
                                                  # Just define the whole arms in the first place to simplify the code
        self.arms         = list( range( 2 ) )    # Since we have both the left/right arms
        self.kins         = list( range( 2 ) )    # Since we have both the left/right arms
        self.grips        = list( range( 2 ) )    # Since we have both the left/right arms

        self.start_time   = rospy.Time.now()

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
        self.grip_ctrls = [ GripperConnect( arm ) for arm in C.LIMB_NAMES ]

        # [BACKUP] When you want to control the grippers
        # Check whether the gripper is opened (100) or closed( 0 )
        # threshold value is simply 50

        self.open_gripper( )
        self.rs.enable()

        print( "[LOG] [INIT COMPLETE] Ctrl-c to quit" )

    def clean_shutdown( self ):

        print( "[LOG] [SHUTTING DOWN ROBOT]" )

        for i in range( 2 ):
            self.arms[ i ].exit_control_mode( )

        if not self.init_state and self.rs.state().enabled:

            self.msg.on = False
            self.pub.publish( self.msg )
            self.rs.disable()

    # ---------------------------------------------------------------- #
    # ------------------------ INIT  FUNCTIONS ----------------------- #
    # ---------------------------------------------------------------- #

    # ================================================================ #
    # ======================== BASIC FUNCTIONS ======================= #
    # ================================================================ #

    def get_arm_pose( self ):

        l_pose = self.arms[ C.LEFT  ].joint_angles( )
        r_pose = self.arms[ C.RIGHT ].joint_angles( )

        return l_pose, r_pose

    def get_gripper_pos( self ):

        l_grip = self.grips[ C.LEFT  ].position( )
        r_grip = self.grips[ C.RIGHT ].position( )

        return l_grip, r_grip

    def open_gripper( self  ):
        self.grips[ C.RIGHT ].open(  block = False )
        self.grips[ C.LEFT  ].open(  block = False )

    def close_gripper( self ):
        self.grips[ C.RIGHT ].close(  block = False )
        self.grips[ C.LEFT  ].close(  block = False )

    def tmp_pose_gen( self, mov_pars ):
        # mov_pars are in order, s1  e1 and w1

        new_pose = dict( )

        new_pose[ 's0' ] =  0.7869321442
        new_pose[ 'e0' ] = -0.0149563127
        new_pose[ 'w0' ] = -0.0464029188
        new_pose[ 'w2' ] = -1.5823011827

        new_pose[ 's1' ] = mov_pars[ 0 ]
        new_pose[ 'e1' ] = mov_pars[ 1 ]
        new_pose[ 'w1' ] = mov_pars[ 2 ]

        return new_pose


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

            q0[  joint_name ] = pose1[ joint_name ] + ( pose2[ joint_name ] - pose1[ joint_name ] ) * ( 10 * tt ** 3 - 15 * tt ** 4 +  6 * tt ** 5 )
            dq0[ joint_name ] =               1.0/D * ( pose2[ joint_name ] - pose1[ joint_name ] ) * ( 30 * tt ** 2 - 60 * tt ** 3 + 30 * tt ** 4 )

        return q0, dq0

    # ---------------------------------------------------------------- #
    # ------------------------ BASIC FUNCTIONS ----------------------- #
    # ---------------------------------------------------------------- #

    # ================================================================ #
    # ======================== MAIN  FUNCTIONS ======================= #
    # ================================================================ #


    def joint_impedance( self, which_arm, poses, Ds = None, toffs = None):

        control_rate = rospy.Rate( self.rate )             # set control rate
        assert which_arm in [ C.RIGHT, C.LEFT, C.BOTH ]    # If which_arm is C.BOTH, then we are moving both the left and right arm

        # Making it as a list
        Ds    = [ Ds    ] if isinstance(    Ds , float ) or isinstance(    Ds , int ) else    Ds
        toffs = [ toffs ] if isinstance( toffs , float ) or isinstance( toffs , int ) else toffs


        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot will timeout and disable
        # Regardless of which_arm, setting both the left and right arm set_command
        self.arms[ C.RIGHT ].set_command_timeout( ( 1.0 / self.rate) * self.missed_cmds )
        self.arms[ C.LEFT  ].set_command_timeout( ( 1.0 / self.rate) * self.missed_cmds )


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
                self.msg.stamp = ( rospy.Time.now() - self.start_time ).to_sec( )
                self.pub.publish( self.msg )
                control_rate.sleep()



def main():

    parser  = argparse.ArgumentParser( formatter_class = argparse.RawDescriptionHelpFormatter )
    parser.add_argument('-s', '--publish_data',
                        dest = 'publish_data',   action = 'store_true',
                        help = 'Save the Data')

    parser.add_argument('-g', '--open_gripper',
                        dest = 'open_gripper',    action = 'store_true',
                        help = 'Open the gripper for the initialization')

    parser.add_argument('-o', '--run_optimization',
                        dest = 'is_run_optimization',    action = 'store_true',
                        help = 'Running the optimization of the whole process')

    parser.add_argument('-c', '--controller',
                        dest = 'ctrl_type',    action = 'store', type = str,
                        help = 'Controller type, joint_position_ctrl, joint_imp_ctrl')


    args = parser.parse_args( rospy.myargv( )[ 1: ] )

    print( "Initializing node... " )
    rospy.init_node( "impedance_control_right" )
    my_baxter = Baxter( args )



    rospy.on_shutdown( my_baxter.clean_shutdown )


    # ==================================================================================================== #
    # Running the simulation
    # ==================================================================================================== #

    if args.is_run_optimization:

        my_log = Logger( record_data = True)


        # Find the input parameters (input_pars) that are aimed to be optimized
        # Possible options (written in integer values) are as follows
        # [REF] https://nlopt.readthedocs.io/en/latest/NLopt_Algorithms/
        idx       = 0
        #   idx are                 0                   1               2               3                  4
        idx_opt   = [ nlopt.GN_DIRECT_L, nlopt.GN_DIRECT_L_RAND, nlopt.GN_DIRECT, nlopt.GN_CRS2_LM, nlopt.GN_ESCH  ]


        # The upper and lower bound of the parameters of Baxter
        # D1, s1, e1, w1
        lb    = np.array( [ -0.7, 0.5, -1.3, 0.3, 0.3, 0.0 ] )
        ub    = np.array( [  0.2, 1.5,  1.3, 1.6, 1.6, 1.0 ] )
        n_opt = 6

        algorithm = idx_opt[ idx ]                                              # Selecting the algorithm to be executed
        opt       = nlopt.opt( algorithm, n_opt )                               # Defining the class for optimization

        opt.set_lower_bounds( lb )
        opt.set_upper_bounds( ub )
        opt.set_maxeval( 130 )

        init = ( lb + ub ) * 0.5 + 0.05 * lb                                    # Setting an arbitrary non-zero initial step

        my_baxter.move2pose( C.RIGHT, C.GRASP_POSE, wait_time = 2, joint_speed = 0.2 )
        my_baxter.move2pose( C.LEFT,  C.GRASP_POSE, wait_time = 2, joint_speed = 0.2 )
        # #
        my_baxter.control_gripper( mode = "timer" )

        tmp_t = 98 # If the coverage is higher than this value, simply set it as 100 and stop optimizaiton

        input( "Ready for optimization, press any key to continue" )


        def nlopt_objective( pars, grad ):                                      # Defining the objective function that we are aimed to optimize.

            # Run Baxter - Code implementation here
            # Manipulating the cloth
            # [STEP #1] Move to initial posture

            # [STEP #2] Initiate movement
            pose = my_baxter.tmp_pose_gen( pars[ 0:3 ] )

            my_log.write( "[Iteration] " + str( opt.get_numevals( ) ) + " [parameters] " + str( pars ) )

            my_baxter.joint_impedance(  C.BOTH, [ C.GRASP_POSE, pose , C.FINAL_POSE ] , Ds = [ pars[ 3 ], pars[ 4 ] ], toffs = [ pars[ 5 ], 5 ]  )


            # Get Baxter's tablecloth performance
            obj = rospy.get_param( 'my_obj_func' )

            if obj >= tmp_t:
                obj = 100.0

            my_log.write( " [obj] " + str( obj ) + "\n" )

            my_baxter.joint_impedance(  C.BOTH, [ C.FINAL_POSE, C.LIFT_POSE   ], Ds = 5, toffs = [ 1 ]  )
            my_baxter.joint_impedance(  C.BOTH, [ C.LIFT_POSE , C.GRASP_POSE  ], Ds = 5, toffs = [ 3 ]  )

            return 100.0 - obj # Inverting the value

        opt.set_min_objective( nlopt_objective )
        opt.set_stopval( -1    )                                                # If value is within 98~100% (i.e., 0~2%)
        xopt = opt.optimize( init )                                             # Start at the mid-point of the lower and upper bound

    # ============================================================================= #

    elif not args.is_run_optimization:

        # Can get the value via rosparam

        # ==================================================================================================== #
        # [Step #1] Setting the gripper
        # ==================================================================================================== #

        # Adding baxter to the controller that we are using
        my_ctrl = JointPositionController( my_baxter )

        # Adding the details of the movements to be sequentially conducted
        my_ctrl.add_movement( C.RIGHT, C.GRASP_POSE, 0.2, 5 )
        my_ctrl.add_movement( C.RIGHT, C.REST_POSE, 0.2, 5 )
        my_ctrl.add_movement( C.LEFT , C.GRASP_POSE, 0.2, 5 )
        my_ctrl.add_movement( C.RIGHT , C.FINAL_POSE, 0.2, 5 )

        # Initiating the movement
        my_ctrl.run( )

        exit( )

        # In case for an impedance controller
        my_ctrl = JointImpedanceController( my_baxter, C.JOINT_IMP_Kq, C.JOINT_IMP_Bq )

        exit( )

        my_baxter.close_gripper( )


        opt_pars = [-0.55, 1., -1.15555556, 0.9, 0.9, 0.5]
        pose = my_baxter.tmp_pose_gen( opt_pars[ 0:3 ] )




        input( "Ready for optimization, press any key to continue" )


        my_baxter.joint_impedance(  C.BOTH, [ C.GRASP_POSE, pose , C.FINAL_POSE ] , Ds = [ opt_pars[ 3 ], opt_pars[ 4 ] ], toffs = [ opt_pars[ 5 ], 8 ]  )
        # my_baxter.control_gripper( mode = "timer" )

        # ==================================================================================================== #
        # [Step #2] Initiate the movement
        # ==================================================================================================== #

        rospy.sleep( 10 )

        # my_baxter.joint_impedance(  C.BOTH, [ C.GRASP_POSE, C.MID_POSE, C.FINAL_POSE  ] , Ds = [1.0, 1.0], toffs = [0.1, 2.0]  )

        # my_baxter.joint_impedance(  C.BOTH, [ C.GRASP_POSE, C.LIFT_POSE   ], Ds = 5, toffs = [1]  )
        # my_baxter.joint_impedance(  C.BOTH, [ C.LIFT_POSE , C.GRASP_POSE  ], Ds = 5, toffs = [1]  )



        # my_baxter.move2pose( C.LEFT , C.GRASP_POSE, wait_time = 2, joint_speed = 0.2 )
        # my_baxter.joint_impedance(  C.BOTH, [ C.GRASP_POSE, C.MID_POSE, C.FINAL_POSE  ] , Ds = [1.0, 1.0], toffs = [0.1, 2.0]  )
        # my_baxter.joint_impedance(  C.BOTH, [ C.GRASP_POSE, C.MID_POSE  ] , Ds = [1.0], toffs = [2.0]  )




if __name__ == "__main__":
    main()
