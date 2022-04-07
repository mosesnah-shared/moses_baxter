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

    def gen_dict( self, which_arm: str, arr: np.ndarray ):

        assert which_arm in [ "right", "left" ]
        assert len( arr ) == 7

        new = dict( )

        for i, joint_name in C.JOINT_NAMES[ which_arm ]:
            new[ joint_name ] = arr[ i ]

        return new

    def poses_delta( self, which_arm: str, pose1: dict, pose2: dict ):
        """
            conduct pose2 - pose1 for each element
        """
        assert all( [ c in pose1.keys( ) for c in C.JOINT_NAMES[ which_arm ] ] )
        assert all( [ c in pose2.keys( ) for c in C.JOINT_NAMES[ which_arm ] ] )

        return { key: pose2[ key ] - pose1[ key ] for key in pose1.keys( ) }


    def pose_right2left( self, pose: dict ):
        """
            Changing the dictionary key's prefix name from "right_" to "left_", and flipping the sign too
            The reason why we don't have left2right is because we will follow the "right-hand" convention

            Arguments:
                [1] pose ( dict ): 7-element dictionary with keys "right_" + s0, s1, e0, e1, w0, w1, w2
        """
        assert all( [ c in pose.keys( ) for c in C.JOINT_NAMES[ "right" ] ] )   # check whether the given dictionary has all the "right_" + s0, s1, e0, e1, w0, w1 and w2 on the keys.

        new_pose = dict( )

        for right_name in C.JOINT_NAMES[ "right" ]:
            left_name = C.RIGHT2LEFT[ right_name ]
            new_pose[ left_name ] = C.LEFT_JOINT_SIGN[ left_name ] * pose[ right_name ]

        return new_pose

class JointImpedanceController( Controller ):
    """
        1st-order joint-space impedance controller
        The equation is as follows:
            tau = Kq( q0 - q ) + Bq( dq0 - dq ) + tau_G

        tau_G, the gravity compensation torque is conducted by Baxter alone
    """

    def __init__( self, robot, Kq : dict, Bq : dict ):


        self.type  = "joint_impedance_controller"
        self.Kq    = Kq
        self.Bq    = Bq

        # Since the left and right moves can be conducted independently, also saving it independently
        self.moves   = { "right" : [ ], "left" : [ ] }
        self.n_moves = { "right" :   0, "left" :   0 }

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
                [2] pose_init  (dict)  : initial "right" posture of the ZTT (Zero-torque-trajectory)
                [3] pose_final (dict)  : final   "right" posture of the ZTT (Zero-torque-trajectory)
                [4] duration   (float) : the    duration of the ZTT (Zero-torque-trajectory)
                [5] toff       (float) : time offset with respect to the previous movements, can be negative value since it can overlap with the previous one
                                         If the movement is the first one (i.e., self.n_l_moves == 0 ), automatically set toff as zero.

        """

        assert which_arm in [ "right", "left" ]
        assert duration >= 0
        assert all( [ c in pose_init.keys(  ) for c in C.JOINT_NAMES[ "right" ] )
        assert all( [ c in pose_final.keys( ) for c in C.JOINT_NAMES[ "right" ] )

        # Details of movement to add
        move = dict( )

        if self.n_moves[ which_arm ] == 0:                                      # For the 1st position

            pi   = self.pose_right2left( pose_init  ) if which_arm == "left" else pose_init
            pf   = self.pose_right2left( pose_final ) if which_arm == "left" else pose_final
            toff = 0
            ti   = 0
            tf   = duration
            D    = duration

        else:

            idx_prev = self.n_moves[ which_arm ] - 1                            # Index of the previous movement
            tf_prev  = self.moves[ which_arm ][ idx_prev ][ "tf" ]              # Getting the final time of the previous movement

            tmpi = self.pose_right2left( pose_init  ) if which_arm == "left" else pose_init
            tmpf = self.pose_right2left( pose_final ) if which_arm == "left" else pose_final

            pi   = self.gen_dict( which_arm = "left", arr = np.zeros( 7 ) )     # Zero initial posture
            pf   = self.delta_poses( tmpi, tmpf )                               # Doing tmpf - tmpi
            toff = toff
            ti   = tf_prev + toff      if tf_prev + toff     >= 0 else 0
            tf   = tf_prev + toff + D  if tf_prev + toff + D >= 0 else duration
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

        assert self.n_moves[ C.RIGHT ] >= 1 or self.n_moves[ C.LEFT ] >= 1

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot will timeout and disable
        # Regardless of which_arm, setting both the left and right arm set_command
        control_rate = rospy.Rate( self.rate )             # set control rate

        self.robot.arms[ C.RIGHT ].set_command_timeout( ( 1.0 / self.rate ) * self.missed_cmds )
        self.robot.arms[ C.LEFT  ].set_command_timeout( ( 1.0 / self.rate ) * self.missed_cmds )


        # The initial time for the simulation
        ts = rospy.Time.now( )

        while not rospy.is_shutdown( ):

            if not self.robot.rs.state( ).enabled:
                rospy.logerr( "Impedance example failed to meet, specified control rate timeout." )
                break

            # The elapsed time
            t = ( rospy.Time.now( ) - ts ).to_sec( )

            q0_R, dq0_R = self.get_reference_traj( which_arm = "right", t )
            q0_L, dq0_L = self.get_reference_traj( which_arm = "left" , t )

            q0  = { "right": q0_R , "left": q0_L  }
            dq0 = { "right", dq0_R, "left": dq0_L }
            tau = { "right": self.gen_dict( which_arm = "right", np.zeros( 7 ) ),
                     "left": self.gen_dict( which_arm =  "left", np.zeros( 7 ) }

            for which_arm in [ "right", "left" ]:

                # Need to check whether the movement exists
                if self.n_moves[ which_arm ] == 0:
                    continue

                for joint_name in C.JOINT_NAMES[ which_arm ]:
                    tau[ which_arm ][ joint_name ]  =  self.Kq[ which_arm ][ joint_name ] * (  q0[ which_arm ][ joint_name ] -  q[ which_arm ][ joint_name )
                    tau[ which_arm ][ joint_name ] +=  self.Bq[ which_arm ][ joint_name ] * ( dq0[ which_arm ][ joint_name ] - dq[ which_arm ][ joint_name )

                if self.publish_data:
                    self.msg.q0_L[ j ]  =  q0_L[  left_name ]
                    self.msg.q_L[ j ]   =   q_L[  left_name ]
                    self.msg.dq_L[ j ]  =  dq_L[  left_name ]
                    self.msg.tau_L[ j ] = tau_L[  left_name ]

                    self.msg.q0_R[ j ]  =  q0_R[ right_name ]
                    self.msg.q_R[ j ]   =   q_R[ right_name ]
                    self.msg.dq_R[ j ]  =  dq_R[ right_name ]
                    self.msg.tau_R[ j ] = tau_R[ right_name ]

                self.robot.arms[ "right" ].set_joint_torques( tau[ "right" ] )
                self.robot.arms[ "left"  ].set_joint_torques( tau[ "left"  ] )


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
                [2] pose2go   (dict)  : Keys "right_" + s0, s1, e0, e1, w0, w1, w2 with corresponding values.
                                        In case if which_arm is left, should change prefix "right" to "left"
                [3] joint_vel (float) : Ranged (0,1) and argument for "set_joint_position_speed" method
                [4] toff      (float) : time offset with respect to the previous movements, must be nonnegative
        """

        assert which_arm in [ "right", "left" ]
        assert all( [ c in pose2go.keys( ) for c in C.JOINT_NAMES[ "right" ] ] )   # check whether the given dictionary has all the "right_" + s0, s1, e0, e1, w0, w1 and w2 on the keys.
        assert joint_vel >= 0 and joint_vel <= 1
        assert      toff >= 0

        # If which_arm is LEFT, change the pose2go to "left" pose (mirror image)
        pose = self.pose_right2left( pose2go ) if which_arm == "left" else pose2go

        # Generating the movement details
        move = dict( )
        move[ "which_arm" ] = which_arm
        move[ "pose"      ] = pose
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


class Baxter( object ):

    # ================================================================ #
    # ======================== INIT  FUNCTIONS ======================= #
    # ================================================================ #

    def __init__( self, args ):

        self.args         = args
        self.publish_data = args.publish_data     # Boolean, data saved
                                                  # Just define the whole arms in the first place to simplify the code
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

        self.open_gripper( )
        self.rs.enable()

        print( "[LOG] [INIT COMPLETE] Ctrl-c to quit" )

    def clean_shutdown( self ):

        print( "[LOG] [SHUTTING DOWN ROBOT]" )

        for limb_name in [ "left", "right" ]:
            self.arms[ limb_name ].exit_control_mode( )

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
        my_ctrl.add_movement( which_arm = "right" , pose2go = C.GRASP_POSE, joint_vel = 0.2, toff = 5 )
        my_ctrl.add_movement( which_arm = "left"  , pose2go = C.GRASP_POSE, joint_vel = 0.2, toff = 5 )
        my_ctrl.add_movement( which_arm = "left"  , pose2go = C.LIFT_POSE , joint_vel = 0.2, toff = 5 )
        my_ctrl.add_movement( which_arm = "right" , pose2go = C.LIFT_POSE , joint_vel = 0.2, toff = 5 )
        my_ctrl.add_movement( which_arm = "right" , pose2go = C.FINAL_POSE, joint_vel = 0.2, toff = 5 )
        my_ctrl.add_movement( which_arm = "left"  , pose2go = C.FINAL_POSE , joint_vel = 0.2, toff = 5 )

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
