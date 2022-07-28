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
from my_utils     import min_jerk_traj, pose_right2left, dict2arr, arr2dict, poses_delta, GripperConnect, Logger

# Local Library, customized messages
from moses_baxter.msg import my_msg

np.set_printoptions( linewidth = 4000, precision = 8)


# =================================================== #
# ==================== Controllers  ================= #
# =================================================== #


class PrintJointController:
    """
        Printing out the joint values by moving around the limbs.

        This controller is useful for debugging.
    """
    def __init__( self, robot ):

        self.robot = robot
        self.type  = "print_joint_controller"

    def run( self ):

        DONE = False
        while not DONE:
            typed_letter = baxter_external_devices.getch( )

            if typed_letter:

                # Catch Esc or ctrl-c
                if typed_letter in ['\x1b', '\x03']:   
                    DONE = True
                    rospy.signal_shutdown( "[LOG] EXITTING PRINT JOINT MODE.")

                if typed_letter == "p":

                    print( "=" * 100 )
                    for limb_name in [ "right", "left" ]:
                        joint_angles = self.robot.get_arm_pose( limb_name )

                        for joint_name in C.JOINT_NAMES[ limb_name ]:
                            print( "'{0}' : {1:.10f},".format( joint_name, joint_angles[ joint_name ] ) )
                    print( "=" * 100 )


class JointPositionController:
    """
        Pure position controller
        Using innate Baxter function
            [1] set_joint_position_speed( joint_speed )
            [2] move_to_joint_positions( pose )
        to conduct the movements

    """

    def __init__( self, robot ):

        self.robot = robot

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
                    
                    
# =================================================== #
# =========== Impedance Controllers  ================ #
# =================================================== #                    

class ImpedanceController( object ):
    """
        Primitive Impedance Controller Object
    """

    def __init__( self, robot, which_arm  ):

        self.robot = robot
        
        assert which_arm in [ "right", "left" ]
        
        self.which_arm      = which_arm 
        self.ctrl_par_names = None
        
    def set_ctrl_par( self, **kwargs ):
        """
            Setting the control parameters
            Each controllers have their own controller parameters names (self.ctrl_par_names),
            This method function will become handy when we want to modify, or set the control parameters.
        """
        # Ignore if the key is not in the self.ctrl_par_names 
        # Setting the attribute of the controller 
        assert self.ctrl_par_names is not None
        
        [ setattr( self, key, val ) for key, val in kwargs.items( ) if key in self.ctrl_par_names ]

    def calc_torque( self, t: float ):
        """
            Return the torque value at given time t 
            
        """
        NotImplementedError( )
                            

class JointImpedanceController( ImpedanceController ):
    """
        1st-order joint-space impedance controller
        The equation is as follows:
            tau = Kq( q0 - q ) + Bq( dq0 - dq ) + tau_G

        tau_G, the gravity compensation torque is conducted by Baxter alone
    """

    def __init__( self, robot, which_arm : str ):

        super().__init__( robot, which_arm )
        self.type           = "joint_impedance_controller"
        self.ctrl_par_names = [ "Kq", "Bq", "qi", "qf", "D", "ti" ]
        
        # Initialize all the ctrl_par_names to None
        [ setattr( self, key, None ) for key in self.ctrl_par_names ]
        
    def setup( self ):
        """
            Setting up the details of the movement. 
            
            This method should be called AFTER calling "set_ctrl_par" method.
        """
        assert all( [ getattr( self, c ) is not None for c in self.ctrl_par_names ]  )

        # Check whether the stiffness and damping matrices are in good shape.
        assert len( self.Kq ) == 7 and len( self.Kq[ 0 ] ) == 7 
        assert len( self.Bq ) == 7 and len( self.Bq[ 0 ] ) == 7 

        # Check whether both matrices are positive definite
        assert np.all( np.linalg.eigvals( self.Kq ) > 0 )
        assert np.all( np.linalg.eigvals( self.Bq ) > 0 )

        # Check whether the size of qi and qf are good
        assert len( self.qi ) == 7
        assert len( self.qf ) == 7

        # Check whether the D and ti are positive and non-negative, respectively. 
        assert self.D > 0 and self.ti >= 0 

    def calc_torque( self, t: float ):
        
        q  = self.robot.get_arm_pose(     self.which_arm )
        dq = self.robot.get_arm_velocity( self.which_arm )
        
        # Post Processing 
        q  = dict2arr( self.which_arm, q  )
        dq = dict2arr( self.which_arm, dq ) 
        
        q0  = np.zeros( 7 )
        dq0 = np.zeros( 7 )
        
        for i in range( 7 ): 
            pos, vel = min_jerk_traj( t, self.ti, self.ti + self.D, self.pi[ i ], self.pf[ i ], self.D )
            q0[ i ]  = pos
            dq0[ i ] = vel
            
        tau = self.Kq @ ( q0 - q ) + self.Bq @ ( dq0 - dq )

        return arr2dict( self.which_arm, tau )


class CartesianImpedanceController( ImpedanceController ):
    
    def __init__( self, robot ):
        super().__init__( robot )

        self.type    = "cartesian_impedance_controller"
        self.moves   = []                  
        self.n_moves = 0
        
    def add_movement( self, which_arm: str, pose2go: dict, joint_vel: float, toff: float ):
        NotImplementedError( )
        
    def reset( self ):
        self.moves   = []
        self.n_moves = 0 

        
        
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
    parser.add_argument('-s', '--save_data'        , dest = 'is_save_data'        , action = 'store_true' ,              help = 'Save the Data'                                   )
    parser.add_argument('-o', '--run_optimization' , dest = 'is_run_optimization' , action = 'store_true' ,              help = 'Running the optimization of the whole process'   )
    parser.add_argument('-c', '--controller'       , dest = 'ctrl_type'           , action = 'store'      , type =  str, help = C.CONTROLLER_DESCRIPTIONS                         )
    args = parser.parse_args( rospy.myargv( )[ 1: ] )


    print( "Initializing node... " )
    rospy.init_node( "impedance_control" )
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
        idx       = 0


        #  Upper/Lower Bound, ordered as:
        #  POSE_MID_s1    POSE_MID_e1     POSE_MID_w1,  POSE_FINAL_s1,  POSE_FINAL_e1, POSE_FINAL_w1   D1   D2    a
        # lb    = np.array( [ -0.65, 0.31, -0.76, -0.65, 0.31, -0.76, 0.6, 0.6, -0.6 ] )
        # ub    = np.array( [  0.00, 0.83, -0.25,  0.00, 0.83, -0.25, 1.5, 1.5,  0.5 ] )
        # n_opt = 9

        lb    = np.array( [ -0.65, 0.31, -0.76, 0.6, 0.6, -0.6 ] )
        ub    = np.array( [  0.00, 0.83, -0.25, 1.5, 1.5,  0.5 ] )
        n_opt = 6

        algorithm = idx_opt[ idx ]                                              # Selecting the algorithm to be executed
        opt       = nlopt.opt( algorithm, n_opt )                               # Defining the class for optimization

        opt.set_lower_bounds( lb )
        opt.set_upper_bounds( ub )
        opt.set_maxeval( 100 )

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
            tmp_arr = np.zeros( 5 )

            for i in range( 5 ):

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

                # POSE3_R = my_ctrl.gen_dict( "right", np.array( [ C.GRASP_POSE[ "right_s0" ],
                #                                                                   pars[ 3 ],
                #                                                  C.GRASP_POSE[ "right_e0" ],
                #                                                                   pars[ 4 ],
                #                                                  C.GRASP_POSE[ "right_w0" ],
                #                                                                   pars[ 5 ],
                #                                                  C.GRASP_POSE[ "right_w2" ] ] ) )
                # POSE3_L = pose_right2left( POSE3_R  )

                POSE3_R = C.FINAL_POSE
                POSE3_L = pose_right2left( C.FINAL_POSE  )


                my_ctrl.add_movement( which_arm = "right", pose_init = POSE1_R, pose_final = POSE2_R, duration = pars[ 3 ], toff = 0.0                   )
                my_ctrl.add_movement( which_arm = "right", pose_init = POSE2_R, pose_final = POSE3_R, duration = pars[ 4 ], toff = pars[ 3 ] * pars[ 5 ] )

                my_ctrl.add_movement( which_arm = "left" , pose_init = POSE1_L, pose_final = POSE2_L, duration = pars[ 3 ], toff = 0.0                   )
                my_ctrl.add_movement( which_arm = "left" , pose_init = POSE2_L, pose_final = POSE3_L, duration = pars[ 4 ], toff = pars[ 3 ] * pars[ 5 ] )


                my_ctrl.run( )
                rospy.sleep( 3 )

                # Get Baxter's tablecloth performance
                obj = rospy.get_param( 'my_obj_func' )
                if obj >= tmp_t:
                    obj = 100.0

                tmp_arr[ i ] = obj
                my_ctrl.reset( )

            good = np.max( tmp_arr )
            my_log.write( "[Iteration] " + str( opt.get_numevals( ) + 1) + " [parameters] " + np.array2string( np.array( pars ).flatten(), separator = ',' ) )
            my_log.write( " [all_vals] " + np.array2string( np.array( tmp_arr ).flatten(), separator = ',' ) )
            my_log.write( " [obj] " + str( good ) + "\n" )

            return 100.0 - good # Inverting the value


        # input( "Ready for optimization, press any key to continue" )

        opt.set_min_objective( nlopt_objective )
        opt.set_stopval( -1    )                                                # If value is within 98~100% (i.e., 0~2%)
        xopt = opt.optimize( init )                                             # Start at the mid-point of the lower and upper bound

        my_log.log.close()
        
        
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
        # ================== PRINT JOINT CONTROLLER ===================== #
        # =============================================================== #

        elif args.ctrl_type == "print_joint_controller":
            my_ctrl = PrintJointController( my_baxter )
            my_ctrl.run( )


        # =============================================================== #
        # ================= JOINT IMPEDANCE CONTROLLER ================== #
        # =============================================================== #

        elif args.ctrl_type == "joint_impedance_controller":
            
            Kq_mat = np.diag( [ 15.0, 15.0, 8.0, 10.0, 3.0, 10.0, 1.5 ]  )
            Bq_mat = 0.2 * Kq_mat

            # First submovement
            imp1 = JointImpedanceController( my_baxter, "right" )
            qi = dict2arr( "right", C.GRASP_POSE )
            qf = dict2arr( "right", C.MID_POSE   )
            imp1.set_ctrl_par( Kq = Kq_mat, Bq = Bq_mat, qi = qi, qf = qf, ti = 0,   )
            imp1.setup( )
            
            imp2 = JointImpedanceController( my_baxter, "right" )
            imp2.setup( )
            
            

        # =============================================================== #
        # ============= CARTESIAN IMPEDANCE CONTROLLER ================== #
        # =============================================================== #


        # =============================================================== #
        # ==================== IMPEDANCE SUPERPOSITION ================== #
        # =============================================================== #

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


            rospy.sleep( 5 )
            my_baxter.close_gripper()
            input( "Ready for optimization, press any key to continue" )
            # my_ctrl.move2pose( C.LIFT_POSE , duration = 5, toff = 1 )

            # POSE_MID_s1    POSE_MID_e1     POSE_MID_w1,  POSE_FINAL_s1,  POSE_FINAL_e1, POSE_FINAL_w1   D1   D2    a, toff = D1 * a
            # pars = [-0.61388889, 0.57       ,-0.675     , -0.25277778, 0.57       , -0.505      , 1.05       , 1.45       , 0.19444444] # DIRECT
            pars = [-0.64213268, 0.33356569 ,-0.46185895, -0.38859129, 0.48006301 , -0.74097092 , 1.34714252 , 0.72278278 , 0.2171941 ] # CRS

            
            POSE1_R = C.GRASP_POSE
            POSE1_L = pose_right2left( C.GRASP_POSE  )
            #
            # # s0, s1, e0, e1, w0, w1, w2 number
            POSE2_R = my_ctrl.gen_dict( "right", np.array( [ C.GRASP_POSE[ "right_s0" ],
                                                                              pars[ 0 ],
                                                             C.GRASP_POSE[ "right_e0" ],
                                                                              pars[ 1 ],
                                                             C.GRASP_POSE[ "right_w0" ],
                                                                              pars[ 2 ],
                                                             C.GRASP_POSE[ "right_w2" ] ] ) )
            POSE2_L = pose_right2left( POSE2_R  )

            # POSE3_R = my_ctrl.gen_dict( "right", np.array( [ C.GRASP_POSE[ "right_s0" ],
            #                                                                   pars[ 3 ],
            #                                                  C.GRASP_POSE[ "right_e0" ],
            #                                                                   pars[ 4 ],
            #                                                  C.GRASP_POSE[ "right_w0" ],
            #                                                                   pars[ 5 ],
            #                                                  C.GRASP_POSE[ "right_w2" ] ] ) )
            # POSE3_L = pose_right2left( POSE3_R  )

            POSE3_R = C.FINAL_POSE
            POSE3_L = pose_right2left( C.FINAL_POSE  )
            #
            #
            my_ctrl.add_movement( which_arm = "right", pose_init = POSE1_R, pose_final = POSE2_R, duration = pars[ 6 ], toff = 0.0                   )
            my_ctrl.add_movement( which_arm = "right", pose_init = POSE2_R, pose_final = POSE3_R, duration = pars[ 7 ], toff = pars[ 6 ] * pars[ 8 ] )

            my_ctrl.add_movement( which_arm = "left" , pose_init = POSE1_L, pose_final = POSE2_L, duration = pars[ 6 ], toff = 0.0                   )
            my_ctrl.add_movement( which_arm = "left" , pose_init = POSE2_L, pose_final = POSE3_L, duration = pars[ 7 ], toff = pars[ 6 ] * pars[ 8 ] )

            my_ctrl.run( )
            #
            rospy.sleep( 3 )
            # # Get Baxter's tablecloth performance
            # obj = rospy.get_param( 'my_obj_func' )
            #
            # my_log.write( "[obj] " + str( obj ) + "\n" )
            #
            # my_ctrl.reset( )


            # # Design the movements in detail
            # POSE1_R = C.GRASP_POSE
            # POSE1_L = pose_right2left( C.GRASP_POSE  )
            #
            # POSE2_R = C.MID_POSE
            # POSE2_L = pose_right2left( C.MID_POSE    )
            #
            # POSE3_R = C.FINAL_POSE
            # POSE3_L = pose_right2left( C.FINAL_POSE  )
            #
            # my_ctrl.add_movement( which_arm = "right", pose_init = POSE1_R, pose_final = POSE2_R, duration = 1, toff =  0.0 )
            # my_ctrl.add_movement( which_arm = "left" , pose_init = POSE1_L, pose_final = POSE2_L, duration = 1, toff =  0.0 )
            #
            # my_ctrl.add_movement( which_arm = "right", pose_init = POSE2_R, pose_final = POSE3_R, duration = 1, toff = 0.4 )
            # my_ctrl.add_movement( which_arm = "left" , pose_init = POSE2_L, pose_final = POSE3_L, duration = 1, toff = 0.4 )
            # my_ctrl.add_movement( which_arm = "right", pose_init = POSE2_R, pose_final = POSE3_R, duration = 1, toff = 0.87 * 0.45 )
            # my_ctrl.add_movement( which_arm = "left" , pose_init = POSE2_L, pose_final = POSE3_L, duration = 1, toff = 0.87 * 0.45 )
            #

            # my_log.log.close()


        # =============================================================== #
        # ================== TASK IMPEDANCE CONTROLLER ================== #
        # =============================================================== #
        elif args.ctrl_type == "cartesian_impedance_controller":
            my_ctrl = CartesianImpedanceController( my_baxter )

if __name__ == "__main__":
    main()
