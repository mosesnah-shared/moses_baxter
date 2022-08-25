#!/usr/bin/python3

import rospy
import scipy.io
import argparse
import numpy             as np
import matplotlib.pyplot as plt

# Local Library, under moses/scripts
from my_robot     import Baxter
from my_constants import Constants as C
from my_utils     import min_jerk_traj, pose_right2left, dict2arr, quat2rot, rot2quat, quat2angx, skew_sym

# Baxter Library
import baxter_external_devices  

# Local Message Defined            
from moses_baxter.msg import my_msg


np.set_printoptions( linewidth = 4000, precision = 8)

class PrintController:
    """
        Printing out the joint values by moving around the limbs.

        This controller is useful for debugging.
    """
    def __init__( self, robot ):

        self.robot = robot
        self.type  = "print_controller"

    def run( self ):

        DONE = False
        while not DONE:
            typed_letter = baxter_external_devices.getch( )

            if typed_letter:
                            
                # Catch Esc or ctrl-c
                if typed_letter in ['\x1b', '\x03']:    
                    DONE = True
                    rospy.signal_shutdown( "[LOG] EXITTING PRINT JOINT MODE.")

                elif typed_letter == "p":

                    print( "=" * 100 )
                    for limb_name in [ "right", "left" ]:
                        joint_angles = self.robot.get_arm_pose( limb_name )

                        for joint_name in C.JOINT_NAMES[ limb_name ]:
                            print( "'{0}' : {1:.10f},".format( joint_name, joint_angles[ joint_name ] ) )
                    print( "=" * 100 )

                elif typed_letter == "x":

                    print( "=" * 100 )

                    for limb_name in [ "right", "left" ]:
                        pos = self.robot.get_end_effector_pos( limb_name )
                        print( pos )
                    

                    print( "=" * 100 )

                elif typed_letter == "r":

                    print( "=" * 100 )

                    for limb_name in [ "right", "left" ]:
                        pos = self.robot.get_end_effector_orientation( limb_name )
                        print( pos )
                    

                    print( "=" * 100 )



# ==== TODO ==== #
# [2022.07.29] Need to change the code, although the functionality is not used often.
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
        self.moves   = [ ]                               # Elements should be dictionary Type
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

    def __init__( self, robot, which_arm : str, name : str, is_save_data : bool ):

        super().__init__( robot, which_arm )
        self.type           = "joint_impedance_controller"
        self.name           = name + "_" + self.type
        self.is_save_data   = is_save_data
        
        # The number of submovements for the ZFT
        self.n_act = 0 
        
        # The number samples for the data
        self.Ns = 2 ** 15

        # Controller Parameters of the Controller 
        # Impedances
        self.Kq = None
        self.Bq = None
        
        # The movement parameters, we save this as an array since multiple submovements may exist
        self.qi = [ ]
        self.qf = [ ] 
        self.D  = [ ]
        self.ti = [ ]

    def set_impedance( self, Kq:np.ndarray, Bq:np.ndarray ):
        
        # Resetting the Kq and Bq will be dangerous, hence asserting. 
        assert self.Kq is None and self.Bq is None
        
        # Check whether the stiffness and damping matrices are in good shape.
        assert len( Kq ) == 7 and len( Kq[ 0 ] ) == 7 
        assert len( Bq ) == 7 and len( Bq[ 0 ] ) == 7         

        # Check whether both matrices are positive definite
        assert np.all( np.linalg.eigvals( Kq ) > 0 )
        assert np.all( np.linalg.eigvals( Bq ) > 0 )
            
        # If they have passed all the asserts, then saving it as an attribute
        self.Kq = Kq 
        self.Bq = Bq 
        
        
    def add_movement( self, qi:np.ndarray, qf:np.ndarray, D:float, ti:float ):
        
        # Check whether the size of qi and qf are good
        assert len( qi ) == 7
        assert len( qf ) == 7

        # Check whether the D and ti are positive and non-negative, respectively. 
        assert D > 0 and ti >= 0 
        
        # If there is more than one movement, then qi must be a zero array. 
        if self.n_act >= 1: assert np.all( ( qi == 0 ) )
        
        self.qi.append( qi )
        self.qf.append( qf )
        self.D.append(  D  )
        self.ti.append( ti )
        
        # Since we have added the new movement, add the number of movements 
        self.n_act += 1 
                
                
    def setup( self ):
        """
            Setting up the details of the movement. 
            
            This method should be called AFTER calling "set_ctrl_par" method.
        """

        # Before Setting, check whether the impedances are defined
        assert self.Kq is not None and self.Bq is not None 
        
        # Check whether the movement is defined 
        assert self.n_act >= 1 

        # The data array for saving the data 
        if self.is_save_data:
            self.t_arr   = np.zeros( self.Ns )
            self.q_arr   = np.zeros( ( 7, self.Ns ) )
            self.dq_arr  = np.zeros( ( 7, self.Ns ) )
            self.q0_arr  = np.zeros( ( 7, self.Ns ) )
            self.dq0_arr = np.zeros( ( 7, self.Ns ) )
            self.tau_arr = np.zeros( ( 7, self.Ns ) )
        
            # The data pointer that we will use for saving the data 
            self.idx_data = 0 

            
    def calc_torque( self, t: float ):
        """
            Calculate and return tau, which is:
            
            tau = Kq ( q0 - q ) + Bq ( dq0 - dq )  
            
        """
        
        # Calling the dictionary values, the keys will be "WHICH_ARM" + s0, s1, e0, e1, w0, w1, w2
        q  = self.robot.get_arm_pose(     self.which_arm )
        dq = self.robot.get_arm_velocity( self.which_arm )
        
        # Changing the dictionary to 1x7 numpy array
        q  = dict2arr( self.which_arm, q  )
        dq = dict2arr( self.which_arm, dq ) 
        
        q0  = np.zeros( 7 )
        dq0 = np.zeros( 7 )
        
        # Calculating the ZFT trajectory
        for i in range( self.n_act ): 
            for j in range( 7 ): 
                ZFT_pos, ZFT_vel = min_jerk_traj( t, self.ti[ i ], self.ti[ i ] + self.D[ i ], self.qi[ i ][ j ], self.qf[ i ][ j ], self.D[ i ] )
                q0[  j ] += ZFT_pos
                dq0[ j ] += ZFT_vel
            
        # Calculate the torque which should be inputed. 
        tau = self.Kq @ ( q0 - q ) + self.Bq @ ( dq0 - dq )
        
        if self.is_save_data:
            self.t_arr[      self.idx_data ] = t
            self.q_arr[   :, self.idx_data ] = q
            self.dq_arr[  :, self.idx_data ] = dq
            self.q0_arr[  :, self.idx_data ] = q0
            self.dq0_arr[ :, self.idx_data ] = dq0
            self.tau_arr[ :, self.idx_data ] = tau
            
            self.idx_data += 1

        return tau
    
    def publish_data( self, dir_name: str ):
        # Printing out all the details of the simulation as a file that is great for MATLAB compatability
        file_name = dir_name + "/" + self.name + ".mat"
        
        # [TODO] There will be a single liner to do this, including this in the parent class
        scipy.io.savemat( file_name, { 'name': self.name, 
                                       'time': self.t_arr[      :self.idx_data ], 
                                         'qi': self.qi, 
                                         'qf': self.qf,
                                          'D': self.D, 
                                         'ti': self.ti, 
                                          'q': self.q_arr[   :, :self.idx_data ], 
                                         'dq': self.dq_arr[  :, :self.idx_data ], 
                                         'q0': self.q0_arr[  :, :self.idx_data ], 
                                        'dq0': self.dq0_arr[ :, :self.idx_data ], 
                                        'tau': self.tau_arr[ :, :self.idx_data ],
                                         'Kq': self.Kq, 'Bq': self.Bq }  )
        
        
        
class CartesianImpedanceControllerPos( ImpedanceController ):
    
    def __init__( self, robot, which_arm : str, name: str, is_save_data:bool = False ):
        
        super( ).__init__( robot, which_arm )
        self.type = "cartesian_impedance_controller"
        self.name = name + "_" + self.type

        # Check whether the input is position only or position and rotation 
        
        # If position only        , set the size of the matrices as 3
        # If position and rotation, set the size of the matrices as 6
        self.n = 3 
        
        # The number of submovements for the ZFT
        self.n_act = 0 

        # Controller Parameters of the Controller 
        # Task-space Impedances
        self.Kx = None
        self.Bx = None
        
        # The movement parameters, we save this as an array since multiple submovements may exist
        # For the positional part, 
        # xp0i : The initial (i) linear position (p) of the zero-torque trajectory (x0)
        self.xp0i = [ ]
        self.xp0f = [ ] 
        self.D    = [ ]
        self.ti   = [ ]

        
        # Check whether we need to save the data or not
        self.is_save_data = is_save_data
        
        # The number samples for the data
        self.Ns = 2 ** 15
        

    def set_linear_impedance( self, Kx:np.ndarray, Bx:np.ndarray ):
        
        # Resetting the Kq and Bq will be dangerous, hence asserting. 
        assert self.Kx is None and self.Bx is None
        
        # Check whether the 2D stiffness and damping matrices are in good shape.
        assert len( Kx ) == 3 and len( Kx[ 0 ] ) == 3
        assert len( Bx ) == 3 and len( Bx[ 0 ] ) == 3

        # Check whether both matrices are positive definite
        assert np.all( np.linalg.eigvals( Kx ) > 0 )
        assert np.all( np.linalg.eigvals( Bx ) > 0 )
            
        # If they have passed all the asserts, then saving it as an attribute
        self.Kx = Kx
        self.Bx = Bx
        
    def add_linear_movement( self, xp0i:np.ndarray, xp0f:np.ndarray, D:float, ti:float ):
        
        # Check whether the size of qi and qf are good
        assert len( xp0i ) == 3
        assert len( xp0f ) == 3

        # Check whether the D and ti are positive and non-negative, respectively. 
        assert D > 0 and ti >= 0 
        
        # If there is more than one movement, then initial position must be a zero array. 
        if self.n_act >= 1: assert np.all( ( xp0i == 0 ) )
        
        self.xp0i.append( xp0i )
        self.xp0f.append( xp0f )
        self.D.append(  D  )
        self.ti.append( ti )
        
        # Since we have added the new movement, add the number of movements 
        self.n_act += 1              
    

    def setup( self ):
        """
            Setting up the details of the movement. 
            
            This method should be called AFTER calling "set_ctrl_par" method.
        """

        # Before Setting, check whether the impedances are defined
        assert self.Kx is not None and self.Bx is not None 
        
        # Check whether the movement is defined 
        assert self.n_act >= 1 

        # The data array for saving the data 
        if self.is_save_data:
            self.t_arr   = np.zeros( self.Ns )
            self.q_arr   = np.zeros( ( 7, self.Ns ) )
            self.dq_arr  = np.zeros( ( 7, self.Ns ) )
            
            # The linear position and velocity of the robot
            self.xp_arr   = np.zeros( ( 3, self.Ns ) )
            self.dxp_arr  = np.zeros( ( 3, self.Ns ) )
            
            # The linear part of the zero-force trajectory
            self.xp0_arr  = np.zeros( ( 3, self.Ns ) )
            self.dxp0_arr = np.zeros( ( 3, self.Ns ) )
            

            # The Jacobian of the robot 
            self.J_arr    = np.zeros( ( 6, 7, self.Ns ) )

            # The input torque of the robot            
            self.tau_arr = np.zeros( ( 7, self.Ns ) )
        
            # The data pointer that we will use for saving the data 
            self.idx_data = 0 

        
    def calc_torque( self, t: float ):
        """
            Calculating the torque equation which is as follows:
            
            tau = J^T ( Kx ( x0 - x ) + Bx ( dx0 - dx )  )
        """
        
        q  = self.robot.get_arm_pose( self.which_arm )
        dq = self.robot.get_arm_velocity( self.which_arm )
        
        # The linear velocity part of the movement
        xp  = self.robot.get_end_effector_pos( self.which_arm )
        dxp = self.robot.get_end_effector_linear_vel( self.which_arm )
        
        # Get the whole Jacobian of the movement
        J = np.array( self.robot.kins[ self.which_arm ].jacobian( joint_values = q ) )
        
        # The positional and rotational jacobian of the end-effector
        Jp = J[ :3, : ]
        Jr = J[ 3:, : ]
                
        xp0  = np.zeros( 3 )
        dxp0 = np.zeros( 3 ) 
                
        # ============================================= #
        # ====== CALCULATION OF THE LINEAR PART ======= #
        # ============================================= #
        for i in range( self.n_act ): 
            for j in range( 3 ): 
                ZFT_pos, ZFT_vel = min_jerk_traj( t, self.ti[ i ], self.ti[ i ] + self.D[ i ], self.xp0i[ i ][ j ], self.xp0f[ i ][ j ], self.D[ i ] )
                xp0[  j ] += ZFT_pos
                dxp0[ j ] += ZFT_vel
            
        # Get the velocity of the linear part
        # Calculate the torque which should be inputed. 
        tau = Jp.T @ ( self.Kx @ ( xp0 - xp ) + self.Bx @ ( dxp0 - dxp ) )
        
        if self.is_save_data:
            self.t_arr[      self.idx_data ] = t
            self.q_arr[   :, self.idx_data ] = dict2arr( self.which_arm,  q )
            self.dq_arr[  :, self.idx_data ] = dict2arr( self.which_arm, dq )
            
            self.tau_arr[ :, self.idx_data ] = tau

            # The linear position and velocity of the robot
            self.xp_arr[  :, self.idx_data ] = xp
            self.dxp_arr[ :, self.idx_data ] = dxp
            self.xp0_arr[  :, self.idx_data ] = xp0
            self.dxp0_arr[ :, self.idx_data ] = dxp0
    
            # The Jacobian of the robot 
            self.J_arr[ :, :, self.idx_data ] = J 
            
            self.idx_data += 1

        return np.squeeze( tau )
    
    def publish_data( self, dir_name: str ):
        # Printing out all the details of the simulation as a file that is great for MATLAB compatability
        file_name = dir_name + "/" + self.name + ".mat"
        
        # [TODO] There will be a single liner to do this, including this in the parent class
        scipy.io.savemat( file_name, { 'name': self.name, 
                                       'time': self.t_arr[      :self.idx_data ], 
                                       'xp0i': self.xp0i, 
                                       'xp0f': self.xp0f, 
                                          'D': self.D, 
                                         'ti': self.ti, 
                                          'q': self.q_arr[   :, :self.idx_data ], 
                                         'dq': self.dq_arr[  :, :self.idx_data ], 
                                         'xp': self.xp_arr[   :, :self.idx_data ], 
                                        'dxp': self.dxp_arr[   :, :self.idx_data ], 
                                        'xp0': self.xp0_arr[   :, :self.idx_data ], 
                                       'dxp0': self.dxp0_arr[   :, :self.idx_data ],         
                                          'J': self.J_arr[ :, :, :self.idx_data ],
                                        'tau': self.tau_arr[ :, :self.idx_data ],
                                         'Kx': self.Kx, 'Bx': self.Bx }  )
        
class CartesianImpedanceControllerPosAndRotType1( ImpedanceController ):
    
    def __init__( self, robot, which_arm : str, name: str, is_save_data:bool = False ):
        
        super( ).__init__( robot, which_arm )
        self.type = "cartesian_impedance_controller_pos_and_rot_type1"
        self.name = name + "_" + self.type
        
        self.is_save_data = is_save_data
        
        # The number samples for the data
        self.Ns = 2 ** 15
        
    def setup( self ):
        # Before Setting, check whether the impedances are defined
        assert self.Kx is not None and self.Bx is not None 
        assert self.k  is not None and  self.b is not None 
        
        # Check whether the movement is defined 
        assert self.n_act >= 1 

        # The data array for saving the data 
        if self.is_save_data:
            self.t_arr   = np.zeros( self.Ns )
            self.q_arr   = np.zeros( ( 7, self.Ns ) )
            self.dq_arr  = np.zeros( ( 7, self.Ns ) )
            
            # The linear position and velocity of the robot
            self.xp_arr   = np.zeros( ( 3, self.Ns ) )
            self.dxp_arr  = np.zeros( ( 3, self.Ns ) )
            
            # The linear part of the zero-force trajectory
            self.xp0_arr  = np.zeros( ( 3, self.Ns ) )
            self.dxp0_arr = np.zeros( ( 3, self.Ns ) )
            
            # The current orientation

            # The Jacobian of the robot 
            self.J_arr    = np.zeros( ( 6, 7, self.Ns ) )

            # The input torque of the robot            
            self.tau_arr = np.zeros( ( 7, self.Ns ) )
        
            # The data pointer that we will use for saving the data 
            self.idx_data = 0 

        
    
    def set_linear_impedance( self, Kx:np.ndarray, Bx:np.ndarray ):
    
        # Resetting the Kq and Bq will be dangerous, hence asserting. 
        assert self.Kx is None and self.Bx is None
        
        # Check whether the 2D stiffness and damping matrices are in good shape.
        assert len( Kx ) == 3 and len( Kx[ 0 ] ) == 3
        assert len( Bx ) == 3 and len( Bx[ 0 ] ) == 3

        # Check whether both matrices are positive definite
        assert np.all( np.linalg.eigvals( Kx ) > 0 )
        assert np.all( np.linalg.eigvals( Bx ) > 0 )
            
        # If they have passed all the asserts, then saving it as an attribute
        self.Kx = Kx
        self.Bx = Bx
        
    def set_rotational_impedance( self, k:float, b: float ):
    
        # Resetting the Kq and Bq will be dangerous, hence asserting. 
        assert self.k is None and self.b is None
        
        # Check whether the 2D stiffness and damping matrices are in good shape.
        assert k > 0 and b > 0
            
        # If they have passed all the asserts, then saving it as an attribute
        self.k = k
        self.b = b
    
    def add_linear_movement( self, xp0i:np.ndarray, xp0f:np.ndarray, D:float, ti:float ):
        
        # Check whether the size of qi and qf are good
        assert len( xp0i ) == 3
        assert len( xp0f ) == 3

        # Check whether the D and ti are positive and non-negative, respectively. 
        assert D > 0 and ti >= 0 
        
        # If there is more than one movement, then initial position must be a zero array. 
        if self.n_act >= 1: assert np.all( ( xp0i == 0 ) )
        
        self.xp0i.append( xp0i )
        self.xp0f.append( xp0f )
        self.D.append(  D  )
        self.ti.append( ti )
        
        # Since we have added the new movement, add the number of movements 
        self.n_act += 1            
    
    def add_desired_orientation( self, quat_des: np.ndarray):
        
        assert len( quat_des ) == 4
        
        self.quat_des = quat_des
    
    def calc_torque( self, t: float ):
        q  = self.robot.get_arm_pose( self.which_arm )
        dq = self.robot.get_arm_velocity( self.which_arm )
        
        # The linear velocity part of the movement
        xp  = self.robot.get_end_effector_pos( self.which_arm )
        dxp = self.robot.get_end_effector_linear_vel( self.which_arm )
        
        # Get the whole Jacobian of the movement
        J = np.array( self.robot.kins[ self.which_arm ].jacobian( joint_values = q ) )
        
        # The positional and rotational jacobian of the end-effector
        Jp = J[ :3, : ]
        Jr = J[ 3:, : ]
                
        xp0  = np.zeros( 3 )
        dxp0 = np.zeros( 3 ) 
                
        # ============================================= #
        # ====== CALCULATION OF THE LINEAR PART ======= #
        # ============================================= #
        for i in range( self.n_act ): 
            for j in range( 3 ): 
                ZFT_pos, ZFT_vel = min_jerk_traj( t, self.ti[ i ], self.ti[ i ] + self.D[ i ], self.xp0i[ i ][ j ], self.xp0f[ i ][ j ], self.D[ i ] )
                xp0[  j ] += ZFT_pos
                dxp0[ j ] += ZFT_vel
            
        # Get the velocity of the linear part
        # Calculate the torque which should be inputed. 
        tau = Jp.T @ ( self.Kx @ ( xp0 - xp ) + self.Bx @ ( dxp0 - dxp ) )       
    
        # Adding the rotational part 
        quat_cur  = self.robot.get_end_effector_orientation( self.which_arm )
        w         = self.robot.get_end_effector_angular_vel( self.which_arm )
        
        R_cur = quat2rot( quat_cur )
        R_des = quat2rot( self.quat_des )
        Rdiff = R_cur.T @ R_des
        
        theta, axis_cur = quat2angx( rot2quat( Rdiff ) )
        axis_0   = R_cur @ axis_cur 
        
        m = axis_0 * self.k * theta - self.b * w
        
        tau += Jr.T @ m
        
        return tau
    
    def publish_data( self, dir_name: str ):
        pass
    
        
class CartesianImpedanceControllerPosAndRotType2( ImpedanceController ):
    

    def __init__( self, robot, which_arm : str, name: str, is_save_data:bool = False ):
        
        super( ).__init__( robot, which_arm )
        self.type = "cartesian_impedance_controller_pos_and_rot_type2"
        self.name = name + "_" + self.type
        
        self.is_save_data = is_save_data
        
        # The number samples for the data
        self.Ns = 2 ** 15
        
    def setup( self ):
        pass
        
    
    def set_linear_impedance( self, Kx:np.ndarray, Bx:np.ndarray ):
    
        # Resetting the Kq and Bq will be dangerous, hence asserting. 
        assert self.Kx is None and self.Bx is None
        
        # Check whether the 2D stiffness and damping matrices are in good shape.
        assert len( Kx ) == 3 and len( Kx[ 0 ] ) == 3
        assert len( Bx ) == 3 and len( Bx[ 0 ] ) == 3

        # Check whether both matrices are positive definite
        assert np.all( np.linalg.eigvals( Kx ) > 0 )
        assert np.all( np.linalg.eigvals( Bx ) > 0 )
            
        # If they have passed all the asserts, then saving it as an attribute
        self.Kx = Kx
        self.Bx = Bx
        
    def set_rotational_impedance( self, Kr:float, Br: float ):
    
        # Resetting the Kq and Bq will be dangerous, hence asserting. 
        # Resetting the Kq and Bq will be dangerous, hence asserting. 
        assert self.Kr is None and self.Br is None
        
        # Check whether the 2D stiffness and damping matrices are in good shape.
        assert len( Kr ) == 3 and len( Kr[ 0 ] ) == 3
        assert len( Br ) == 3 and len( Br[ 0 ] ) == 3
        
        # If they have passed all the asserts, then saving it as an attribute
        self.Kr = Kr
        self.Br = Br
    
    def add_linear_movement( self, xp0i:np.ndarray, xp0f:np.ndarray, D:float, ti:float ):
        
        # Check whether the size of qi and qf are good
        assert len( xp0i ) == 3
        assert len( xp0f ) == 3

        # Check whether the D and ti are positive and non-negative, respectively. 
        assert D > 0 and ti >= 0 
        
        # If there is more than one movement, then initial position must be a zero array. 
        if self.n_act >= 1: assert np.all( ( xp0i == 0 ) )
        
        self.xp0i.append( xp0i )
        self.xp0f.append( xp0f )
        self.D.append(  D  )
        self.ti.append( ti )
        
        # Since we have added the new movement, add the number of movements 
        self.n_act += 1            
    
    def add_desired_orientation( self, Rdes: np.ndarray):
        
        # Up to this point, we only set a single orientation
        self.Rdes = Rdes 
    
    def add_desired_orientation( self, quat_des: np.ndarray):
        
        assert len( quat_des ) == 4
        
        self.quat_des = quat_des
    
    def calc_torque( self, t: float ):
        q  = self.robot.get_arm_pose( self.which_arm )
        dq = self.robot.get_arm_velocity( self.which_arm )
        
        # The linear velocity part of the movement
        xp  = self.robot.get_end_effector_pos( self.which_arm )
        dxp = self.robot.get_end_effector_linear_vel( self.which_arm )
        
        # Get the whole Jacobian of the movement
        J = np.array( self.robot.kins[ self.which_arm ].jacobian( joint_values = q ) )
        
        # The positional and rotational jacobian of the end-effector
        Jp = J[ :3, : ]
        Jr = J[ 3:, : ]
                
        xp0  = np.zeros( 3 )
        dxp0 = np.zeros( 3 ) 
                
        # ============================================= #
        # ====== CALCULATION OF THE LINEAR PART ======= #
        # ============================================= #
        for i in range( self.n_act ): 
            for j in range( 3 ): 
                ZFT_pos, ZFT_vel = min_jerk_traj( t, self.ti[ i ], self.ti[ i ] + self.D[ i ], self.xp0i[ i ][ j ], self.xp0f[ i ][ j ], self.D[ i ] )
                xp0[  j ] += ZFT_pos
                dxp0[ j ] += ZFT_vel
            
        # Get the velocity of the linear part
        # Calculate the torque which should be inputed. 
        tau = Jp.T @ ( self.Kx @ ( xp0 - xp ) + self.Bx @ ( dxp0 - dxp ) )       
    
        # Adding the rotational part 
        # FILL-in
        
        return tau
    
    def publish_data( self, dir_name: str ):
        pass
        
# Mainly for Debugging the Robot Controller
if __name__ == "__main__":
    
    parser  = argparse.ArgumentParser( formatter_class = argparse.RawTextHelpFormatter )
    parser.add_argument('-c', '--controller' , dest = 'ctrl_type', action = 'store', type =  str, help = C.CONTROLLER_DESCRIPTIONS)
    
    args = parser.parse_args( rospy.myargv( )[ 1: ] )

    # Initializing the Baxter Robot
    my_baxter = Baxter( None )
    
    # for imp in imp_arr: imp.setup( )
    
    ts = rospy.Time.now( )
    t  = 0
    
    if   args.ctrl_type == "print_controller":
        my_ctrl = PrintController( my_baxter )
        my_ctrl.run( )
        
    elif args.ctrl_type == "joint_position_controller":
        my_ctrl = JointPositionController( my_baxter )
        my_ctrl.add_movement( which_arm = "right", pose2go = C.GRASP_POSE_UP                   , joint_vel = 0.5, toff = 3 )    
        my_ctrl.add_movement( which_arm =  "left", pose2go = pose_right2left( C.GRASP_POSE_UP ), joint_vel = 0.5, toff = 3 )
        my_ctrl.run( )
        
        my_ctrl = PrintController( my_baxter )
        my_ctrl.run( )
        
        
    elif args.ctrl_type == "imp_controller":     
        # Move the robot to the Specific Joint Posture
        pass

    else:
        
        pass            
    