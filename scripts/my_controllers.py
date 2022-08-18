#!/usr/bin/python3

import rospy
import scipy.io
import numpy             as np
import matplotlib.pyplot as plt

# Local Library, under moses/scripts
from my_robot     import Baxter
from my_constants import Constants as C
from my_utils     import min_jerk_traj, pose_right2left, dict2arr, arr2dict, poses_delta, make_dir

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
                    # for limb_name in [ "right", "left" ]:
                    print( self.robot.get_end_effector_pos( "right" ) ) 
                        # print( self.robot.get_end_effector_linear_vel( limb_name ) ) 
                        

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
        
        
        
class CartesianImpedanceController( ImpedanceController ):
    
    def __init__( self, robot, which_arm : str, which_type: str ):
        
        super( ).__init__( robot, which_arm )
        self.type           = "cartesian_impedance_controller"
        self.ctrl_par_names = [ "Kx", "Bx", "xi", "xf", "D", "ti" ]

        # Check whether the input is position only or position and rotation 
        assert which_type in [ "pos", "pos_and_rot"]
        self.which_type = which_type
        
        # If position only        , set the size of the matrices as 3
        # If position and rotation, set the size of the matrices as 6
        self.n = 3 if which_type == "pos" else 6
        
        # The number of submovements for the ZFT
        self.n_act = 0 
        
        # The number samples for the data
        self.Ns = 2 ** 15

        # Controller Parameters of the Controller 
        # Task-space Impedances
        self.Kx = None
        self.Bx = None
        
        # The movement parameters, we save this as an array since multiple submovements may exist
        self.xi = [ ]
        self.xf = [ ] 
        self.D  = [ ]
        self.ti = [ ]
            
    def set_impedance( self, Kx:np.ndarray, Bx:np.ndarray ):
        
        # Resetting the Kq and Bq will be dangerous, hence asserting. 
        assert self.Kx is None and self.Bx is None
        
        # Check whether the 2D stiffness and damping matrices are in good shape.
        assert len( Kx ) == self.n and len( Kx[ 0 ] ) == self.n 
        assert len( Bx ) == self.n and len( Bx[ 0 ] ) == self.n         

        # Check whether both matrices are positive definite
        assert np.all( np.linalg.eigvals( Kx ) > 0 )
        assert np.all( np.linalg.eigvals( Bx ) > 0 )
            
        # If they have passed all the asserts, then saving it as an attribute
        self.Kx = Kx
        self.Bx = Bx
        
    def add_movement( self, xi:np.ndarray, xf:np.ndarray, D:float, ti:float ):
        
        # Check whether the size of qi and qf are good
        assert len( xi ) == self.n
        assert len( xf ) == self.n

        # Check whether the D and ti are positive and non-negative, respectively. 
        assert D > 0 and ti >= 0 
        
        # If there is more than one movement, then qi must be a zero array. 
        if self.n_act >= 1: assert np.all( ( xi == 0 ) )
        
        self.xi.append( xi )
        self.xf.append( xf )
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
        self.t_arr   = np.zeros( self.Ns )
        self.q_arr   = np.zeros( ( 7, self.Ns ) )
        self.dq_arr  = np.zeros( ( 7, self.Ns ) )
        self.q0_arr  = np.zeros( ( 7, self.Ns ) )
        self.dq0_arr = np.zeros( ( 7, self.Ns ) )
        self.x_arr   = np.zeros( ( 7, self.Ns ) )
        self.dx_arr  = np.zeros( ( 6, self.Ns ) )
        self.x0_arr  = np.zeros( ( self.n, self.Ns ) )
        self.dx0_arr = np.zeros( ( self.n, self.Ns ) )
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
        
        x  = self.robot.get_end_effector_pos( self.which_arm )
        dx = self.robot.get_end_effector_linear_vel( self.which_arm )
        
        # Get the Jacobian of the linear part 
        J = self.robot.kins[ self.which_arm ].jacobian( joint_values = q )
                
        x0  = np.zeros( 3 )
        dx0 = np.zeros( 3 ) 
                
        # Calculating the linear ZFT trajectory
        for i in range( self.n_act ): 
            for j in range( 3 ): 
                ZFT_pos, ZFT_vel = min_jerk_traj( t, self.ti[ i ], self.ti[ i ] + self.D[ i ], self.xi[ i ][ j ], self.xf[ i ][ j ], self.D[ i ] )
                x0[  j ] += ZFT_pos
                dx0[ j ] += ZFT_vel
            
        # Get the velocity of the linear part
        # Calculate the torque which should be inputed. 
        tau = J.T @ ( self.Kx @ ( x0 - x ) + self.Bx @ ( dx0 - dx ) )
        
        # If the type is 
        if self.which_type == "pos_and_rot":
            NotImplementedError( )
        # tau += ROTATION TYPE
        
        if self.is_save_data:
            self.t_arr[      self.idx_data ] = t
            self.q_arr[   :, self.idx_data ] = q
            self.dq_arr[  :, self.idx_data ] = dq
            self.x0_arr[  :, self.idx_data ] = x0
            self.dx0_arr[ :, self.idx_data ] = dx0
            self.tau_arr[ :, self.idx_data ] = tau
            
            self.idx_data += 1

        return tau
        
# Mainly for Debugging the Robot Controller
if __name__ == "__main__":

    # Initializing the Baxter Robot
    my_baxter = Baxter( None )
    
    # Flag for saving data
    is_save_data = False

    # Setting up the Impedance Parameters of the Joint
    Kq_mat = np.diag( [ 15.0, 15.0, 8.0, 10.0, 3.0, 10.0, 1.5 ]  )
    Bq_mat = 0.2 * Kq_mat

    # ============================================================ #
    # ================== RIGHT LIMB IMPEDANCES =================== #
    # ============================================================ #

    # First RIGHT Impedance
    impR_1 = JointImpedanceController( my_baxter, which_arm = "right", name = "right_imp1", is_save_data = is_save_data )
    impR_1.set_impedance( Kq = Kq_mat, Bq = Bq_mat )
                                    
    # Add the movements                               
    qi = dict2arr( which_arm = "right", my_dict = my_baxter.get_arm_pose( which_arm = "right" ) )
    qf = dict2arr( which_arm = "right", my_dict = C.GRASP_POSE )
    D1 = 3.0
    impR_1.add_movement( qi = qi, qf = qf, D = D1, ti = 0. )

    # Second RIGHT Impedance    
    # Time offset of the 2nd movement 
    alpha = -0.2
    qi = np.zeros( 7 )
    qf = dict2arr( "right", poses_delta( "right", C.GRASP_POSE, C.MID_POSE ) )
    D2 = 5.0
    impR_1.add_movement( qi = qi, qf = qf, D = D2, ti = ( 1 + alpha ) * D1 )
        
    # ============================================================ #
    # =================== LEFT LIMB IMPEDANCES =================== #
    # ============================================================ #
        
    impL_1 = JointImpedanceController( my_baxter, which_arm = "left", name = "left_imp1", is_save_data = is_save_data )
    impL_1.set_impedance( Kq = Kq_mat, Bq = Bq_mat )
    
    # Add the movements 
    qi = dict2arr( which_arm = "left", my_dict = my_baxter.get_arm_pose( which_arm = "left" ) )
    qf = dict2arr( which_arm = "left", my_dict = pose_right2left( C.GRASP_POSE ) )
    D1 = 3.0
    impL_1.add_movement( qi = qi, qf = qf, D = D1, ti = 0. ) 

    # Saving these impedances as an array to iterate over 
    imp_arr  = [ impR_1, impL_1 ]
    
    for imp in imp_arr: imp.setup( )
    
    ts = rospy.Time.now( )
    t  = 0
    
    # Running the main loop
    while not rospy.is_shutdown( ) and t <= 13:

        tau = { "right": np.zeros( 7 ), "left": np.zeros( 7 ) }
        
        # Iterating over the impedances 
        for imp in imp_arr:
            
            # Calculate the Torque of the arm 
            tmp_tau = imp.calc_torque( t )
            tau[ imp.which_arm ] += tmp_tau 

        for limb_name in [ "right", "left" ]:
            my_baxter.arms[ limb_name ].set_joint_torques( arr2dict( limb_name, arr = tau[ limb_name ]  )  )
            
        # if is_publish_data:        
        my_baxter.control_rate.sleep( )
        
        t = ( rospy.Time.now( ) - ts ).to_sec( )


    # After finishing the movement, publishing the data 
    # The number of impedances should match the message
    # Each Impedance Must have an independent Message
    # Check whether you will publish the data or not.
    dir_name = make_dir( )
    
    for imp in imp_arr:
        imp.publish_data( dir_name = dir_name )
        
    