#!/usr/bin/python3

import rospy
import numpy             as np
import matplotlib.pyplot as plt

# Local Library, under moses/scripts
from my_robot     import Baxter
from my_constants import Constants as C
from my_utils     import min_jerk_traj, pose_right2left, dict2arr, poses_delta

# Baxter Library
import baxter_external_devices  

# Local Message Defined
from moses_baxter.msg import my_msg


np.set_printoptions( linewidth = 4000, precision = 8)

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

    def __init__( self, robot, which_arm : str, name : str ):

        super().__init__( robot, which_arm )
        self.type           = "joint_impedance_controller"
        self.name           = name + "_" + self.type
        self.ctrl_par_names = [ "Kq", "Bq", "qi", "qf", "D", "ti" ]
        
        
        # Initialize all the ctrl_par_names to None
        [ setattr( self, key, None ) for key in self.ctrl_par_names ]
                
    def setup( self, Kq, Bq, qi, qf, D, ti ):
        """
            Setting up the details of the movement. 
            
            This method should be called AFTER calling "set_ctrl_par" method.
        """
        
        # Check whether the stiffness and damping matrices are in good shape.
        assert len( Kq ) == 7 and len( Kq[ 0 ] ) == 7 
        assert len( Bq ) == 7 and len( Bq[ 0 ] ) == 7 

        # Check whether both matrices are positive definite
        assert np.all( np.linalg.eigvals( Kq ) > 0 )
        assert np.all( np.linalg.eigvals( Bq ) > 0 )

        # Check whether the size of qi and qf are good
        assert len( qi ) == 7
        assert len( qf ) == 7

        # Check whether the D and ti are positive and non-negative, respectively. 
        assert D > 0 and ti >= 0 
        
        # If all the given data is correct, save those 
        self.set_ctrl_par( Kq = Kq, Bq = Bq, qi = qi, qf = qf, D = D, ti = ti )

        # Double check whether the values are assigned 
        assert all( [ getattr( self, c ) is not None for c in self.ctrl_par_names ]  )

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
        
        # Calculate the ZFT for given time t
        print(  "t",       t )
        print( "qi", self.qi )
        print( "qf", self.qf )
        
        for i in range( 7 ): 
            ZFT_pos, ZFT_vel = min_jerk_traj( t, self.ti, self.ti + self.D, self.qi[ i ], self.qf[ i ], self.D )
            q0[  i ] = ZFT_pos
            dq0[ i ] = ZFT_vel
            
        # Calculate the torque which should be inputed. 
        tau = self.Kq @ ( q0 - q ) + self.Bq @ ( dq0 - dq )

        return tau, q0, dq0, q, dq
    
   
class CartesianImpedanceController( ImpedanceController ):
    
    def __init__( self, robot, which_arm : str ):
        
        super().__init__( robot, which_arm )
        self.type           = "cartesian_impedance_controller"
        self.ctrl_par_names = [ "Kx", "Bx", "xi", "xf", "D", "ti" ]

    def setup( self ):
        """
            Setting up the details of the movement. 
            
            This method should be called AFTER calling "set_ctrl_par" method.
        """
        # Check whether the stiffness and damping matrices are in good shape.
        # assert len( self.Kx ) == 7 and len( self.Kx[ 0 ] ) == 7 
        # assert len( self.Bx ) == 7 and len( self.Bx[ 0 ] ) == 7 

        # # Check whether both matrices are positive definite
        # assert np.all( np.linalg.eigvals( self.Kq ) > 0 )
        # assert np.all( np.linalg.eigvals( self.Bq ) > 0 )

        # # Check whether the size of qi and qf are good
        # assert len( self.qi ) == 7
        # assert len( self.qf ) == 7

        # # Check whether the D and ti are positive and non-negative, respectively. 
        # assert self.D > 0 and self.ti >= 0 
        
        # # If all the given data is correct, save those 
        # self.set_ctrl_par( Kq = Kq, Bq = Bq, qi = qi, qf = qf, D = D, ti = ti )

        # # Double check whether the values are assigned 
        # assert all( [ getattr( self, c ) is not None for c in self.ctrl_par_names ]  )
        NotImplementedError( )
        
    def calc_torque( self, t: float ):
        """
            Calculating the torque equation which is as follows:
        """
        q = self.robot.get_arm_pose( self.which_arm )
        J = self.robot.kins[ self.which_arm ].jacobian( joint_values = q )
    

if __name__ == "__main__":

    my_baxter = Baxter( None )

    # Setting up the impedance parameters 
    Kq_mat = np.diag( [ 15.0, 15.0, 8.0, 10.0, 3.0, 10.0, 1.5 ]  )
    Bq_mat = 0.2 * Kq_mat

    # First Impedance
    imp1 = JointImpedanceController( my_baxter, "right", "ctrl1" )
    qi = dict2arr( "right", C.GRASP_POSE )
    qf = dict2arr( "right", C.MID_POSE   )
    D1 = 1.0
    imp1.setup( Kq = Kq_mat, Bq = Bq_mat, qi = qi, qf = qf, D = D1, ti = 0 )
    
    # Offset of the 2nd movement 
    alpha = -0.3
    
    # Second Impedance
    imp2 = JointImpedanceController( my_baxter, "right", "ctrl2" )
    qi = np.zeros( 7 )
    qf = dict2arr( "right", poses_delta( "right", C.MID_POSE, C.FINAL_POSE ) )
    D2 = 2.0
    imp2.setup( Kq = Kq_mat, Bq = Bq_mat, qi = qi, qf = qf, D = D2, ti = ( 1 + alpha ) * D1 )
    
    # Saving these impedances as an array to iterate over 
    imp_arr = [ imp1, imp2 ]
    
    # Check whether you will publish the data or not.
    is_publish_data = True
    
    if is_publish_data:
        pub = rospy.Publisher( 'my_msg_print' , my_msg ) 
        msg    = my_msg()
        msg.on = True

    for t in np.linspace( 0, 8, 100 ):
        
        tau = { "right": np.zeros( 7 ), "left": np.zeros( 7 ) }
        
        for imp in imp_arr:
            
            # Calculate the Torque of the arm 
            tmp_tau, q0, dq0, q, dq = imp.calc_torque( t )
            tau[ imp.which_arm ] += tmp_tau 

        if is_publish_data:
            
            msg.stamp = t
            
            msg.x   = q 
            msg.dx  = dq 
            msg.x0  = q0 
            msg.dx0 = dq0 

            msg.K   = imp.Kq.flatten( )
            msg.B   = imp.Bq.flatten( )
            
            msg.tau = tmp_tau
            
            msg.name      = imp.name
            msg.which_arm = imp.which_arm
            
            pub.publish( msg )
            