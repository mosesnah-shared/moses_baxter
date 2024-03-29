import os
import sys
import time
import rospy
import datetime
import numpy as np

from my_constants import Constants as C

from baxter_interface import DigitalIO, Gripper, Navigator, CHECK_VERSION 

def min_jerk_traj( t: float, ti: float, tf: float, pi: float, pf: float, D: float ):
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

def index_containing_substring( the_list: list, substring: str ):
    
    for i, s in enumerate( the_list ):
        if substring in s:
              return i
          
    return -1


def pose_right2left( pose: dict ):
    """
        Changing the dictionary key's prefix name from "right_" to "left_", and flipping the sign too
        The reason why we don't have left2right is because we will follow the "right-hand" convention,
        meaning, all the constants in "my_constants.py" are saved as "right" hand informatino

        Arguments:
            [1] pose ( dict ): 7-element dictionary with keys "right_" + s0, s1, e0, e1, w0, w1, w2
    """
    
    # Check whether the given dictionary has all the "right_" + s0, s1, e0, e1, w0, w1 and w2 on the keys.
    assert all( [ c in pose.keys( ) for c in C.JOINT_NAMES[ "right" ] ] )   

    new_pose = dict( )
    
    # We can write this with a single line, but just for code readability, using the for loop 
    for right_name in C.JOINT_NAMES[ "right" ]:
        left_name = C.RIGHT2LEFT[ right_name ]
        new_pose[ left_name ] = C.LEFT_JOINT_SIGN[ left_name ] * pose[ right_name ]

    return new_pose

def task_right2left( p: np.ndarray ):
    """
        Change the task space 3D position from left to right
    """
    
    # Change the right posture direction 
    assert len( p ) == 3
    
    # Create a new object
    new_p = np.zeros( 3 )
    
    # Flip the y sign
    new_p[ 0 ] =  p[ 0 ]
    new_p[ 1 ] = -p[ 1 ]
    new_p[ 2 ] =  p[ 2 ]
    
    return new_p


def dict2arr( which_arm: str, my_dict: dict  ):
    """
        Change the dictionary to array value     
        
        Args: 
        
            [2] my_dict - should contain joints s0, s1, e0, e1, w0, w1, w2 
        
    """

    assert which_arm in [ "right", "left" ]

    # Check whether s0, s1, e0, e1, w0, w1, w2, which is the list of C.BASIC_JOINT_NAMES are in the my_dict keys    
    assert all( [ c in my_dict.keys( ) for c in C.JOINT_NAMES[ which_arm ] ] ) 
    
    # The length of the dictionary must be 7
    assert len( my_dict ) == 7
    
    # Generate a 1x7 array with numbers ordered as s0, s1, e0, e1, w0, w1, w2 
    # Again, for readability, we will set this using a for loop
    return np.array( [ my_dict[ key ] for key in C.JOINT_NAMES[ which_arm ] ] )

def arr2dict( which_arm: str, arr: np.ndarray ):

    assert which_arm in [ "right", "left" ]
    assert len( arr ) == 7

    return { joint_name : arr[ i ] for i, joint_name in enumerate( C.JOINT_NAMES[ which_arm ] ) }

def poses_delta( which_arm: str, pose1: dict, pose2: dict ):
    """
        conduct pose2 - pose1 for each element
    """
    assert which_arm in [ "right", "left" ]

    assert all( [ c in pose1.keys( ) for c in C.JOINT_NAMES[ which_arm ] ] )
    assert all( [ c in pose2.keys( ) for c in C.JOINT_NAMES[ which_arm ] ] )

    return { key: pose2[ key ] - pose1[ key ] for key in pose1.keys( ) }


def make_dir( ):
    
    dir_name  = C.SAVE_DIR + datetime.datetime.now().strftime( '%Y_%m_%d' )
    
    if not os.path.exists( dir_name ):
        os.mkdir( dir_name )
        
    # After making the directory, then generating a subdirectory including h_m_s
    sub_dir_name = dir_name + "/" + datetime.datetime.now().strftime( '%Y%m%d_%H%M%S' )
    
    if not os.path.exists( sub_dir_name ):
        os.mkdir( sub_dir_name )
        
    return sub_dir_name


class GripperConnect( object ):
    """
        Connects wrist button presses to gripper open/close commands.

        Uses the DigitalIO Signal feature to make callbacks to connected
        action functions when the button values change.
    """

    def __init__( self, arm, lights = True ):
        """
            @type arm: str
            @param arm: arm of gripper to control {left, right}
            @type lights: bool
            @param lights: if lights should activate on cuff grasp
        """
        self._arm = arm

        # inputs
        self._close_io = DigitalIO( '%s_upper_button' % ( arm, ) )  # 'dash' btn
        self._open_io  = DigitalIO( '%s_lower_button' % ( arm, ) )  # 'circle' btn
        self._light_io = DigitalIO( '%s_lower_cuff'   % ( arm, ) )  # cuff squeeze

        # outputs
        self._gripper = Gripper(   '%s' % ( arm, ), CHECK_VERSION )
        self._nav     = Navigator( '%s' % ( arm, ) )

        # connect callback fns to signals
        if self._gripper.type( ) != 'custom':
            if not ( self._gripper.calibrated( ) or
                     self._gripper.calibrate(  ) == True ):
                rospy.logwarn( "%s (%s) calibration failed.",
                               self._gripper.name.capitalize( ),
                               self._gripper.type( ) )
        else:
            msg = (("%s (%s) not capable of gripper commands."
                   " Running cuff-light connection only.") %
                   ( self._gripper.name.capitalize( ), self._gripper.type( ) ) )
            rospy.logwarn( msg )

        self._gripper.on_type_changed.connect( self._check_calibration )
        self._open_io.state_changed.connect(   self._open_action )
        self._close_io.state_changed.connect(  self._close_action )

        if lights:
            self._light_io.state_changed.connect(self._light_action)

        rospy.loginfo("%s Cuff Control initialized...",
                      self._gripper.name.capitalize( ) )

    def _open_action( self, value ):
        if value and self._is_grippable():
            rospy.logdebug( "gripper open triggered" )
            self._gripper.open()

    def _close_action( self, value ):
        if value and self._is_grippable():
            rospy.logdebug( "gripper close triggered" )
            self._gripper.close()

    def _light_action(self, value):
        if value:
            rospy.logdebug("cuff grasp triggered")
        else:
            rospy.logdebug("cuff release triggered")
        self._nav.inner_led = value
        self._nav.outer_led = value

    def _check_calibration(self, value):
        if self._gripper.calibrated():
            return True
        elif value == 'electric':
            rospy.loginfo("calibrating %s...",
                          self._gripper.name.capitalize())
            return (self._gripper.calibrate() == True)
        else:
            return False

    def _is_grippable(self):
        return ( self._gripper.calibrated( ) and self._gripper.ready( ) )

def skew_sym( w ):
    assert len( w ) == 3

    wtilde = np.zeros( ( 3, 3 ) )

    wtilde[ 0, 1 ] = -w[ 2 ]
    wtilde[ 0, 2 ] =  w[ 1 ]
    wtilde[ 2, 1 ] = -w[ 0 ]

    wtilde[ 1, 0 ] =  w[ 2 ]
    wtilde[ 2, 0 ] = -w[ 1 ]
    wtilde[ 1, 2 ] =  w[ 0 ]

    return wtilde


def quat2angx( q ):

    assert q[ 0 ] <= 1
    theta = 2 * np.arccos( q[ 0 ] )
    axis = np.copy( q[ 1: ] )
    
    # If the axis values are super small, then 
    tmp = np.sum( axis**2 )
    
    if tmp != 0:
        axis = axis/ np.sqrt( tmp )

    else:
        axis = np.array( [ 1., 0., 0. ] )
        theta = 0 

    return theta, axis


def rot2quat( R: np.ndarray ):
    # [REF] https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
    # [REF] From Johannes

    assert len( R ) == 3 and len( R[ 0 ] ) == 3

    q = np.zeros( 4 )

    R00 = np.trace( R )
    tmp = np.array( [ R00, R[ 0,0 ], R[ 1,1 ], R[ 2,2 ] ] )
    k = np.argmax( tmp )

    q[ k ] = 0.5 * np.sqrt( 1 + 2 * tmp[ k ] - R00 )

    if k == 0:
        q[ 1 ] = 0.25/q[ k ] * ( R[ 2, 1 ] - R[ 1, 2 ] )
        q[ 2 ] = 0.25/q[ k ] * ( R[ 0, 2 ] - R[ 2, 0 ] )
        q[ 3 ] = 0.25/q[ k ] * ( R[ 1, 0 ] - R[ 0, 1 ] )

    elif k == 1:
        q[ 0 ] = 0.25/q[ k ] * ( R[ 2, 1 ] - R[ 1, 2 ] )
        q[ 2 ] = 0.25/q[ k ] * ( R[ 1, 0 ] + R[ 0, 1 ] )
        q[ 3 ] = 0.25/q[ k ] * ( R[ 0, 2 ] + R[ 2, 0 ] )

    elif k == 2:
        q[ 0 ] = 0.25/q[ k ] * ( R[ 0, 2 ] - R[ 2, 0 ] )
        q[ 2 ] = 0.25/q[ k ] * ( R[ 1, 0 ] + R[ 0, 1 ] )
        q[ 3 ] = 0.25/q[ k ] * ( R[ 2, 1 ] + R[ 1, 2 ] )

    elif k == 3:
        q[ 0 ] = 0.25/q[ k ] * ( R[ 1, 0 ] - R[ 0, 1 ] )
        q[ 1 ] = 0.25/q[ k ] * ( R[ 0, 2 ] + R[ 2, 0 ] )
        q[ 2 ] = 0.25/q[ k ] * ( R[ 2, 1 ] + R[ 1, 2 ] )

    if q[ 0 ] < 0 : q = -q

    return q


def quat2rot( quat: np.ndarray ):

    # [REF] https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html

    assert len( quat ) == 4

    q0, q1, q2 ,q3  = quat[:]    

    R = np.zeros( ( 3, 3 ) )

    R[ 0, 0 ] = q0 ** 2 + q1 ** 2 - q2 ** 2 - q3 ** 2
    R[ 0, 1 ] = 2 * q1 * q2 - 2 * q0 * q3
    R[ 0, 2 ] = 2 * q1 * q3 + 2 * q0 * q2

    R[ 1, 0 ] = 2 * q1 * q2 + 2 * q0 * q3
    R[ 1, 1 ] = q0 ** 2 - q1 ** 2 + q2 ** 2 - q3 ** 2
    R[ 1, 2 ] = 2 * q2 * q3 - 2 * q0 * q1

    R[ 2, 0 ] = 2 * q1 * q3 - 2 * q0 * q2
    R[ 2, 1 ] = 2 * q2 * q3 + 2 * q0 * q1
    R[ 2, 2 ] = q0 ** 2 - q1 ** 2 - q2 ** 2 + q3 ** 2

    return R