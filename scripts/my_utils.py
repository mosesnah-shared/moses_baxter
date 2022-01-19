import time
import sys

import rospy

from baxter_interface import (
    DigitalIO,
    Gripper,
    Navigator,
    CHECK_VERSION,
)



from my_constants import Constants as C


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


class Logger(object):
    def __init__(self, record_data = False):
        self.record_data = record_data
        self.terminal    = sys.stdout

        if record_data == True:
            time_now = time.strftime( '%Y_%m_%d-%H_%M' )                 # Redirecting the stdout to file
            self.log = open( C.SAVE_DIR + 'baxter_'  + time_now + '.txt' , 'w')
        else:
            self.log = None

    def write(self, message ):
        self.terminal.write( message )

        if self.log is not None:
            self.log.write( message )

    def flush(self):
        # this flush method is needed for python 3 compatibility.
        # this handles the flush command by doing nothing.
        # you might want to specify some extra behavior here.
        pass
