#!/usr/bin/python3

# =========================================================== #
# [Author] Moses C. Nah
# [Email]  mosesnah@mit.edu
# [Date]   2022.01.26
# =========================================================== #

import sys
import datetime
import rospy
from moses_baxter.msg import my_msg
from my_constants import Constants as C


class Logger(object):
    def __init__( self, logfile ):
        self.terminal = sys.stdout
        self.log = open( logfile, "w+" )

    def write(self, message ):
        self.terminal.write( message )
        self.log.write( message )

    def flush( self ):
        # this flush method is needed for python 3 compatibility.
        # this handles the flush command by doing nothing.
        # you might want to specify some extra behavior here.
        pass


class myListener( ):
    def __init__( self ):
        self.node = rospy.init_node(  'my_listener', anonymous = True )
        self.sub  = rospy.Subscriber( "my_baxter" , my_msg, self.callback  )

        self.r    = rospy.Rate( 20 )  # 20Hz
        self.on   = True

        self.ti   = rospy.Time.now()
        self.MAX_TIME = 30

    def callback( self, data ):
        self.on = data.on
        print( "[time]", data.stamp, "[qo_L]" , data.q0_L  )
        print( "[time]", data.stamp, "[q_L]"  , data.q_L   )
        print( "[time]", data.stamp, "[dq_L]" , data.dq_L  )
        print( "[time]", data.stamp, "[tau_L]", data.tau_L )

        print( "[time]", data.stamp, "[qo_R]" , data.q0_R  )
        print( "[time]", data.stamp, "[q_R]"  , data.q_R   )
        print( "[time]", data.stamp, "[dq_R]" , data.dq_R  )
        print( "[time]", data.stamp, "[tau_R]", data.tau_R )


    def listen( self ):

        while self.on and ( rospy.Time.now( ) - self.ti ).to_sec() < self.MAX_TIME :
            self.r.sleep( )


if __name__ == '__main__':

    listener = myListener( )
    sys.stdout = Logger( C.SAVE_DIR + datetime.datetime.now().strftime( '%Y%m%d_%H%M%S' ) + '.txt' )

    listener.listen( )
