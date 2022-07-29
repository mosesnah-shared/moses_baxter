#!/usr/bin/python3

"""
# =========================================================== #
# [Author            ] Moses C. Nah
# [Email             ] mosesnah@mit.edu
# [Date Created      ] 2022.01.26
# [Last Modification ] 2022.04.17
# =========================================================== #
"""

import os
import sys
import datetime
import rospy
import numpy as np

from moses_baxter.msg import my_msg
from my_constants     import Constants as C


np.set_printoptions( precision = 7, suppress = True )


class Logger( object ):

    def __init__( self, logfile ):
        self.terminal = sys.stdout
        self.log      = open( logfile, "w+" )

    def write( self, msg ):
        self.terminal.write( msg )
        self.log.write(      msg )

    def flush( self ):
        # this flush method is needed for python 3 compatibility.
        # this handles the flush command by doing nothing.
        # you might want to specify some extra behavior here.
        pass


class Listener( ):
    def __init__( self ):
        self.node = rospy.init_node(  'my_listener', anonymous = True )
        self.sub  = rospy.Subscriber( "my_msg_print" , my_msg, self.callback  )

        self.r    = rospy.Rate( 20 )  # 20Hz
        self.on   = True

        self.ti   = rospy.Time.now()
        self.MAX_TIME = 40

    def callback( self, data ):
        self.on = data.on

        print( "[name] { }  [time] {:.7f} [qo_L]   {:}".format( data.name, data.stamp, '(' + ', '.join( ('%.6f' % f ) for f in data.q0_L   ) + ')' )  )
        print( "[name] { }  [time] {:.7f} [dqo_L]  {:}".format( data.name, data.stamp, '(' + ', '.join( ('%.6f' % f ) for f in data.dq0_L  ) + ')' )  )
        print( "[name] { }  [time] {:.7f} [q_L]    {:}".format( data.name, data.stamp, '(' + ', '.join( ('%.6f' % f ) for f in data.q_L    ) + ')' )  )
        print( "[name] { }  [time] {:.7f} [dq_L]   {:}".format( data.name, data.stamp, '(' + ', '.join( ('%.6f' % f ) for f in data.dq_L   ) + ')' )  )
        print( "[name] { }  [time] {:.7f} [tau_L]  {:}".format( data.name, data.stamp, '(' + ', '.join( ('%.6f' % f ) for f in data.tau_L  ) + ')' )  )

        print( "[name] { }  [time] {:.7f} [qo_R]   {:}".format( data.name, data.stamp, '(' + ', '.join( ('%.6f' % f ) for f in data.q0_R   ) + ')' )  )
        print( "[name] { }  [time] {:.7f} [dqo_R]  {:}".format( data.name, data.stamp, '(' + ', '.join( ('%.6f' % f ) for f in data.dq0_R  ) + ')' )  )
        print( "[name] { }  [time] {:.7f} [q_R]    {:}".format( data.name, data.stamp, '(' + ', '.join( ('%.6f' % f ) for f in data.q_R    ) + ')' )  )
        print( "[name] { }  [time] {:.7f} [dq_R]   {:}".format( data.name, data.stamp, '(' + ', '.join( ('%.6f' % f ) for f in data.dq_R   ) + ')' )  )
        print( "[name] { }  [time] {:.7f} [tau_R]  {:}".format( data.name, data.stamp, '(' + ', '.join( ('%.6f' % f ) for f in data.tau_R  ) + ')' )  )


    def listen( self ):

        while self.on and ( rospy.Time.now( ) - self.ti ).to_sec() < self.MAX_TIME :
            self.r.sleep( )


if __name__ == '__main__':


    listener = Listener( )

    dir_name  = C.SAVE_DIR + datetime.datetime.now().strftime( '%Y_%m_%d' )
    file_name = dir_name + "/" + datetime.datetime.now().strftime( '%Y%m%d_%H%M%S' ) + '.txt'

    # If directory does not exists, then create one
    if not os.path.exists( dir_name ):
        os.mkdir( dir_name )

    sys.stdout = Logger( file_name )

    listener.listen( )
