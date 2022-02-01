#!/usr/bin/python3

# =========================================================== #
# [Author] Moses C. Nah
# [Email]  mosesnah@mit.edu
# [Date]   2022.01.26
# =========================================================== #

import time
import rospy
from   baxter_interface   import CHECK_VERSION
from   sensor_msgs.msg    import JointState
from   moses_baxter.msg   import my_msg

# Listener to print out the necessary data for the robot.
# Local Library, under moses/scripts
from my_constants import Constants as C


def callback1( data ):
    # For the JointState data, we have the following data
    # [1] position
    # [2] velocity
    # [3] effort
    # [4] name
    # [5]
    # [REF] https://stackoverflow.com/questions/57271100/how-to-feed-the-data-obtained-from-rospy-subscriber-data-into-a-variable
    # We are interested at the joint position data, hence check whether the given data are joint data
    # We will check this via finding a specific string
    if 'torso_t0' in data.name:
        for name in C.LEFT_JOINT_NAMES:

        	# Find index of the name
        	try:
        		idx = data.name.index( name )
        		rospy.loginfo( "name = %s,  pos    = %f", name, data.position[ idx ]  )
        		rospy.loginfo( "name = %s,  vel    = %f", name, data.velocity[ idx ]  )
        		rospy.loginfo( "name = %s,  torque = %f", name, data.effort[   idx ]  )


        	except:
        		NotImplementedError( )


        for name in C.RIGHT_JOINT_NAMES:
        	# Find index of the name
        	try:
        		idx = data.name.index( name )
        		rospy.loginfo( "name = %s,  pos    = %f", name, data.position[ idx ]  )
        		rospy.loginfo( "name = %s,  vel    = %f", name, data.velocity[ idx ]  )
        		rospy.loginfo( "name = %s,  torque = %f", name, data.effort[   idx ]  )

        	except:
        		NotImplementedError( )


def callback2( data ):
    rospy.loginfo( "A:%d B:%d C:%d" % ( data.A, data.B, data.C ) )


def joint_state_listener():

    rospy.init_node( 'my_listener', anonymous = False )
    rospy.Subscriber( "robot/joint_states", JointState, callback1 )
    rospy.Subscriber( "my_talk" , my_msg, callback2 )


    rospy.spin( )


if __name__ == '__main__':
    joint_state_listener()
