#!/usr/bin/python3

# =========================================================== #
# [Author] Moses C. Nah
# [Email]  mosesnah@mit.edu
# [Date]   2021.10.26
# [Description]
#   - The reading the joint states via ros
# 	- You can actually type `rostopic echo /robot/joint_states' at the command line, but this script "pythonize" the method
#	- [REF] https://books.google.com/books?id=g3JQDwAAQBAJ&pg=PA416&lpg=PA416&dq=baxter+robot_state_publisher&source=bl&ots=8-zTxhpDy5&sig=ACfU3U073scB5DMan27ujZiV-54ZTrRyBw&hl=en&sa=X&ved=2ahUKEwj5no6r7ujzAhVjn-AKHUgRB9YQ6AF6BAgVEAM#v=onepage&q=baxter%20robot_state_publisher&f=false
#   - [REF] https://sdk.rethinkrobotics.com/wiki/API_Reference#Arm_Joints
# =========================================================== #

import rospy
import time
import baxter_interface
import numpy        as np
import std_msgs.msg as msg
import threading


from   std_msgs.msg       import UInt16, Empty, String
from   baxter_interface   import CHECK_VERSION
from   sensor_msgs.msg    import JointState


# Local Library, under moses/scripts
from my_constants import Constants as C


def callback(data):
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


    # print( data.position )
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def joint_state_listener():

    rospy.init_node( 'my_listener', anonymous = False )
    rospy.Subscriber( "robot/joint_states", JointState, callback)


    # ctrl.joint_state_listener( )
    # rospy.sleep(2)
    rospy.spin( )


if __name__ == '__main__':
    joint_state_listener()
