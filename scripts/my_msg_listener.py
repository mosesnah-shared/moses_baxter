#!/usr/bin/python3

# =========================================================== #
# [Author] Moses C. Nah
# [Email]  mosesnah@mit.edu
# [Date]   2022.01.26
# =========================================================== #

import rospy
from moses_baxter.msg import my_msg

def callback( data ):
    rospy.loginfo( "A:%d B:%d C:%d" % ( data.A, data.B, data.C ) )

def listener():
    rospy.init_node(  'my_listener', anonymous = True)
    rospy.Subscriber( "my_talk" , my_msg, callback )

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
