#!/usr/bin/python3

# =========================================================== #
# [Author] Moses C. Nah
# [Email]  mosesnah@mit.edu
# [Date]   2022.01.26
# =========================================================== #

import rospy
from moses_baxter.msg import my_msg

def talker():
    pub = rospy.Publisher( 'my_talk' , my_msg)
    rospy.init_node( 'my_talker', anonymous = True )
    r = rospy.Rate( 20 )

    msg = my_msg()
    msg.q0_L[ 0 ] = 0.1
    msg.q0_L[ 3 ] = 0.1

    while not rospy.is_shutdown():
        # rospy.loginfo( msg )

        pub.publish( msg )
        r.sleep()

if __name__ == '__main__':
   try:
       talker( )
   except rospy.ROSInterruptException: pass
