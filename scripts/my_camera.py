#!/usr/bin/python3

# First check whether the head_camera is opened.
# Note that if we want to open head_camera, we need to first close one of the hand cameras.
# >> rosrun baxter_tools camera_control.py -c right_hand_camera
# >> rosrun baxter_tools camera_control.py -c left_hand_camera
# [REF] https://web.ics.purdue.edu/~rvoyles/Classes/ROSprogramming/Using_Baxter_Cameras.pdf
# >> rosrun baxter_tools camera_control.py -l       # Check whether the heaD_camera is opened
# >> rosrun baxter_tools camera_control.py -o head_camera -r 1280x800

# Connecting ros with cv2

import rospy
import cv2
import cv_bridge
import baxter_interface

from sensor_msgs.msg  import Image
from baxter_interface import CHECK_VERSION

bridge = cv_bridge.CvBridge( ) # Initialize CV Bridge object

# Callback function to subscribe to images
def image_callback( ros_img ):
    # Convert received image message to OpenCv image
    cv_image = bridge.imgmsg_to_cv2( ros_img, desired_encoding="passthrough")
    cv2.imshow( 'Image', cv_image ) # display image
    cv2.waitKey( 1 )

if __name__ == '__main__':
    rospy.init_node( 'my_camera_subscriber', anonymous = True ) # Initialze ROS node

    # Subscribe to head_camera image topic
    # rospy.Subscriber('/cameras/head_camera/image', Image, image_callback)
    rospy.Subscriber('/cameras/right_hand_camera/image', Image, image_callback)


    # Moving baxter to middle posture might be useful.
    # head = baxter_interface.Head( )
    # rs   = baxter_interface.RobotEnable( CHECK_VERSION )
    # rs.enable()
    # head.set_pan( 0 )   # Neutral Position
    # Doesn't look bad


    rospy.spin() # sleep
    cv2.destroyAllWindows() # Destroy CV image window on shut_down
