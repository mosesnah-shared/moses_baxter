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
class CameraCalibrate( object ):

    def __init__( self ):
        self.rate  = 100.0   # Hz
        self.Nmax  = 10     # Maximum 10 photos

        # Subscribe to head_camera image topic
        rospy.init_node( 'my_camera_subscriber', anonymous = True ) # Initialze ROS node
        rospy.Subscriber('/cameras/head_camera/image', Image, self.image_callback )


    def image_callback( self, ros_img ):
        # Convert received image message to OpenCv image
        self.cv_image = bridge.imgmsg_to_cv2( ros_img, desired_encoding="passthrough" )

        # crop the image, since the camera is sort of broken (REF: "baxter_camera/head_camera_images/why_crop"
        # The right part of the image should be cropped, the best pixel width number was 900
        self.cv_image_cropped = self.cv_image[0:-1, 0:950]
        cv2.imshow( 'Image', self.cv_image_cropped ) # display image

        cv2.waitKey( 1 )

    def run( self ):


        # rospy.Subscriber('/cameras/right_hand_camera/image', Image, image_callback)

        # Moving baxter to middle posture might be useful.
        # head = baxter_interface.Head( )
        # rs   = baxter_interface.RobotEnable( CHECK_VERSION )
        # rs.enable()
        # head.set_pan( 0 )   # Neutral Position


        # Doesn't look bad
        ti    = rospy.Time.now()   # initial time

        t     = 0                  # current time
        i     = 1

        control_rate = rospy.Rate( self.rate )             # set control rate


        while i <= self.Nmax:

            t = ( rospy.Time.now( ) - ti ).to_sec( )
            # cv2.imwrite( img_name, frame )

            if t >= 5 * i: # Every 5 second

                print( "{:10.4f}, picture taken".format( t ) )
                img_name = "pic_{0:}.png".format( i )
                cv2.imwrite( img_name, self.cv_image_cropped )
                i += 1

            control_rate.sleep( )

        cv2.destroyAllWindows() # Destroy CV image window on shut_down


if __name__ == '__main__':
    my_cam = CameraCalibrate( )
    my_cam.run( )
