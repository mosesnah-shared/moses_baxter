#!/usr/bin/python3


import cv2
import argparse
import pickle
import math
import numpy as np
import rospy

from my_constants import Constants as C

FINAL_LINE_COLOR = (255, 255, 255)
WORKING_LINE_COLOR = (127, 127, 127)

class Camera( object ):

    def __init__( self, args ):

        self.args = args
        self.vid  = cv2.VideoCapture( 0 )   # Turn on webcam


        self.done    = False        # Flag signalling we're done
        self.current = (0, 0)       # Current position, so we can draw the line-in-progress
        self.points  = []           # List of points defining our polygon

        self.scl     = 0.4                                                   # The scale of the image

        if args.calibrated:
            NotImplementedError( )
            # [Moses C. Nah] [BACKUP]
            # We've checked that calibration is not required yet
            # f_name = "cam_vars.pkl"
            # with open( f_name, 'rb' ) as file_object:
            #     raw_data = file_object.read( )
            #
            #     raw_data = pickle.loads( raw_data ) # deserialization
            #     mtx, dist, optimal_camera_matrix, roi = raw_data
            # Additional Backup
            # if args.calibrated:
            #     # Undistort the image
            #     undistorted_image = cv2.undistort( frame, mtx, dist, None, optimal_camera_matrix )
            #
            #     # Crop the image. Uncomment these two lines to remove black lines
            #     # on the edge of the undistorted image.
            #     # x, y, w, h  = roi
            #     # frame       = undistorted_image[y:y+h, x:x+w]                           # Rewriting the frame
            #     cv2.imshow('frame', undistorted_image)                                    # Display the resulting frame

    def on_mouse( self, event, x, y, buttons, user_param):


        # Mouse callback that gets called for every mouse event (i.e. moving, clicking, etc.)
        if self.done: # Nothing more to do
            return

        if event == cv2.EVENT_MOUSEMOVE:
            self.current = (x, y)

        elif event == cv2.EVENT_LBUTTONDOWN:
            print( "Adding point #%d with position(%d,%d)" % ( len( self.points ), x, y ) )
            self.points.append( (x, y) )

        elif event == cv2.EVENT_RBUTTONDOWN:
            print( "Completing polygon with %d points." % len(self.points))
            self.done = True

    def detect_platform( self ):

        # Code for detecting the platform, either manually or via computer algorithm
        # We can do this via computer algorithm, but not today
        # [Moses C. Nah] [Should be improved]
        cv2.namedWindow( "platform" )
        cv2.setMouseCallback( "platform", self.on_mouse )

        while( not self.done ):
            # This is our drawing loop, we just continuously draw new images and show them in the named window
            ( grabbed, img_platform ) = self.vid.read( )

            # If frame grabbed, then continue

            img_platform = cv2.resize(   img_platform, ( 0,0 ), fx = self.scl, fy = self.scl )
            self.img_h, self.img_w = img_platform.shape[ : 2 ]

            if ( len( self.points ) > 0 ):
                cv2.polylines( img_platform, np.array([self.points]), False, FINAL_LINE_COLOR, 1)                  # Draw all the current polygon segments
                cv2.line( img_platform, self.points[-1], self.current, WORKING_LINE_COLOR)                 # And  also show what the current segment would look like

            # Update the window
            cv2.imshow( "platform", img_platform )
            k = cv2.waitKey( 1 )                    # Wait for 1ms and get the key input


        # User finised entering the polygon points, so let's make the final drawing
        # canvas = np.zeros( ( self.img_h, self.img_w ), np.uint8)
        # of a filled polygon
        img_platform_masked = np.zeros( ( self.img_h, self.img_w ) , np.uint8 )

        if ( len( self.points ) > 0):
            cv2.fillPoly( img_platform_masked, np.array( [ self.points ] ), FINAL_LINE_COLOR )
            img_platform_masked = cv2.cvtColor( img_platform_masked, cv2.COLOR_BGR2GRAY )   # convert img to grayscale


        cv2.imshow( "platform_masked", img_platform_masked )
        k = cv2.waitKey( 0 )                    # Wait for 1ms and get the key input


    def run( self ):

        try:
            while True:

                # Capture the video frame-by-frame
                ( grabbed, img_raw ) = self.vid.read( )

                # If frame grabbed, then continue
                img_raw = cv2.resize(   img_raw, ( 0,0 ), fx = self.scl, fy = self.scl )
                hsv     = cv2.cvtColor( img_raw, cv2.COLOR_BGR2HSV)

                self.img_h, self.img_w = img_raw.shape[ : 2 ]

                if args.is_platform_detect:
                    self.detect_platform( )
                    exit( )
                else:
                    img_platform_masked = np.zeros( ( self.img_h, self.img_w ) , np.uint8 )

                    if ( len( self.args.points ) > 0):
                        cv2.fillPoly( img_platform_masked, np.array( [ self.args.points ] ), FINAL_LINE_COLOR )

                    cv2.imshow( "platform_masked", img_platform_masked )

                if not grabbed: # check whether the frame was grabbed
                    continue




                # [Step #1] Masking out the lower and upper bound of the yellow
                mask               = cv2.inRange( hsv, np.array( C.COLOR_LOWER_BOUND_YELLOW ), np.array( C.COLOR_UPPER_BOUND_YELLOW ) )     # The output is the 2D matrix with zeros and ones
                img_yellow_masked  = cv2.bitwise_and( img_raw, img_raw, mask = mask )                                   # Simply masking and letting the "yellow" outputs in the frame

                # [Step #2] Changing again the figure to gray scale and thresholding to clean up
                img_tmp = cv2.cvtColor( img_yellow_masked, cv2.COLOR_BGR2GRAY )   # convert img to grayscale
                thresh  = cv2.threshold( img_tmp, 100, 255, cv2.THRESH_BINARY)[ 1 ]  # threshold

                # [Step #3] Find the polygon for the fill
                kernel = cv2.getStructuringElement( cv2.MORPH_ELLIPSE, ( 35,35 ) )
                morph  = cv2.morphologyEx( thresh, cv2.MORPH_CLOSE, kernel )

                contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours( img_yellow_masked, contours, -1, (0,255,0), 3)

                size_elements = 0
                for cnt in contours:
                    size_elements += cv2.contourArea( cnt )

                # [Step #4] Fill the polygon
                img_filled = cv2.fillPoly( np.zeros( ( self.img_h, self.img_w ) ), pts = contours, color = ( 255, 255, 255 ) )


                # [Step #5] Draw all the images
                cv2.imshow( "main"          , img_raw           )
                cv2.imshow( "contour"       , img_yellow_masked )
                cv2.imshow( "filled"        , img_filled        )


                tmp1 = ( img_platform_masked == 255 )
                tmp2 = ( img_filled == 255 )

                N  = np.sum( tmp1  )
                N1 = np.sum( np.logical_and( tmp1, tmp2  ) )

                print( "filled region is ", N1/N * 100 )

                cov_val = ( size_elements / ( self.img_h * self.img_w ) )*100
                print("rate of fullness : % ", cov_val )

                rospy.set_param( 'my_obj_func', cov_val )


                k = cv2.waitKey( 1 )                    # Wait for 1ms and get the key input


        except KeyboardInterrupt:
            print( 'interrupted via Ctrl-c, halting' )

            self.vid.release()
            cv2.destroyAllWindows()



if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("-c", "--calibrated",         help = "turn on calibration"                     , action = 'store_true')
    parser.add_argument("-s", "--is_platform_detect", help = "mouse run to detect the target platform" , action = 'store_true')

    args   = parser.parse_args()

    if not args.is_platform_detect:
        args.points = ( ( 354, 74 ), (344, 300), ( 659, 279 ), ( 595, 59 )  )

    my_cam = Camera( args )
    my_cam.run( )
