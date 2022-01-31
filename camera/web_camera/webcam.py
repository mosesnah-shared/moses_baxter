import cv2
import argparse
import pickle
import math
import numpy as np
import rospy

from my_constants import Constants as C


class Camera( object ):

    def __init__( self, args ):

        self.args = args
        self.vid  = cv2.VideoCapture( 0 )   # Turn on webcam


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

    def detect_platform( self ):
        # Code for detecting the platform, either manually or via computer algorith,
        NotImplementedError( )

    def run( self ):

        try:
            while True:

                # Capture the video frame-by-frame
                ( grabbed, img_raw ) = self.vid.read( )

                if not grabbed: # check whether the frame was grabbed
                    continue

                # If frame grabbed, then continue
                scl     = 0.4                                                   # The scale of the image
                img_raw = cv2.resize(   img_raw, ( 0,0 ), fx = scl, fy = scl )
                hsv     = cv2.cvtColor( img_raw, cv2.COLOR_BGR2HSV)

                self.img_h, self.img_w = frame.shape[ : 2 ]

                # [Step #1] Masking out the lower and upper bound of the yellow
                mask               = cv2.inRange( hsv, C.COLOR_LOWER_BOUND_YELLOW, C.COLOR_UPPER_BOUND_YELLOW )     # The output is the 2D matrix with zeros and ones
                img_yellow_masked  = cv2.bitwise_and( frame, frame, mask = mask )                                   # Simply masking and letting the "yellow" outputs in the frame

                # [Step #2] Changing again the figure to gray scale and thresholding to clean up
                img_tmp = cv2.cvtColor( img_yellow_masked, cv2.COLOR_BGR2GRAY )   # convert img to grayscale
                thresh  = cv2.threshold( img_tmp, 100, 255, cv2.THRESH_BINARY)[ 1 ]  # threshold

                # [Step #3] Find the polygon for the fill
                kernel = cv2.getStructuringElement( cv2.MORPH_ELLIPSE, ( 35,35 ) )
                morph  = cv2.morphologyEx( thresh, cv2.MORPH_CLOSE, kernel )

                contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                img_contour = cv2.drawContours( img_yellow_masked, contours, -1, (0,255,0), 3)

                size_elements = 0
                for cnt in contours:
                    size_elements += cv2.contourArea( cnt )

                # [Step #4] Fill the polygon
                img = np.zeros( ( hh, ww ) ) # create a single channel 200x200 pixel black image
                img_filled = cv2.fillPoly( img, pts = contours, color = ( 255, 255, 255 ) )

                # [Step #5] Draw all the images
                cv2.imshow( "raw and masked",     cv2.hconcat( [ img_raw, img_yellow_masked ] )  )
                cv2.imshow( "contour and filled", cv2.hconcat( [ img_contour, img_filled    ] )  )

                cov_val = (size_elements/(hh * ww ))*100
                print("rate of fullness : % ", cov_val )

                rospy.set_param( 'my_obj_func', cov_val )


                k = cv2.waitKey( 1 )                    # Wait for 1ms and get the key input


        except KeyboardInterrupt:
            print( 'interrupted via Ctrl-c, halting' )

            vid.release()
            cv2.destroyAllWindows()



if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("-c", "--calibrated",         help = "turn on calibration"                     , action = 'store_true')
    parser.add_argument("-s", "--is_platform_detect", help = "mouse run to detect the target platform" , action = 'store_true')

    args   = parser.parse_args()
