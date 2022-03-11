#!/usr/bin/python3


import cv2
import argparse
import pickle
import math
import numpy as np
import rospy

from my_constants import Constants as C

FINAL_LINE_COLOR   = ( 255, 255, 255 )
WORKING_LINE_COLOR = ( 127, 127, 127 )


class Camera( object ):

    def __init__( self, args ):

        self.args = args
        self.vid  = cv2.VideoCapture( 0 )   # Turn on webcam
        self.img  = None                    # The current image of the main window that we often refer to

        if args.calibrated:     # [2022.03.10] We can calibrate the camera to take off the fish-bowl distortion effect.
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

    def _get_img( self, scl = 0.5, mode = "hsv" ):

        assert mode in [ "rgb", "hsv", "gray" ]

        grabbed = False

        if not grabbed:
            ( grabbed, img ) = self.vid.read( )

        img = cv2.resize( img, ( 0,0 ), fx = scl, fy = scl )

        if   mode == "gray":
            img = cv2.cvtColor( img, cv2.COLOR_BGR2GRAY )

        elif mode == "hsv":
            img = cv2.cvtColor( img, cv2.COLOR_BGR2HSV  )

        # If rgb, then do nothing

        return img

    def _mouse_get_pos_and_color( self, event, x, y, buttons, pars ):

        if event == cv2.EVENT_LBUTTONDOWN:
            print( "[Clicked Pixel] ({0:d},{1:d}), with Color {2:}".format( x, y, self.img[ y, x ] ) )

    def _get_img_size( self, img ):
        return ( img.shape[ : 2 ] )

    def _get_pos_and_color( self, mode = "hsv", scl = 0.5 ):

        self.img = self._get_img( scl = scl, mode = mode )
        window_name = mode + "_image"

        cv2.namedWindow( window_name )
        cv2.setMouseCallback( window_name, self._mouse_get_pos_and_color )

        while True:

            self.img = self._get_img( scl = scl, mode = mode )
            cv2.imshow( window_name, self.img )
            k = cv2.waitKey( 1 )


    def mask_color( self, img, lower_bound, upper_bound ):

        hsv = cv2.cvtColor( img, cv2.COLOR_BGR2HSV )

        lb = np.array( lower_bound )
        ub = np.array( upper_bound )

        mask       = cv2.inRange( hsv, lb, ub )                                 # Return the img in 0 or 255, which is simply a boolean
        img_masked = cv2.bitwise_and( img, img, mask = mask )                   # Simply masking and letting the "masked color" outputs in the frame
                                                                                # If the mask element is nonzero,

        return img_masked

    def find_polygons( self, img, n_edge ):

        contours, _ = cv2.findContours( img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )

        # [REF] https://docs.opencv.org/3.4/dc/dcf/tutorial_js_contour_features.html
        # 2nd argument is the number of edges
        poly_contour = [ ]

        for contour in contours:

            # 2nd argument is to check whether the contour is closed or not, True if closed.
            eps    = 0.1 * cv2.arcLength( contour, True )
            approx = cv2.approxPolyDP( contour, eps, True )

            # Only draw rectangles on the contour
            if len( approx ) == n_edge:
                poly_contour.append( approx )

        return poly_contour


    def detect_platform( self, mode = "debug" ):

        assert mode in [ "debug", "detect" ]

        # If mode is debug, simply run the loop forever
        # If mode is detect, sample and retrieve the average points of the platform
        tmp = []


        # This is our drawing loop, we just continuously draw new images and show them in the named window
        while True:

            img_raw  = self._get_img( scl = 0.5, mode = "rgb" )
            img_hsv  = cv2.cvtColor( img_raw, cv2.COLOR_BGR2HSV  )
            gray     = cv2.cvtColor( img_raw, cv2.COLOR_BGR2GRAY )

            # ======================================================================== #
            # ======================== [STEP 1] DENOISE PROCESS ====================== #
            # ======================================================================== #
            # First the noise should be cleaned-up
            img_denoised = cv2.fastNlMeansDenoisingColored( img_hsv, None, 20, 10, 7, 21 )                      # [REF] https://www.bogotobogo.com/python/OpenCV_Python/python_opencv3_Image_Non-local_Means_Denoising_Algorithm_Noise_Reduction.php
            img_filtered = cv2.inRange( img_denoised, np.array( [100, 20, 40] ), np.array( [120, 90, 80] ) )    # The range was discovered manually _get_pos_and_color( ) method

            # If you want to check the denoised image, uncomment the following line
            # cv2.imshow( "denoised_filtered", img_filtered )

            # ======================================================================== #
            # ======================== [STEP 2] MORPH PROCESS  ======================= #
            # ======================================================================== #
            kernel    = cv2.getStructuringElement( cv2.MORPH_ELLIPSE, ( 9, 9 ) )
            img_morph = cv2.morphologyEx( img_filtered, cv2.MORPH_DILATE, kernel )      # DILATION is useful to take-off noises
                                                                                        # [REF] https://docs.opencv.org/3.4/db/df6/tutorial_erosion_dilatation.html
            # If you want to check out the image after morph process, uncomment the following line
            # cv2.imshow( "morph", img_morph )

            # ======================================================================== #
            # ======================== [STEP 3] FIND CONTOURS  ======================= #
            # ======================================================================== #
            contour = self.find_polygons( img_morph, n_edge = 4 )           # Find 4 (rectangles) in the image

            # Find the mazimum-size polygons within the contours
            # max_contour = self.find_max_polygon( contour )
            if contour:

                # Find the contour with maximum area
                sorted_contours = sorted( contour, key = cv2.contourArea, reverse = True)
                max_contour = sorted_contours[ 0 ]
                tmp.append( np.squeeze( max_contour ) )
                cv2.drawContours( img_raw, [ max_contour ] , -1, ( 0, 255, 0 ), 10 )


            cv2.imshow( "img_contoured", img_raw )

            if mode == "detect" and len( tmp ) > 10:

                platform_points = np.around( np.median( tmp, 0 ) )
                print( "platform points are as follows:", platform_points )

                return platform_points

            k = cv2.waitKey( 1 )

    # [2022.03.10] This code can be taken off since it is like a legacy code
    def draw_platform( self ):

        # Code for detecting the platform, either manually or via computer algorithm
        # We can do this via computer algorithm, but not today
        # [Moses C. Nah] [Should be improved]
        cv2.namedWindow( "platform" )
        cv2.setMouseCallback( "platform", self.on_mouse )

        self.done    = False        # Flag signalling we're done
        self.current = ( 0, 0 )     # Current position, so we can draw the line-in-progress
        self.points  = [ ]          # Empty List of points defining our polygon

        while( not self.done ):
            # This is our drawing loop, we just continuously draw new images and show them in the named window
            ( grabbed, img_platform ) = self.vid.read( )

            if not grabbed: continue

            img_platform = cv2.resize(   img_platform, ( 0,0 ), fx = self.scl, fy = self.scl )
            self.img_h, self.img_w = self.get_img_size( img_platform )

            if ( len( self.points ) > 0 ):
                cv2.polylines( img_platform, np.array( [self.points] ), False, FINAL_LINE_COLOR, 6)     # Draw all the current polygon segments
                cv2.line( img_platform, self.points[ -1 ], self.current, WORKING_LINE_COLOR, 6 )            # And  also show what the current segment would look like

            # Update the window
            cv2.imshow( "platform", img_platform )
            k = cv2.waitKey( 1 )                    # Wait for 1ms and get the key input

        # If drawing done, finish the drawing by filled image
        img_platform_masked = np.zeros( ( self.img_h, self.img_w ), np.uint8 )

        if ( len( self.points ) > 0):
            cv2.fillPoly( img_platform_masked, np.array( [ self.points ] ), FINAL_LINE_COLOR )

        cv2.imshow( "platform_masked", img_platform_masked )
        print( "points are ", self.points )
        k = cv2.waitKey( 0 )                    # Wait for 1ms and get the key input


    def run( self ):

        platform_points = self.detect_platform( mode = "detect" )
        print( platform_points )

        try:
            while True:

                ( grabbed, img_raw ) = self.vid.read( )                         # Capture the video frame-by-frame
                if not grabbed: continue

                # If frame grabbed, then continue
                img_raw = cv2.resize(   img_raw, ( 0,0 ), fx = self.scl, fy = self.scl )
                img_yellow_contour = img_raw.copy( )


                self.img_h, self.img_w = self.get_img_size( img_raw )
                img_platform_masked    = np.zeros( ( self.img_h, self.img_w ) , np.uint8 )

                if ( len( platform_points ) > 0):
                    cv2.fillPoly( img_platform_masked, np.array( [ platform_points ] ), ( 255, 0, 0 ) )

                # [Step #1] Masking out the lower and upper bound of the yellow
                # img_masked = self.mask_color( img_raw, C.COLOR_LOWER_BOUND_YELLOW, C.COLOR_UPPER_BOUND_YELLOW )
                img_masked = self.mask_color( img_raw, C.COLOR_LOWER_BOUND_YELLOW, C.COLOR_UPPER_BOUND_YELLOW )

                # [Step #2] Changing again the figure to gray scale and thresholding to clean up
                img_tmp = cv2.cvtColor( img_masked, cv2.COLOR_BGR2GRAY )
                thresh  = cv2.threshold( img_tmp, 100, 255, cv2.THRESH_BINARY)[ 1 ]  # threshold

                # [Step #3] Find the polygon for the fill
                #           Before that, fill in the empty dots before drawing the contou
                kernel = cv2.getStructuringElement( cv2.MORPH_ELLIPSE, ( 35,35 ) )
                morph  = cv2.morphologyEx( thresh, cv2.MORPH_CLOSE, kernel )

                contours, hierarchy = cv2.findContours( thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )
                cv2.drawContours( img_yellow_contour, contours, -1, ( 0,255,0 ), 3 )

                size_elements = 0
                for cnt in contours:
                    size_elements += cv2.contourArea( cnt )

                # [Step #4] Fill the polygon
                img_filled = cv2.fillPoly( np.zeros( ( self.img_h, self.img_w ) ), pts = contours, color = ( 255, 255, 255 ) )

                # [Step #5] Draw all the images
                cv2.imshow( "main"           , img_raw            )
                # cv2.imshow( "yellow_masked"  , img_masked         )
                cv2.imshow( "yellow_contour" , img_yellow_contour )


                tmp1 = ( img_platform_masked == 255 )
                tmp2 = (          img_filled == 255 )

                tmp_overlap = np.logical_and( tmp1, tmp2 )

                over_lap = 255 * tmp_overlap.astype( np.uint8 )
                over_lap = cv2.cvtColor( over_lap, cv2.COLOR_GRAY2RGB )

                b, g, r = cv2.split( over_lap )

                zeros_ch = np.zeros( ( self.img_h, self.img_w ), dtype="uint8")
                blue_img = cv2.merge( [b, zeros_ch, zeros_ch] )
                # cv2.imshow("Blue Image", blue_img)
                tmpt = cv2.addWeighted( blue_img, 0.7, img_raw, 0.3, 0)

                cv2.imshow( "filled2"        , tmpt        )

                N  = np.sum( tmp1  )
                N1 = np.sum( np.logical_and( tmp1, tmp2  ) )

                cov_val = float( N1/N * 100 )

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
    parser.add_argument("-s", "--is_draw_platform"  , help = "mouse run to draw the target platform" , action = 'store_true')
    parser.add_argument(      "--scale"             , help = "The scale of the whole image"            , type = float, default = 0.4)


    args   = parser.parse_args()
    my_cam = Camera( args )
    my_cam.run(  )
