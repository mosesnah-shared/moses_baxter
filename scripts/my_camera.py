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


    def detect_platform( self ):
        # This is our drawing loop, we just continuously draw new images and show them in the named window
        while True:

            ( grabbed, img_raw ) = self.vid.read( )

            if not grabbed: continue


            # img_raw  = cv2.resize(   img_raw, ( 0,0 ), fx = self.scl, fy = self.scl )
            img_raw  = cv2.resize(   img_raw, ( 0,0 ), fx = self.scl, fy = self.scl )

            img_hsv  = cv2.cvtColor( img_raw, cv2.COLOR_BGR2HSV  )
            gray     = cv2.cvtColor( img_raw, cv2.COLOR_BGR2GRAY )

            # This is the rough upper/lower bound of the platform found via "get_color" method
            dst = cv2.fastNlMeansDenoisingColored( img_hsv,None, 20,10,7,21 )

            cv2.imshow( "denoised", dst )



            img_filtered = cv2.inRange( dst, np.array( [100, 20, 40] ), np.array( [120, 90, 80] ))

            kernel = cv2.getStructuringElement( cv2.MORPH_RECT, ( 5,5 ) )
            morph  = cv2.morphologyEx( img_filtered, cv2.MORPH_DILATE, kernel )
#
            # kernel = cv2.getStructuringElement( cv2.MORPH_ELLIPSE, ( 9,9 ) )
            # morph  = cv2.morphologyEx( morph, cv2.MORPH_OPEN , kernel )

            #
            kernel = cv2.getStructuringElement( cv2.MORPH_ELLIPSE, ( 9,9 ) )
            morph  = cv2.morphologyEx( morph, cv2.MORPH_CLOSE , kernel )

            kernel = cv2.getStructuringElement( cv2.MORPH_ELLIPSE, ( 15,15 ) )
            morph  = cv2.morphologyEx( morph, cv2.MORPH_CLOSE , kernel)

            kernel = cv2.getStructuringElement( cv2.MORPH_ELLIPSE, ( 19,19 ) )
            morph  = cv2.morphologyEx( morph, cv2.MORPH_CLOSE , kernel )

            kernel = cv2.getStructuringElement( cv2.MORPH_RECT, ( 19,19 ) )
            morph  = cv2.morphologyEx( morph, cv2.MORPH_CLOSE , kernel )

            contours, hierarchy = cv2.findContours( morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1 )
            cv2.drawContours( img_raw, contours, -1, ( 0,255,0 ), 3 )

            # [REF] https://www.tutorialspoint.com/opencv/opencv_adaptive_threshold.htm
            # # thresh = cv2.adaptiveThreshold( gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 21, 10)
            # gray   = cv2.medianBlur( gray, 5 )
            # thresh = cv2.adaptiveThreshold( gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 31, 10)
            #
            # cv2.imshow( "thresh" , thresh )

            cv2.imshow( "filtered", img_filtered )
            cv2.imshow( "filtered2", morph )
            cv2.imshow( "contour", img_raw )
            #
            #
            #     # [Step #3] Find the polygon for the fill
            #     #           Before that, fill in the empty dots before drawing the contou


            #
            #
            # contours, hierarchy = cv2.findContours( morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )
            # cv2.drawContours( img_raw, contours, -1, ( 0,255,0 ), 3 )
            #
            # cv2.imshow( "morph"  , morph   )
            # cv2.imshow( "contour", img_raw )

            # gray[ gray >= 220 ] = 0

            # gray_blurred = cv2.medianBlur( gray, 5 )
            # masked   = cv2.bitwise_and( gray, gray, mask = glare )




            # print( np.unique( masked ) )
            # canny = cv2.Canny( gray_blurred, 0, 120, 1 )
            # cv2.imshow( "edge"  , canny )

            # Dilation
            # kernel     = np.ones( ( 30,30 ), np.uint8 )
            # img_closed = cv2.morphologyEx( gray, cv2.MORPH_CLOSE, kernel)


            # se = cv2.getStructuringElement( cv2.MORPH_RECT , (8,8) )
            # gray = cv2.morphologyEx( gray, cv2.MORPH_DILATE, se)
            #
            # blurred = cv2.medianBlur(gray, 5)
            # canny = cv2.Canny(blurred, 120, 255, 1)
            #
            # clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            # cl1 = clahe.apply( gray )

            # cnts = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # cnts = cnts[0] if len(cnts) == 2 else cnts[1]
            #
            # # Iterate thorugh contours and draw rectangles around contours
            # for c in cnts:
            #     x,y,w,h = cv2.boundingRect(c)
            #     cv2.rectangle( img_raw, (x, y), (x + w, y + h), (36,255,12), 2)

            # cv2.imshow('img_raw'  , img_raw      )
            # cv2.imshow('img_hsv'  , img_hsv      )
            # cv2.imshow('img_gray' , cl1         )
            # cv2.imshow('img_dil'  , img_closed )

            # Usually, glares are near 255 values


            # print( np.max( glare ) )
            # Changin the values to


            # cv2.imshow('image', img_raw)
            #
            #
            # cv2.imshow( "test1", img_raw )
            # # cv2.imshow( "test3", cl1 )              # https://docs.opencv.org/3.1.0/d5/daf/tutorial_py_histogram_equalization.html
            # cv2.imshow( "maksed", masked  )
            #
            # cv2.imshow( "glare", glare    )


            k = cv2.waitKey( 1 )

            if k%256 == 27:         # ESC Pressed
                print("Escape hit, closing...")
                break
            elif k == ord('a'):
                cv2.imwrite( "tmp1.jpg", img_raw )
                cv2.imwrite( "tmp2.jpg", thresh  )



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


    def run( self, platform_points = ( ( 342,130 ), ( 325, 351 ), ( 642, 335 ), ( 585, 118 )  ) ):

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

    # my_cam.detect_platform( )
    my_cam._get_pos_and_color( mode = "rgb", scl = 0.8 )
    exit( )



    if args.is_draw_platform:
        my_cam.draw_platform( )

    else:

        points = [ (409, 164), (401, 367), (673, 358), (625, 166) ]

        # points =  [(337, 167), (318, 368), (603, 363), (563, 164)]
        my_cam.run( platform_points = points )
