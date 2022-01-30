import cv2
import argparse
import pickle
import math
import numpy as np

parser = argparse.ArgumentParser()

parser.add_argument("-c", "--calibrated", help = "turn on calibration", action = 'store_true')
args   = parser.parse_args()

vid = cv2.VideoCapture(0)

if args.calibrated:
    f_name = "cam_vars.pkl"
    with open( f_name, 'rb' ) as file_object:
        raw_data = file_object.read( )

    raw_data = pickle.loads( raw_data ) # deserialization
    mtx, dist, optimal_camera_matrix, roi = raw_data

while( True ):

    # Capture the video frame-by-frame
    (grabbed, frame) = vid.read()

    if not grabbed:
        break

    if args.calibrated:
        # Undistort the image
        undistorted_image = cv2.undistort( frame, mtx, dist, None, optimal_camera_matrix )

        # Crop the image. Uncomment these two lines to remove black lines
        # on the edge of the undistorted image.
        # x, y, w, h  = roi
        # frame       = undistorted_image[y:y+h, x:x+w]                           # Rewriting the frame
        cv2.imshow('frame', undistorted_image)                                                  # Display the resulting frame

    else:

        # Resizing the image to half
        frame = cv2.resize( frame, (0,0), fx = 0.4, fy = 0.4)
        hsv   = cv2.cvtColor( frame, cv2.COLOR_BGR2HSV)

        hh, ww = frame.shape[:2]


        # The best upper-lower-bound
        lower_yellow = np.array([0, 80, 0])
        upper_yellow = np.array([45, 255, 255])


        # Here we are defining range of bluecolor in HSV
        # This creates a mask of blue coloured
        # objects found in the frame.
        mask          = cv2.inRange( hsv, lower_yellow, upper_yellow )          # The output is the 2D matrix with zeros and ones
        frame_masked  = cv2.bitwise_and( frame, frame, mask = mask )            # Simply masking and letting the "yellow" outputs in the frame
                                                                                # frame_masked is (y, x, 3)


        # Changing the figure into gray scale and thresholding again
        frame_gray = cv2.cvtColor( frame_masked, cv2.COLOR_BGR2GRAY )   # convert img to grayscale
        thresh     = cv2.threshold( frame_gray, 100, 255, cv2.THRESH_BINARY)[ 1 ]  # threshold

        # [REF]
        kernel = cv2.getStructuringElement( cv2.MORPH_ELLIPSE, (35,35))
        morph = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel )

        # contours = cv2.findContours( morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        frame_contour = cv2.drawContours( frame_masked, contours, -1, (0,255,0), 3)

        size_elements = 0
        for cnt in contours:
            size_elements += cv2.contourArea( cnt )
            # print( cv2.contourArea( cnt ) )



        # Fill the polygone
        img = np.zeros( ( hh, ww ) ) # create a single channel 200x200 pixel black image
        frame_poly = cv2.fillPoly( img, pts =contours, color=(255,255,255))

        # contours = contours[0] if len(contours) == 2 else contours[1]
        # for cntr in contours:
        #     cv2.drawContours( morph, [ cntr ] , 0, ( 0, 255, 0) , -1)

        cv2.imshow( "Images1", frame_masked )
        cv2.imshow( "Images2", thresh )
        cv2.imshow( "Images3", morph )
        cv2.imshow( "Images4", frame_contour )
        cv2.imshow( "Images5", frame_poly )

        print("rate of fullness : % ", (size_elements/(hh * ww ))*100)




    k = cv2.waitKey( 1 )                    # Wait for 1ms and get the key input

                                            # [BACKUP] ord('q'):
    if k%256 == 27:                         # If (ESC) key is given, stop the video
        print( "ESC inputted, Close Camera!" )
        break

vid.release()
cv2.destroyAllWindows()
