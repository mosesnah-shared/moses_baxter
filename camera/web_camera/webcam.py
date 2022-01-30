import cv2
import argparse
import pickle
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

        # The best upper-lower-bound
        lower_yellow = np.array([0, 80, 0])
        upper_yellow = np.array([45, 255, 255])

        # Here we are defining range of bluecolor in HSV
        # This creates a mask of blue coloured
        # objects found in the frame.
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # cv2.imshow( 'raw_mask', mask )
        # The bitwise and of the frame and mask is done so
        # that only the blue coloured objects are highlighted
        # and stored in res
        frame_masked = cv2.bitwise_and( frame, frame, mask = mask )



        # blur = cv2.GaussianBlur(final_result, (7, 7), 0)

        # contours, hierarchy = cv2.findContours( mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # output = cv2.drawContours( res, contours, -1, (0, 0, 255), 3)
        #
        # # cv2.imshow('frame',frame)
        # cv2.imshow('res'     ,res  )
        # cv2.imshow('contour' ,output )


        # remove noises of the vision data
        # [REF] https://techvidvan.com/tutorials/detect-objects-of-similar-color-using-opencv-in-python/
        cv2.imshow( "Images", np.hstack( [ frame, frame_masked ] ) )





    k = cv2.waitKey( 1 )                    # Wait for 1ms and get the key input

                                            # [BACKUP] ord('q'):
    if k%256 == 27:                         # If (ESC) key is given, stop the video
        print( "ESC inputted, Close Camera!" )
        break

vid.release()
cv2.destroyAllWindows()
