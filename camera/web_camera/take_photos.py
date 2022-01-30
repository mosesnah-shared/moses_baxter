# [Last update] [Moses C. Nah] 2021.06.29
# [Code from Following]
# [REF] https://stackoverflow.com/questions/34588464/python-how-to-capture-image-from-webcam-on-click-using-opencv

import cv2
import time
import argparse

from datetime import date


# [Backup] [Moses C. Nah] 2021.06.29
# In case if you want to save the date for the photos
#today   = date.today()
#date    = today.strftime("%d_%m_%H%M%S")


                                # (1) return: True/False
                                # (2)  frame: (width x height x 3) BGR value of the screen

def take_photos( args ):

    cam = cv2.VideoCapture( 0 )     # Call webcam
    img_cnt = 0


    if args.keyboard: # If keyboard mode
        while True:

            ret, frame = cam.read( )        # cam.read( ) returns
                                            # (1) return: True/False
                                            # (2)  frame: (width x height x 3) BGR value of the screen
            if not ret:                     # If the return isn't successful
                print("failed to grab frame")
                break

            cv2.imshow( "take_photo", frame )

            k = cv2.waitKey( 1 )            # Display (or wait) the image for 1ms and then move forward

            if k%256 == 27:                 # ESC (ASCII #27) pressed
                print( "Escape hit, closing..." )
                break

            elif k%256 == 32:               # SPACE (ASCII #32) pressed
                img_name = "pic_{}.png".format( img_cnt )
                cv2.imwrite( img_name, frame )
                print( "{} written!".format( img_name ) )
                img_cnt += 1

    else:      # If auto mode
        ti    = time.time( )                  # current time
        i     = 1
        Nmax  = 2                  # The maximum image numbers

        while i <= Nmax:

            ret, frame = cam.read( )                            # cam.read( ) returns
            cv2.imshow( 'frame', frame )                        # Display the resulting frame
            t = ( time.time( ) - ti )

            if t >= 5 + 5 * i: # Every 5 second, 10 seconds for the beginning

                print( "{:10.4f}, picture taken".format( t ) )
                img_name = "pic_{0:}.png".format( i )
                cv2.imwrite( img_name, frame )
                i += 1

            cv2.waitKey( 1 )

    cam.release( )
    cv2.destroyAllWindows( )

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("-k", "--keyboard", help = "turn on calibration", action = 'store_true')
    args   = parser.parse_args()

    take_photos( args )
