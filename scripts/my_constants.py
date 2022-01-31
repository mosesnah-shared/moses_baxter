class Constants:
    PROJECT_NAME            = '[M3X] BAXTER Project'
    VERSION                 = '1.0.1'
    UPDATE_DATE             = '2021.10.25'
    AUTHOR_GITHUB           = 'mosesnah-shared'
    AUTHOR_FULL_NAME        = 'Moses C. Nah'
    URL                     = 'https://github.com/mosesnah-shared/whip-project-targeting',
    AUTHOR_EMAIL            = 'mosesnah@mit.edu', 'mosesnah@naver.com'

    # =============================================================== #
    # Constant variables for running the simulation


    # The directory which saves all the simulation results
    SAVE_DIR      = "/home/baxterplayground/ros_ws/src/newmanlab_code/moses_baxter/results/"

    # The constants used for the limb
    RIGHT = 0       # This is at the same time, an index for the list
    LEFT  = 1       # This is at the same time, an index for the list
    BOTH  = 2

    GRIP_MAX = 100
    GRIP_MIN = 0

    LIMB_NAMES = [ "right", "left" ]


    # The best upper-lower-bound
    COLOR_LOWER_BOUND_YELLOW = [  0,  80,   0 ] 
    COLOR_UPPER_BOUND_YELLOW = [ 45, 255, 255 ] 

    LEFT_JOINT_NAMES  = [  "left_s0",  "left_s1",  "left_e0",  "left_e1",  "left_w0",  "left_w1",  "left_w2" ]
    RIGHT_JOINT_NAMES = [ "right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2" ]
    JOINT_NAMES       = [       "s0",       "s1",       "e0",       "e1",       "w0",       "w1",       "w2" ]
    JOINT_TO_FLIP     = [       "s0",                   "e0",                   "w0",                   "w2" ]

    # All the values are in "right" standard, "left" should add minus signs
    # [Moses C. Nah] Assuming that the movement is 2D planar, the following joint values give you the sign info
    #               joint s1: (shoulder flex/extension) for the right arm, the value is negative as you ``lift'' up
    #                                                   0   rad is when the upper limb is parallel to ground
    #                                                   -.8 rad is the lower limit, i.e., maximum lift up
    #               joint e1: (elbow    flex/extension) for the right arm, the value goes from 1.5 ~ 0, where 0 is the lower limit
    #                                                   1.5 rad (pi/2) when  it sees down, 0 rad for full extension
    #               joint w1: (wrist    flex/extension) for the right arm, the value goes from 1.5 - -1.5 very wide compared to others
    #                                                   1.5 rad for seeing down, -1.5 for seeing up
    #               Note that these joint correspond to "not flip joints"

    GRASP_POSE = {  's0' : 0.7869321442,
                    's1' : 0.4045874328,
                    'e0' : -0.0149563127,
                    'e1' : 1.4116458201,
                    'w0' : -0.0464029188,
                    'w1' :  0.3879126465,
                    'w2' : -1.5823011827 }

    MID_POSE = {    's0' : 0.7869321442,
                    's1' : -0.3419854693,
                    'e0' : -0.0149563127,
                    'e1' : 0.3097428536,
                    'w0' : -0.0464029188,
                    'w1' : -0.759660457,
                    'w2' : -1.5823011827     }

    FINAL_POSE = {    's0' : 0.7869321442,
                    's1' :  0.0000000000,
                    'e0' : -0.0149563127,
                    'e1' : 0.7097428536,
                    'w0' : -0.0464029188,
                    'w1' :  0.389660457,
                    'w2' : -1.5823011827     }

    # IMPEDANCE PARAMETERS
    # # Case 1 - original 2022.01.28
    JOINT_IMP1_Kq = { 's0': 10.0,
                      's1': 15.0,
                      'e0':  5.0,
                      'e1': 15.0,
                      'w0':  3.0,
                      'w1':  3.0,
                      'w2':  1.5 }


    JOINT_IMP1_Bq = { 's0': 5.0,
                      's1': 7.5,
                      'e0': 2.5,
                      'e1': 7.5,
                      'w0': 0.1,
                      'w1': 1.5,
                      'w2': 1.5  }

    # # Case 2 - Stronger
    JOINT_IMP2_Kq = { 's0': 10.0,
                      's1': 15.0,
                      'e0':  5.0,
                      'e1': 20.0,
                      'w0':  3.0,
                      'w1': 12.0,
                      'w2':  1.5 }


    JOINT_IMP2_Bq = { 's0': 5.0,
                      's1': 10.0,
                      'e0': 2.5,
                      'e1': 7.5,
                      'w0': 0.1,
                      'w1': 4.0,
                      'w2': 1.5  }
