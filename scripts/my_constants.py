class Constants:
    PROJECT_NAME            = '[M3X] BAXTER Project'
    VERSION                 = '1.0.1'
    UPDATE_DATE             = '2022.04.06'
    AUTHOR_GITHUB           = 'mosesnah-shared'
    AUTHOR_FULL_NAME        = 'Moses C. Nah'
    URL                     = 'https://github.com/mosesnah-shared/moses_baxter',
    AUTHOR_EMAIL            = 'mosesnah@mit.edu', 'mosesnah@naver.com'

    # =============================================================== #
    # Constant variables for running the simulation


    # The directory which saves all the simulation results
    SAVE_DIR      = "/home/baxterplayground/ros_ws/src/newmanlab_code/moses_baxter/results/"

    # The constants used for the limb
    GRIP_MAX = 100
    GRIP_MIN = 0

    # The best upper-lower-bound
    COLOR_LOWER_BOUND_YELLOW = [  0,  80,   0 ]
    COLOR_UPPER_BOUND_YELLOW = [ 45, 255, 255 ]

    COLOR_LOWER_BOUND_RED   = [170, 70, 50]
    COLOR_UPPER_BOUND_RED   = [180,255,255]

    BASIC_JOINT_NAMES = [       "s0",       "s1",       "e0",       "e1",       "w0",       "w1",       "w2" ]
    LEFT_JOINT_NAMES  = [  "left_s0",  "left_s1",  "left_e0",  "left_e1",  "left_w0",  "left_w1",  "left_w2" ]
    RIGHT_JOINT_NAMES = [ "right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2" ]
    JOINT_NAMES       = { "left": LEFT_JOINT_NAMES, "right": RIGHT_JOINT_NAMES }

    RIGHT2LEFT       = { "right_s0": "left_s0" ,
                         "right_s1": "left_s1" ,
                         "right_e0": "left_e0" ,
                         "right_e1": "left_e1" ,
                         "right_w0": "left_w0" ,
                         "right_w1": "left_w1" ,
                         "right_w2": "left_w2" }

    LEFT2RIGHT       = { "left_s0": "right_s0" ,
                         "left_s1": "right_s1" ,
                         "left_e0": "right_e0" ,
                         "left_e1": "right_e1" ,
                         "left_w0": "right_w0" ,
                         "left_w1": "right_w1" ,
                         "left_w2": "right_w2" }


    RIGHT_JOINT_SIGN = { "right_s0": +1, "right_s1": +1, "right_e0": +1, "right_e1": +1, "right_w0": +1, "right_w1": +1, "right_w2": +1 }       # Whether it is plus or minus of the value of the joint
    LEFT_JOINT_SIGN  = {  "left_s0": -1,  "left_s1": +1,  "left_e0": -1,  "left_e1": +1,  "left_w0": -1,  "left_w1": +1,  "left_w2": -1 }
    JOINT_SIGNS      = {  "right": RIGHT_JOINT_SIGN , "left": LEFT_JOINT_SIGN   }

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

    REST_POSE  = {  'right_s0' : 0.7869321442,
                    'right_s1' : 0.0,
                    'right_e0' : -0.0149563127,
                    'right_e1' : 1.41,
                    'right_w0' : -0.0464029188,
                    'right_w1' :  0.0,
                    'right_w2' : -1.5823011827 }

    LIFT_POSE  = {  'right_s0' : 0.7869321442,
                    'right_s1' : -0.8413884622,
                    'right_e0' : -0.0149563127,
                    'right_e1' : 0.8617137076,
                    'right_w0' : -0.0464029188,
                    'right_w1' : 1.700,
                    'right_w2' : -1.4384904838 }

    GRASP_POSE = {  'right_s0' : 0.7869321442,
                    'right_s1' : 0.4045874328,
                    'right_e0' : -0.0149563127,
                    'right_e1' : 1.4116458201,
                    'right_w0' : -0.0464029188,
                    'right_w1' :  0.3879126465,
                    'right_w2' : -1.5823011827 }


    MID_POSE = {    'right_s0' : 0.7869321442,
                    'right_s1' : -0.3419854693,
                    'right_e0' : -0.0149563127,
                    'right_e1' : 0.3097428536,
                    'right_w0' : -0.0464029188,
                    'right_w1' : -0.759660457,
                    'right_w2' : -1.5823011827     }

    FINAL_POSE = {  'right_s0' : 0.7869321442,
                    'right_s1' : 0.4881893857,
                    'right_e0' : -0.0149563127,
                    'right_e1' : 0.7309418454,
                    'right_w0' : -0.0464029188,
                    'right_w1' : -0.2511893540,
                    'right_w2' : -1.5823011827     }

    # IMPEDANCE PARAMETERS
    # Case 1 - Stronger
    JOINT_IMP_Kq_R  = { 'right_s0': 10.0,
                        'right_s1': 15.0,
                        'right_e0':  5.0,
                        'right_e1': 20.0,
                        'right_w0':  3.0,
                        'right_w1': 12.0,
                        'right_w2':  1.5 }

    JOINT_IMP_Kq_L  = {  'left_s0': 10.0,
                         'left_s1': 15.0,
                         'left_e0':  5.0,
                         'left_e1': 20.0,
                         'left_w0':  3.0,
                         'left_w1': 12.0,
                         'left_w2':  1.5 }

    JOINT_IMP_Bq_R  = { 'right_s0':  5.0,
                        'right_s1': 10.0,
                        'right_e0':  2.5,
                        'right_e1':  7.5,
                        'right_w0':  0.1,
                        'right_w1':  1.0,
                        'right_w2':  1.5  }

    JOINT_IMP_Bq_L  ={   'left_s0':  5.0,
                         'left_s1': 10.0,
                         'left_e0':  2.5,
                         'left_e1':  7.5,
                         'left_w0':  0.1,
                         'left_w1':  1.0,
                         'left_w2':  1.5  }


    # # Case 1 - original 2022.01.28
    # Legacy Code
    # JOINT_IMP_Kq  = { 's0': 10.0,
    #                   's1': 15.0,
    #                   'e0':  5.0,
    #                   'e1': 15.0,
    #                   'w0':  3.0,
    #                   'w1':  3.0,
    #                   'w2':  1.5 }
    #
    # JOINT_IMP_Bq  = { 's0': 5.0,
    #                   's1': 7.5,
    #                   'e0': 2.5,
    #                   'e1': 7.5,
    #                   'w0': 0.1,
    #                   'w1': 1.5,
    #                   'w2': 1.5  }
