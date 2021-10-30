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
    SAVE_DIR      = "/home/baxterbuddy/ros_ws/src/newmanlab_code/moses/results/"

    # The constants used for the limb
    RIGHT = 0       # This is at the same time, an index for the list
    LEFT  = 1       # This is at the same time, an index for the list
    BOTH  = 2

    GRIP_MAX = 100
    GRIP_MIN = 0

    LIMB_NAMES = [ "right", "left" ]

    LEFT_JOINT_NAMES  = [  "left_s0",  "left_s1",  "left_e0",  "left_e1",  "left_w0",  "left_w1",  "left_w2" ]
    RIGHT_JOINT_NAMES = [ "right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2" ]

    GRASP_POSE_RIGHT = {
        'right_s0':  0.63621853177547540, 
        'right_s1': -0.30219421521342650, 
        'right_e0':  0.45099035163831164, 
        'right_e1':  1.78095169473496530,
        'right_w0': -0.03604854851530722, 
        'right_w1': -1.44922834935474460, 
        'right_w2': -2.00567988016017830 
    }


    GRASP_POSE_LEFT = {
        'left_s0': -0.63621853177547540, 
        'left_s1': -0.30219421521342650, 
        'left_e0': -0.45099035163831164, 
        'left_e1':  1.78095169473496530,
        'left_w0':  0.03604854851530722, 
        'left_w1': -1.44922834935474460, 
        'left_w2':  2.00567988016017830, 
    }


    GRASP_POSE_RIGHT_WIDER = {
        'right_s0':  0.6089903727905093, 
        'right_s1': -0.3451456772742181, 
        'right_e0':  0.3079466431679968, 
        'right_e1':  1.5964905049917444,
        'right_w0': -0.3305728597893067, 
        'right_w1': -1.0783884938834458, 
        'right_w2': -1.8177672336442152 
    }

    GRASP_POSE_LEFT_WIDER = {
        'left_s0': -0.6089903727905093, 
        'left_s1': -0.3451456772742181, 
        'left_e0': -0.3079466431679968, 
        'left_e1':  1.5964905049917444,
        'left_w0':  0.3305728597893067, 
        'left_w1': -1.0783884938834458, 
        'left_w2':  1.8177672336442152 
    }

    MID_POSE_RIGHT = {
        'right_s0':  0.70217970565454810, 
        'right_s1': -0.45137384683528300,
        'right_e0':  0.19711653124327566, 
        'right_e1':  0.34898062924393164, 
        'right_w0': -0.17755827619773665, 
        'right_w1':  0.16451943950071063, 
        'right_w2': -1.7732817907955383 }


    MID_POSE_LEFT = {
        'left_s0': -0.70217970565454810, 
        'left_s1': -0.45137384683528300,
        'left_e0': -0.19711653124327566, 
        'left_e1':  0.34898062924393164, 
        'left_w0':  0.17755827619773665, 
        'left_w1':  0.16451943950071063, 
        'left_w2':  1.7732817907955383 }

    WIDE_POSE_RIGHT = {
        'right_s0': -0.14112623248545805, 
        'right_s1': -0.22127672865247094,
        'right_e0':  0.20248546400087460, 
        'right_e1':  1.34453416058156500,         
        'right_w0': -0.09088836168221076, 
        'right_w1': -1.33149532388453900, 
        'right_w2': -1.73915071826508780 
    }

    WIDE_POSE_LEFT = {
         'left_s0':  0.14112623248545805, 
         'left_s1': -0.22127672865247094,
         'left_e0': -0.20248546400087460, 
         'left_e1':  1.34453416058156500,         
         'left_w0':  0.09088836168221076, 
         'left_w1': -1.33149532388453900, 
         'left_w2':  1.73915071826508780 
    }

    REST_POSE_RIGHT = {
        'right_s0':  0.6634466907604415, 
        'right_s1':  0.5150340495325276, 
        'right_e0':  0.42951462060791584, 
        'right_e1':  0.17755827619773665,  
        'right_w0': -0.16988837225830958, 
        'right_w1':  0.9894176081860918, 
        'right_w2': -2.0075973561450353, 
    }


    REST_POSE_LEFT = {
        'left_s0': -0.6634466907604415, 
        'left_s1':  0.5150340495325276, 
        'left_e0': -0.42951462060791584, 
        'left_e1':  0.17755827619773665,  
        'left_w0':  0.16988837225830958, 
        'left_w1':  0.9894176081860918, 
        'left_w2':  2.0075973561450353, 
    }

    # IMPEDANCE PARAMETERS
    # Case 1
    JOINT_IMP1_Kq = { 'right_s0': 10.0, 
                      'right_s1': 15.0, 
                      'right_e0':  5.0,  
                      'right_e1':  5.0,
                      'right_w0':  3.0, 
                      'right_w1':  3.0, 
                      'right_w2':  1.5 } 
  

    JOINT_IMP1_Bq = { 'right_s0': 2.0, 
                      'right_s1': 3.0, 
                      'right_e0': 1.0, 
                      'right_e1': 1.0,
                      'right_w0': 0.6, 
                      'right_w1': 0.6,
                      'right_w2': 0.3  } 
