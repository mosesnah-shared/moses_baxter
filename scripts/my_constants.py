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
    JOINT_NAMES       = [       "s0",       "s1",       "e0",       "e1",       "w0",       "w1",       "w2" ]
    JOINT_TO_FLIP     = [       "s0",                   "e0",                   "w0",                   "w2" ]

    # All the values are in "right" standard, "left" should add minus signs
    GRASP_POSE = {
                's0':  0.63621853177547540, 
                's1': -0.30219421521342650, 
                'e0':  0.45099035163831164, 
                'e1':  1.78095169473496530,
                'w0': -0.03604854851530722, 
                'w1': -1.44922834935474460, 
                'w2': -2.00567988016017830 
    }


    GRASP_POSE_WIDER = {
                's0':  0.6089903727905093, 
                's1': -0.3451456772742181, 
                'e0':  0.3079466431679968, 
                'e1':  1.5964905049917444,
                'w0': -0.3305728597893067, 
                'w1': -1.0783884938834458, 
                'w2': -1.8177672336442152 }


    MID_POSE =  {
                's0':  0.70217970565454810, 
                's1': -0.45137384683528300,
                'e0':  0.19711653124327566, 
                'e1':  0.34898062924393164, 
                'w0': -0.17755827619773665, 
                'w1':  0.16451943950071063, 
                'w2': -1.7732817907955383 }

    WIDE_POSE = {
                's0': -0.14112623248545805, 
                's1': -0.22127672865247094,
                'e0':  0.20248546400087460, 
                'e1':  1.34453416058156500,         
                'w0': -0.09088836168221076, 
                'w1': -1.33149532388453900, 
                'w2': -1.73915071826508780 }

    REST_POSE = {
                's0':  0.6634466907604415, 
                's1':  0.5150340495325276, 
                'e0':  0.42951462060791584, 
                'e1':  0.17755827619773665,  
                'w0': -0.16988837225830958, 
                'w1':  0.9894176081860918, 
                'w2': -2.0075973561450353 }


    # IMPEDANCE PARAMETERS
    # Case 1
    JOINT_IMP1_Kq = { 's0': 10.0, 
                      's1': 15.0, 
                      'e0':  5.0,  
                      'e1':  5.0,
                      'w0':  3.0, 
                      'w1':  3.0, 
                      'w2':  1.5 } 
  

    JOINT_IMP1_Bq = { 's0': 2.0, 
                      's1': 3.0, 
                      'e0': 1.0, 
                      'e1': 1.0,
                      'w0': 0.6, 
                      'w1': 0.6,
                      'w2': 0.3  } 

    # # Case 2 - Stronger
    JOINT_IMP2_Kq = { 's0': 10.0, 
                      's1': 15.0, 
                      'e0':  5.0,  
                      'e1': 15.0,
                      'w0':  3.0, 
                      'w1':  3.0, 
                      'w2':  1.5 } 
  

    JOINT_IMP2_Bq = { 's0': 5.0, 
                      's1': 7.5, 
                      'e0': 2.5, 
                      'e1': 7.5,
                      'w0': 0.1,
                      'w1': 1.5,
                      'w2': 1.5  } 