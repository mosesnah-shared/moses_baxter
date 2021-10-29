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

    MID_POSE_RIGHT  = {
        'right_s0':  0.6830049458059805, 
        'right_s1': -0.9219224535191337,
        'right_e0':  0.3267379078195931, 
        'right_e1':  0.3673883986985566, 
        'right_w0': -0.16336895390979655, 
        'right_w1':  1.267451625990323, 
        'right_w2': -1.7172914920377207 
    }


    MID_POSE_LEFT  = {
        'left_s0': -0.6830049458059805, 
        'left_s1': -0.9219224535191337,
        'left_e0': -0.3267379078195931, 
        'left_e1':  0.3673883986985566, 
        'left_w0':  0.16336895390979655, 
        'left_w1':  1.267451625990323, 
        'left_w2':  1.7172914920377207 
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