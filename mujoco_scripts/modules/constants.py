class Constants:
    PROJECT_NAME            = '[M3X] Whip Project'
    VERSION                 = '1.0.0'
    AUTHOR_GITHUB           = 'mosesnah-shared'
    AUTHOR_FULL_NAME        = 'Moses C. Nah'
    DESCRIPTION             = "mujoco-py scripts for running a whip-targeting simuation"
    URL                     = 'https://github.com/mosesnah-shared/whip-project-targeting',
    AUTHOR_EMAIL            = 'mosesnah@mit.edu', 'mosesnah@naver.com'

    # =============================================================== #
    # Constant variables for running the simulation

    # The model directory which contains all the xml model files.
    MODEL_DIR     = "models/"

    # The directory which saves all the simulation results
    SAVE_DIR      = "results/"

    LIMB_NAMES = [ "right", "left" ]

    # The constants used for the limb
    RIGHT = 0       # This is at the same time, an index for the list
    LEFT  = 1       # This is at the same time, an index for the list
    BOTH  = 2


    LEFT_JOINT_NAMES  = [  "left_s0",  "left_s1",  "left_e0",  "left_e1",  "left_w0",  "left_w1",  "left_w2" ]
    RIGHT_JOINT_NAMES = [ "right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2" ]
    JOINT_NAMES       = [       "s0",       "s1",       "e0",       "e1",       "w0",       "w1",       "w2" ]
    JOINT_TO_FLIP     = [       "s0",                   "e0",                   "w0",                   "w2" ]
