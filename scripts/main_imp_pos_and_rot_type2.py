#!/usr/bin/python3

"""
# =========================================================== #
# [Author            ] Moses C. Nah
# [Email             ] mosesnah@mit.edu
# [Date Created      ] 2022.01.26
# [Last Modification ] 2022.04.17
# =========================================================== #
"""

import rospy
import nlopt
import argparse
import numpy             as np
import matplotlib.pyplot as plt

# Local Library, under moses/scripts
from my_robot        import Baxter
from my_constants    import Constants as C
from my_utils        import pose_right2left, dict2arr, arr2dict, make_dir, quat2rot, skew_sym, quat2angx, rot2quat, poses_delta
from my_controllers  import JointPositionController, JointImpedanceController, CartesianImpedanceControllerRotationType2


np.set_printoptions( linewidth = 4000, precision = 8)

def main():

    parser  = argparse.ArgumentParser( formatter_class = argparse.RawTextHelpFormatter )
    parser.add_argument('-s', '--save_data' , dest = 'is_save_data' , action = 'store_true' , help = 'Save the Data' )                                     
    args = parser.parse_args( rospy.myargv( )[ 1: ] )
    
    # Initializing the Baxter Robot
    my_baxter = Baxter( None )
    
    my_ctrl = JointPositionController( my_baxter )
    my_ctrl.add_movement( which_arm = "right", pose2go = C.GRASP_POSE_UP                   , joint_vel = 0.3, toff = 3 )    
    my_ctrl.add_movement( which_arm = "left" , pose2go = pose_right2left( C.GRASP_POSE_UP ), joint_vel = 0.3, toff = 3 )    
    my_ctrl.run( )
    
    
    # Adding the gripper 
    # Close the gripper
    # my_baxter.close_gripper( )
    
    # Define the impedance controller
    Kq_mat = np.diag( [ 15.0, 15.0, 8.0, 10.0, 3.0, 10.0, 1.5 ]  )
    Bq_mat = 0.2 * Kq_mat

    # First RIGHT Impedance
    impR_1 = JointImpedanceController( my_baxter, which_arm = "right", name = "right_imp1", is_save_data = False )
    impR_1.set_impedance( Kq = Kq_mat, Bq = Bq_mat )
                                    
    # Add the movements                               
    tmp_R_pose = my_baxter.get_arm_pose( which_arm = "right" )
    qi = dict2arr( which_arm = "right", my_dict = tmp_R_pose )
    qf = dict2arr( which_arm = "right", my_dict = C.GRASP_TMP )
    D1 = 4.0
    toff = 1.0
    impR_1.add_movement( qi = qi, qf = qf, D = D1, ti = 0. )
    impR_1.add_movement( qi = np.zeros( 7 ), qf = dict2arr( which_arm = "right", my_dict = poses_delta( which_arm = "right", pose1 = C.GRASP_TMP, pose2 = tmp_R_pose ) ), D = D1, ti = D1 + toff )
    impR_1.add_movement( qi = np.zeros( 7 ), qf = dict2arr( which_arm = "right", my_dict = poses_delta( which_arm = "right", pose1 = tmp_R_pose, pose2 = C.GRASP_TMP ) ), D = D1, ti = 2 * ( D1 + toff  ) )
    impR_1.add_movement( qi = np.zeros( 7 ), qf = dict2arr( which_arm = "right", my_dict = poses_delta( which_arm = "right", pose1 = C.GRASP_TMP, pose2 = tmp_R_pose ) ), D = D1, ti = 3 * ( D1 + toff  ) )


    # First LEFT Impedance
    impL_1 = JointImpedanceController( my_baxter, which_arm = "left", name = "left_imp1", is_save_data = False )
    impL_1.set_impedance( Kq = Kq_mat, Bq = Bq_mat )
                                    
    # Add the movements      
    tmp_L_pose = my_baxter.get_arm_pose( which_arm = "left" )                
    qi = dict2arr( which_arm = "left", my_dict = tmp_L_pose )
    qf = dict2arr( which_arm = "left", my_dict = pose_right2left( C.GRASP_TMP ) )
    D1 = 4.0
    toff = 1.0
    impL_1.add_movement( qi = qi, qf = qf, D = D1, ti = 0. )
    impL_1.add_movement( qi = np.zeros( 7 ), qf = dict2arr( which_arm = "left", my_dict = poses_delta( which_arm = "left", pose1 = pose_right2left( C.GRASP_TMP ), pose2 = tmp_L_pose ) ), D = D1, ti = D1 + toff )
    impL_1.add_movement( qi = np.zeros( 7 ), qf = dict2arr( which_arm = "left", my_dict = poses_delta( which_arm = "left", pose1 = tmp_L_pose, pose2 = pose_right2left( C.GRASP_TMP ) ) ), D = D1, ti = 2 * ( D1 + toff ) )
    impL_1.add_movement( qi = np.zeros( 7 ), qf = dict2arr( which_arm = "left", my_dict = poses_delta( which_arm = "left", pose1 = pose_right2left( C.GRASP_TMP ), pose2 = tmp_L_pose ) ), D = D1, ti = 3 * ( D1 + toff ) )

     
    # The 2nd impedance for matching the orientation
    impR_2 = CartesianImpedanceControllerRotationType2( my_baxter, which_arm = "right", name = "right_imp2", is_save_data = args.is_save_data )
    impR_2.set_rotational_impedance( Kr = 20 * np.eye( 3 ), Br = 2 * np.eye( 3 ) )
    right_orient = my_baxter.get_end_effector_orientation( which_arm = "right" )
    impR_2.set_desired_orientation( right_orient )
    
    # The 2nd impedance for matching the orientation
    impL_2 = CartesianImpedanceControllerRotationType2( my_baxter, which_arm = "left", name = "left_imp2", is_save_data = args.is_save_data )
    impL_2.set_rotational_impedance( Kr = 0 * np.eye( 3 ), Br = 0 * np.eye( 3 ) )
    left_orient = my_baxter.get_end_effector_orientation( which_arm = "left" )
    impL_2.set_desired_orientation( left_orient )    
    
    
    # Saving these impedances as an array to iterate over 
    imp_arr  = [ impR_1 , impL_1 ,impR_2 , impL_2 ]
    
    for imp in imp_arr: imp.setup( )
                    
    input( "Ready for Initiating the movements? press any key to continue" )

    ts = rospy.Time.now( )
    t  = 0

    # Running the main loop
    while not rospy.is_shutdown( ) and t <= 22:

        tau = { "right": np.zeros( 7 ), "left": np.zeros( 7 ) }
        
        # Iterating over the impedances 
        for imp in imp_arr:
            
            # Calculate the Torque of the arm 
            tmp_tau = imp.calc_torque( t )
            
            tau[ imp.which_arm ] += tmp_tau 

        for limb_name in [ "right", "left" ]:
            my_baxter.arms[ limb_name ].set_joint_torques( arr2dict( limb_name, arr = tau[ limb_name ]  )  )
            
        # if is_publish_data:        
        my_baxter.control_rate.sleep( )
        
        t = ( rospy.Time.now( ) - ts ).to_sec( )
        
    if args.is_save_data: dir_name = make_dir( )

    for imp in imp_arr:
        if imp.is_save_data: imp.publish_data( dir_name = dir_name )

if __name__ == "__main__":
    
    main( )
