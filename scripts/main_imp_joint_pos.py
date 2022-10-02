#!/usr/bin/python3

import rospy
import argparse
import numpy             as np

# Local Library, under moses/scripts
from my_robot        import Baxter
from my_constants    import Constants as C
from my_utils        import pose_right2left, dict2arr, arr2dict, make_dir, poses_delta
from my_controllers  import JointImpedanceController

np.set_printoptions( linewidth = 4000, precision = 8 )


# A helper function for this script
def go2pos( my_baxter, impR, impL, pos, D ):
    
    # Add the right movements                               
    qi = dict2arr( which_arm = "right", my_dict = my_baxter.get_arm_pose( which_arm = "right" ) )
    qf = dict2arr( which_arm = "right", my_dict = pos )
    impR.add_movement( qi = qi, qf = qf, D = D, ti = 0. )
               
    # Add the left movements                               
    qi = dict2arr( which_arm = "left", my_dict = my_baxter.get_arm_pose( which_arm = "left" ) )
    qf = dict2arr( which_arm = "left", my_dict = pose_right2left( pos ) )
    impL.add_movement( qi = qi, qf = qf, D = D, ti = 0. )
    
    # Saving these impedances and going to the grasp_posture 
    imp_arr  = [ impR, impL ]
    
    for imp in imp_arr: imp.setup( )
                    
    ts = rospy.Time.now( )
    t  = 0

    # Running the main loop
    while not rospy.is_shutdown( ) and t <= D + 2:

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
        
        

def main():

    parser  = argparse.ArgumentParser( formatter_class = argparse.RawTextHelpFormatter )
    parser.add_argument('-s', '--save_data' , dest = 'is_save_data' , action = 'store_true' , help = 'Save the Data' )                                     
    args = parser.parse_args( rospy.myargv( )[ 1: ] )
    
    # Initializing the Baxter Robot
    my_baxter = Baxter( None )
        
    # Define the right impedance controller
    Kq_mat = np.diag( [ 15.0, 15.0, 8.0, 10.0, 3.0, 12.0, 1.5 ]  )
    Bq_mat = 0.2 * Kq_mat
    
    # Define the right impedance controller
    impR = JointImpedanceController( my_baxter, which_arm = "right", name = "right_imp1", is_save_data = False )
    impR.set_impedance( Kq = Kq_mat, Bq = Bq_mat )
    
    # Define the left impedance controller
    impL = JointImpedanceController( my_baxter, which_arm = "left" , name = "left_imp1" , is_save_data = False )
    impL.set_impedance( Kq = Kq_mat, Bq = Bq_mat )    
    
    # Goto grasp posture    
    go2pos( my_baxter, impR, impL, C.FINAL_POSE2, 5 )
    
    
    # Adding the gripper 
    # Close the gripper
    rospy.sleep( 5 )
    my_baxter.close_gripper( )        
    rospy.sleep( 3 )
    
    # Before conducting the movement, reset the movements of the controller
    # Saving these impedances and going to the grasp_posture 
    imp_arr  = [ impR, impL ]
    
    for imp in imp_arr:  imp.reset( )
    
    # Defining the movements
    # Add the right movements                   
    D1    = 0.85
    D2    = 1.35
    alpha = 0.31666667  
    ti_tmp = 1
                

    # Add the right movement                
    # MOVEMENT 1
    qi = dict2arr( which_arm = "right", my_dict = my_baxter.get_arm_pose( which_arm = "right" ) )
    qf = dict2arr( which_arm = "right", my_dict = C.OPT_MID_POS )
    impR.add_movement( qi = qi, qf = qf, D = D1, ti = ti_tmp )
    
    # MOVEMENT 2
    qi = np.zeros( 7 )
    qf = dict2arr( which_arm = "right", my_dict = poses_delta( "right", C.OPT_MID_POS, C.FINAL_POSE ) ) 
    impR.add_movement( qi = np.zeros( 7 ), qf = qf, D = D2, ti = ti_tmp + D1 + alpha * D2 )    
               
    # Add the left movements
    # MOVEMENT 1
    qi = dict2arr( which_arm = "left", my_dict = my_baxter.get_arm_pose( which_arm = "left" ) )
    qf = dict2arr( which_arm = "left", my_dict = pose_right2left( C.OPT_MID_POS ) )
    impL.add_movement( qi = qi, qf = qf, D = D1, ti = ti_tmp )
    
    # MOVEMENT 2
    qi = np.zeros( 7 )
    qf = dict2arr( which_arm = "left", my_dict = poses_delta( "left", pose_right2left( C.OPT_MID_POS ), pose_right2left( C.FINAL_POSE ) ) ) 
    impL.add_movement(  qi = np.zeros( 7 ), qf = qf, D = D2, ti = ti_tmp + D1 + alpha * D2 )    
    
    # Setup the impedances 
    imp_arr  = [ impR, impL ]
    for imp in imp_arr:  imp.setup( )    
                                
    input( "Ready for Initiating the movements? press any key to continue" )        

          
    ts = rospy.Time.now( )
    t  = 0        

    # Running the main loop
    while not rospy.is_shutdown( ) and t <= ti_tmp + D1 + D2 * ( 1 + alpha ) + 5:

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
            
            
if __name__ == "__main__":
    
    main( )
