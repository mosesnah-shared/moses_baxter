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
    
    # Initializing the Baxter Robot
    my_baxter = Baxter( None )
   
    
    S0 =  0.7869321442
    E0 = -0.0149563127
    W0 = -0.0464029188
    W1 = -1.5823011827    
    
    # Define the right impedance controller
    Kq_mat = np.diag( [ 15.0, 15.0, 8.0, 10.0, 3.0, 12.0, 1.5 ]  )
    Bq_mat = 0.2 * Kq_mat
    
    # Define the right impedance controller
    impR = JointImpedanceController( my_baxter, which_arm = "right", name = "right_imp1", is_save_data = False )
    impR.set_impedance( Kq = Kq_mat, Bq = Bq_mat )
    
    # Define the left impedance controller
    impL = JointImpedanceController( my_baxter, which_arm = "left" , name = "left_imp1" , is_save_data = False )
    impL.set_impedance( Kq = Kq_mat, Bq = Bq_mat )    

    # Uncomment the following 3 sentences in case if the tablecloth is not equipped.
    # my_baxter.move2pose( C.RIGHT, C.GRASP_POSE, wait_time = 2, joint_speed = 0.2 )
    # my_baxter.move2pose( C.LEFT,  C.GRASP_POSE, wait_time = 2, joint_speed = 0.2 )
    # my_baxter.control_gripper( mode = "timer" )

    # Goto grasp posture    
    go2pos( my_baxter, impR, impL, C.GRASP_POSE, 5 )

    # Adding the gripper 
    # Close the gripper
    rospy.sleep( 5 )
    my_baxter.close_gripper( )        
    rospy.sleep( 2 )
    
    imp_arr  = [ impR, impL ]
    for imp in imp_arr:  imp.reset( )
    
    # We only change the s1, e1, w1 values 
    # Setting the other joint values (w.r.t. right hand) as constants
    S0 =  0.7869321442
    E0 = -0.0149563127
    W0 = -0.0464029188
    W1 = -1.5823011827
    
    # The optimal movement parameters of NLOPT DIRECT-L
    # [REF] /home/baxterplayground/ros_ws/src/newmanlab_code/moses_baxter/results/2022_10_02/small_cloth_DIRECT_L.txt
    mov_pars = np.array( [-0.66666667, 0.6       , 0.56666667, 0.        , 0.6       , 0.        , 1.05      , 1.05      , 0.        ] )
    
    D1 = mov_pars[ 6 ]
    D2 = mov_pars[ 7 ]
    toff = mov_pars[ 8 ] * D2
    
    # Goto grasp posture    
    go2pos( my_baxter, impR, impL, C.GRASP_POSE, 5 )
    
    impR.reset( )
    impL.reset( )
    
    RIGHT_MID_POSE = arr2dict( which_arm = "right", arr = np.array( [ S0, mov_pars[ 0 ], E0, mov_pars[ 1 ], W0, mov_pars[ 2 ], W1 ] ) )
    LEFT_MID_POSE  = pose_right2left( RIGHT_MID_POSE )
    
    # Setting the movement    
    # MOVEMENT 1
    qi = dict2arr( which_arm = "right", my_dict = C.GRASP_POSE   )
    qf = dict2arr( which_arm = "right", my_dict = RIGHT_MID_POSE )
    impR.add_movement( qi = qi, qf = qf, D = D1, ti = 0 )
    
    RIGHT_FINAL_POSE = arr2dict( which_arm = "right", arr = np.array( [ S0, mov_pars[ 3 ], E0, mov_pars[ 4 ], W0, mov_pars[ 5 ], W1 ] ) )
    LEFT_FINAL_POSE  = pose_right2left( RIGHT_FINAL_POSE )            
    
    # MOVEMENT 2
    qi = np.zeros( 7 )
    qf = dict2arr( which_arm = "right", my_dict = poses_delta( "right", RIGHT_MID_POSE, RIGHT_FINAL_POSE ) ) 
    impR.add_movement( qi = np.zeros( 7 ), qf = qf, D = D2, ti = D1 + toff )    
            
    # Add the left movements
    # MOVEMENT 1
    qi = dict2arr( which_arm = "left", my_dict = pose_right2left( C.GRASP_POSE ) )
    qf = dict2arr( which_arm = "left", my_dict = LEFT_MID_POSE  )
    impL.add_movement( qi = qi, qf = qf, D = D1, ti = 0 )
    
    # MOVEMENT 2
    qi = np.zeros( 7 )
    qf = dict2arr( which_arm = "left", my_dict = poses_delta( "left", LEFT_MID_POSE, LEFT_FINAL_POSE ) ) 
    impL.add_movement(  qi = np.zeros( 7 ), qf = qf, D = D2, ti = D1 + toff )            
    
    # Setup the impedances 
    imp_arr  = [ impR, impL ]
    for imp in imp_arr:  imp.setup( )                             
    
    # Time initialization
    ts = rospy.Time.now( )
    t  = 0        
    
    # Running the main loop
    while not rospy.is_shutdown( ) and t <= D1 + toff + D2 + 3:

        tau = { "right": np.zeros( 7 ), "left": np.zeros( 7 ) }
        
        # Iterating over the impedances 
        for imp in imp_arr:
            
            # Calculate the Torque of the arm 
            tmp_tau = imp.calc_torque( t )
            
            tau[ imp.which_arm ] += tmp_tau 

        for limb_name in [ "right", "left" ]:
            my_baxter.arms[ limb_name ].set_joint_torques( arr2dict( limb_name, arr = tau[ limb_name ]  )  )
            
        my_baxter.control_rate.sleep( )                
        t = ( rospy.Time.now( ) - ts ).to_sec( )
    
    impR.reset( )
    impL.reset( )
    
    # Goto grasp posture    
    go2pos( my_baxter, impR, impL, C.FINAL_POSE2, 5 )

    rospy.sleep( 2 )            
    
    impR.reset( )
    impL.reset( )            
            
if __name__ == "__main__":
    
    main( )
