#!/usr/bin/python3

"""
# =========================================================== #
# [Author            ] Moses C. Nah
# [Email             ] mosesnah@mit.edu
# [Date Created      ] 2022.09.24
# [Last Modification ] 2022.09.24
# =========================================================== #
"""

import rospy
import nlopt
import datetime
import numpy             as np

# Local Library, under moses/scripts
from my_robot        import Baxter
from my_constants    import Constants as C
from my_utils        import pose_right2left, dict2arr, arr2dict, poses_delta
from my_controllers  import JointImpedanceController

from scipy.io import savemat


np.set_printoptions( linewidth = 4000, precision = 8, suppress = True )


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

    # Find the input parameters (input_pars) that are aimed to be optimized
    # Possible options (written in integer values) are as follows
    # [REF] https://nlopt.readthedocs.io/en/latest/NLopt_Algorithms/


    #   idx are                 0                   1               2               3                  4
    idx_opt   = [ nlopt.GN_DIRECT_L, nlopt.GN_DIRECT_L_RAND, nlopt.GN_DIRECT, nlopt.GN_CRS2_LM, nlopt.GN_ESCH  ]
    idx       = 0
    lb    = np.array( [ -0.80, 0.20, -0.85, -0.10, 0.20, -0.85, 0.6, 0.6, -0.7 ] )
    ub    = np.array( [  0.00, 1.00,  0.85,  0.10, 1.00,  0.85, 1.5, 1.5,  0.7 ] )
    n_opt = 9


    FINAL_POSE = {  'right_s0' : 0.7869321442,
                    'right_s1' : 0.00000000000,
                    'right_e0' : -0.0149563127,
                    'right_e1' : 0.7309418454,
                    'right_w0' : -0.0464029188,
                    'right_w1' : -0.2511893540,
                    'right_w2' : -1.5823011827     }

    algorithm = idx_opt[ idx ]                                             
    opt       = nlopt.opt( algorithm, n_opt )              

    opt.set_lower_bounds( lb )
    opt.set_upper_bounds( ub )
                

    # The number of total trials
    Nt = 300
    opt.set_maxeval( Nt )
    
    # The number of trials for EACH iteration
    # Eventually, we will try Nt * Ns times of optimization
    Ns = 3
     
    # The original array for saving the data 
    data_arr = np.zeros( ( Nt, Ns    ) )
    pars_arr = np.zeros( ( Nt, n_opt ) )
     
    init = ( lb + ub ) * 0.5 + 0.05 * lb                                   

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
    
    tmp_t = 98 # If the coverage is higher than this value, simply set it as 100 and stop optimizaiton

    # Defining the objective function that we are aimed to optimize.
    def nlopt_objective( pars, grad ):                                 

        D1 = pars[ 6 ]
        D2 = pars[ 7 ]
        toff = pars[ 8 ] * D2
        
        for i in range( Ns ):
            
            # Goto grasp posture    
            go2pos( my_baxter, impR, impL, C.GRASP_POSE, 5 )
            
            impR.reset( )
            impL.reset( )
            
            RIGHT_MID_POSE = arr2dict( which_arm = "right", arr = np.array( [ S0, pars[ 0 ], E0, pars[ 1 ], W0, pars[ 2 ], W1 ] ) )
            LEFT_MID_POSE  = pose_right2left( RIGHT_MID_POSE )
            
            # Setting the movement    
            # MOVEMENT 1
            qi = dict2arr( which_arm = "right", my_dict = C.GRASP_POSE   )
            qf = dict2arr( which_arm = "right", my_dict = RIGHT_MID_POSE )
            impR.add_movement( qi = qi, qf = qf, D = D1, ti = 0 )
            
            RIGHT_FINAL_POSE = arr2dict( which_arm = "right", arr = np.array( [ S0, pars[ 3 ], E0, pars[ 4 ], W0, pars[ 5 ], W1 ] ) )
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
            

            # Get Baxter's tablecloth performance
            obj = rospy.get_param( 'my_obj_func' )
            if obj >= tmp_t:
                obj = 100.0

            data_arr[ opt.get_numevals( ), i ] = obj
                        
            
            impR.reset( )
            impL.reset( )            
            
        # Iteration done
        best_val = np.max( data_arr[ opt.get_numevals( ), : ] )
        pars_arr[ opt.get_numevals( ), : ] = pars[ : ]
        
        print( "[Iteration] " + str( opt.get_numevals( ) + 1 ) + " [parameters] " + np.array2string( pars.flatten() , separator = ',' )  )
        print( " [all_vals] " + np.array2string( data_arr[ opt.get_numevals( ), : ].flatten( ), separator = ',' ) )
        print( " [obj] " + str( best_val ) )

        # If done, reset
        for imp in imp_arr:  imp.reset( )

        return 100.0 - best_val 


    # input( "Ready for optimization, press any key to continue" )
    opt.set_min_objective( nlopt_objective )
    opt.set_stopval( 0.001  )                                      
    
    # Ready for optimization
    input( "Ready for optimization? press any key to continue" )        
    xopt = opt.optimize( init )

    mdic = { "data": data_arr, "pars": pars_arr }
    savemat(  C.SAVE_DIR + datetime.datetime.now().strftime( '%Y_%m_%d' ) + "/" +  datetime.datetime.now().strftime( '%Y%m%d_%H%M%S' ) + ".mat", mdic )

if __name__ == "__main__":
    
    main( )
