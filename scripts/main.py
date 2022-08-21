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
from my_utils        import pose_right2left, dict2arr, arr2dict, poses_delta, Logger, make_dir
from my_controllers  import PrintController, JointPositionController, JointImpedanceController, CartesianImpedanceController


np.set_printoptions( linewidth = 4000, precision = 8)


def main():

    parser  = argparse.ArgumentParser( formatter_class = argparse.RawTextHelpFormatter )
    parser.add_argument('-s', '--save_data'        , dest = 'is_save_data'        , action = 'store_true' ,              help = 'Save the Data'                                   )
    parser.add_argument('-o', '--run_optimization' , dest = 'is_run_optimization' , action = 'store_true' ,              help = 'Running the optimization of the whole process'   )
    parser.add_argument('-c', '--controller'       , dest = 'ctrl_type'           , action = 'store'      , type =  str, help = C.CONTROLLER_DESCRIPTIONS                         )
    args = parser.parse_args( rospy.myargv( )[ 1: ] )

    print( "Initializing node... " )
    rospy.init_node( "impedance_control" )
    my_baxter = Baxter( args )
    rospy.on_shutdown( my_baxter.clean_shutdown )

    if args.is_run_optimization:

        my_log = Logger( record_data = True )

        # Find the input parameters (input_pars) that are aimed to be optimized
        # Possible options (written in integer values) are as follows
        # [REF] https://nlopt.readthedocs.io/en/latest/NLopt_Algorithms/

        # Define the controller
        my_ctrl = JointImpedanceController( my_baxter, is_save_data = False )
        print( "[LOG] OPTIMIZATION MODE" )


        #   idx are                 0                   1               2               3                  4
        idx_opt   = [ nlopt.GN_DIRECT_L, nlopt.GN_DIRECT_L_RAND, nlopt.GN_DIRECT, nlopt.GN_CRS2_LM, nlopt.GN_ESCH  ]
        idx       = 0


        #  Upper/Lower Bound, ordered as:
        #  POSE_MID_s1    POSE_MID_e1     POSE_MID_w1,  POSE_FINAL_s1,  POSE_FINAL_e1, POSE_FINAL_w1   D1   D2    a
        # lb    = np.array( [ -0.65, 0.31, -0.76, -0.65, 0.31, -0.76, 0.6, 0.6, -0.6 ] )
        # ub    = np.array( [  0.00, 0.83, -0.25,  0.00, 0.83, -0.25, 1.5, 1.5,  0.5 ] )
        # n_opt = 9

        lb    = np.array( [ -0.65, 0.31, -0.76, 0.6, 0.6, -0.6 ] )
        ub    = np.array( [  0.00, 0.83, -0.25, 1.5, 1.5,  0.5 ] )
        n_opt = 6

        algorithm = idx_opt[ idx ]                                              # Selecting the algorithm to be executed
        opt       = nlopt.opt( algorithm, n_opt )                               # Defining the class for optimization

        opt.set_lower_bounds( lb )
        opt.set_upper_bounds( ub )
        opt.set_maxeval( 100 )

        init = ( lb + ub ) * 0.5 + 0.05 * lb                                    # Setting an arbitrary non-zero initial step


        # Uncomment the following 3 sentences in case if the tablecloth is not equipped.
        # my_baxter.move2pose( C.RIGHT, C.GRASP_POSE, wait_time = 2, joint_speed = 0.2 )
        # my_baxter.move2pose( C.LEFT,  C.GRASP_POSE, wait_time = 2, joint_speed = 0.2 )
        # my_baxter.control_gripper( mode = "timer" )

        tmp_t = 98 # If the coverage is higher than this value, simply set it as 100 and stop optimizaiton

        def nlopt_objective( pars, grad ):                                      # Defining the objective function that we are aimed to optimize.

            # Run Baxter - Code implementation here
            # Manipulating the cloth
            # [STEP #1] Move to initial posture
            # [STEP #2] Initiate movement
            # Go to the initial posture
            tmp_arr = np.zeros( 5 )

            for i in range( 5 ):

                my_ctrl.move2pose( C.GRASP_POSE, duration = 5, toff = 0.1 )


                POSE1_R = C.GRASP_POSE
                POSE1_L = pose_right2left( C.GRASP_POSE  )

                # s0, s1, e0, e1, w0, w1, w2 number
                POSE2_R = my_ctrl.gen_dict( "right", np.array( [ C.GRASP_POSE[ "right_s0" ],
                                                                                  pars[ 0 ],
                                                                 C.GRASP_POSE[ "right_e0" ],
                                                                                  pars[ 1 ],
                                                                 C.GRASP_POSE[ "right_w0" ],
                                                                                  pars[ 2 ],
                                                                 C.GRASP_POSE[ "right_w2" ] ] ) )
                POSE2_L = pose_right2left( POSE2_R  )

                # POSE3_R = my_ctrl.gen_dict( "right", np.array( [ C.GRASP_POSE[ "right_s0" ],
                #                                                                   pars[ 3 ],
                #                                                  C.GRASP_POSE[ "right_e0" ],
                #                                                                   pars[ 4 ],
                #                                                  C.GRASP_POSE[ "right_w0" ],
                #                                                                   pars[ 5 ],
                #                                                  C.GRASP_POSE[ "right_w2" ] ] ) )
                # POSE3_L = pose_right2left( POSE3_R  )

                POSE3_R = C.FINAL_POSE
                POSE3_L = pose_right2left( C.FINAL_POSE  )


                my_ctrl.add_movement( which_arm = "right", pose_init = POSE1_R, pose_final = POSE2_R, duration = pars[ 3 ], toff = 0.0                   )
                my_ctrl.add_movement( which_arm = "right", pose_init = POSE2_R, pose_final = POSE3_R, duration = pars[ 4 ], toff = pars[ 3 ] * pars[ 5 ] )

                my_ctrl.add_movement( which_arm = "left" , pose_init = POSE1_L, pose_final = POSE2_L, duration = pars[ 3 ], toff = 0.0                   )
                my_ctrl.add_movement( which_arm = "left" , pose_init = POSE2_L, pose_final = POSE3_L, duration = pars[ 4 ], toff = pars[ 3 ] * pars[ 5 ] )


                my_ctrl.run( )
                rospy.sleep( 3 )

                # Get Baxter's tablecloth performance
                obj = rospy.get_param( 'my_obj_func' )
                if obj >= tmp_t:
                    obj = 100.0

                tmp_arr[ i ] = obj
                my_ctrl.reset( )

            good = np.max( tmp_arr )
            my_log.write( "[Iteration] " + str( opt.get_numevals( ) + 1 ) + " [parameters] " + np.array2string( np.array( pars ).flatten(), separator = ',' ) )
            my_log.write( " [all_vals] " + np.array2string( np.array( tmp_arr ).flatten(), separator = ',' ) )
            my_log.write( " [obj] " + str( good ) + "\n" )

            return 100.0 - good # Inverting the value


        # input( "Ready for optimization, press any key to continue" )

        opt.set_min_objective( nlopt_objective )
        opt.set_stopval( -1    )                                                # If value is within 98~100% (i.e., 0~2%)
        xopt = opt.optimize( init )                                             # Start at the mid-point of the lower and upper bound

        my_log.log.close()
        

    elif not args.is_run_optimization:


        if   args.ctrl_type == "joint_position_controller":
            my_ctrl = JointPositionController( my_baxter )
            my_ctrl.run_example( type = "default" )

        elif args.ctrl_type == "print_controller":
            my_ctrl = PrintController( my_baxter )
            my_ctrl.run( )

        elif args.ctrl_type == "joint_impedance_controller":
            
            # Setting up the Impedance Parameters of the Joint
            Kq_mat = np.diag( [ 15.0, 15.0, 8.0, 10.0, 3.0, 10.0, 1.5 ]  )
            Bq_mat = 0.2 * Kq_mat

            # ============================================================ #
            # ================== RIGHT LIMB IMPEDANCES =================== #
            # ============================================================ #

            # First RIGHT Impedance
            impR_1 = JointImpedanceController( my_baxter, which_arm = "right", name = "right_imp1", is_save_data = args.is_save_data )
            impR_1.set_impedance( Kq = Kq_mat, Bq = Bq_mat )
                                            
            # Add the movements                               
            qi = dict2arr( which_arm = "right", my_dict = my_baxter.get_arm_pose( which_arm = "right" ) )
            qf = dict2arr( which_arm = "right", my_dict = C.GRASP_POSE )
            D1 = 3.0
            impR_1.add_movement( qi = qi, qf = qf, D = D1, ti = 0. )

            # Second RIGHT Impedance    
            # Time offset of the 2nd movement 
            alpha = -0.2
            qi = np.zeros( 7 )
            qf = dict2arr( "right", poses_delta( "right", C.GRASP_POSE, C.MID_POSE ) )
            D2 = 5.0
            impR_1.add_movement( qi = qi, qf = qf, D = D2, ti = ( 1 + alpha ) * D1 )
                
            # ============================================================ #
            # =================== LEFT LIMB IMPEDANCES =================== #
            # ============================================================ #
                
            impL_1 = JointImpedanceController( my_baxter, which_arm = "left", name = "left_imp1", is_save_data = args.is_save_data )
            impL_1.set_impedance( Kq = Kq_mat, Bq = Bq_mat )
            
            # Add the movements 
            qi = dict2arr( which_arm = "left", my_dict = my_baxter.get_arm_pose( which_arm = "left" ) )
            qf = dict2arr( which_arm = "left", my_dict = pose_right2left( C.GRASP_POSE ) )
            D1 = 3.0
            impL_1.add_movement( qi = qi, qf = qf, D = D1, ti = 0. ) 

            # Saving these impedances as an array to iterate over 
            imp_arr  = [ impR_1, impL_1 ]
            
            for imp in imp_arr: imp.setup( )
            
            ts = rospy.Time.now( )
            t  = 0
            
            # Running the main loop
            while not rospy.is_shutdown( ) and t <= 13:

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


            # After finishing the movement, publishing the data 
            # The number of impedances should match the message
            # Each Impedance Must have an independent Message
            # Check whether you will publish the data or not.
            dir_name = make_dir( )

            for imp in imp_arr:
                imp.publish_data( dir_name = dir_name )
                

        elif args.ctrl_type == "cartesian_impedance_controller":
            imp1 = CartesianImpedanceController( my_baxter, "right" )

        elif args.ctrl_type == "cartesian_and_joint_impedance_controller":
            pass

if __name__ == "__main__":
    
    main( )
