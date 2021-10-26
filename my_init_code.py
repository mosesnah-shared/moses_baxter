#!/usr/bin/env python

# =========================================================== #
# [Author] Moses C. Nah
# [Email]  mosesnah@mit.edu
# [Date]   2021.10.23
# [Description]
#   - The most basic code for running Baxter
# =========================================================== #


import rospy
import time
import baxter_interface
import numpy        as np
import std_msgs.msg as msg 
import threading

from   std_msgs.msg       import UInt16, Empty, String
from   baxter_interface   import CHECK_VERSION
from   numpy.linalg       import inv, pinv
from   baxter_pykdl       import baxter_kinematics
from   tf.transformations import ( quaternion_from_euler , quaternion_matrix   , 
                                   quaternion_inverse    , quaternion_multiply )


# Local Library, under moses/scripts
from my_constants import Constants as C


# [BACKUP]
# import math
# from tf.transformations import (euler_from_quaternion, quaternion_from_euler)
 
class BaxterControl( object ):

    def __init__( self, record_data = False ):   # Using arm_type could be C.LEFT/C.RIGHT/C.BOTH

        self.record_data = record_data           # Boolean, data saved 
                                                 # Just define the whole arms in the first place to simplify the code
        self.arms        = range( 2 )            # Since we have both the left/right arms
        self.kins        = range( 2 )            # Since we have both the left/right arms
        self.grips       = range( 2 )            # Since we have both the left/right arms


        # Saving the limb objects
        for idx, name in enumerate( [ "right", "left" ] ):
            self.arms[  idx ] = baxter_interface.limb.Limb( name )
            self.kins[  idx ] = baxter_kinematics(          name )
            self.grips[ idx ] = baxter_interface.Gripper(   name )

            # Initialize the grippers holding force
            self.grips[ idx ].set_holding_force( 0 )

        self.robot_init( )  # Robot initialization

        # Setting the rate of the robot
        self.joint_publish_rate = 1000.0 # Hz      
        self.controller_rate    = 1000.0 # Hz

        # set joint state publishing to 1000Hz
        # [REF] http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
        # self.joint_state_pub = rospy.Publisher('robot/joint_state_publish_rate', String, queue_size = 10 )
        # self.joint_state_pub.publish( "hello" )   


        if self.record_data:
            time_now = time.strftime('%Y_%m_%d-%H_%M')
            self.file_Q  = open( C.SAVE_DIR + 'Q_MotionCon_'  + time_now + '.txt', 'w')      
     
    
    def robot_init( self ):

        # Initialization of the robot
        # Enable Robot
        rospy.loginfo( "Getting robot state... " )
        self.rs = baxter_interface.RobotEnable( CHECK_VERSION )
        self.init_state = self.rs.state().enabled
        rospy.loginfo("Enabling robot... ")
        self.rs.enable()


    def joint_impedances( self, pose1, pose2, D ):
        # Inputting the joint inputs
        # Given point
        Kq = np.array( [ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 ] )
        Bq = np.array( [ 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 ] )

        # Filling in the trajectory using minimum-jerk trajectory 

    def print_pose( self, pose ):

        # Print the joint data in specific order
        if   all( "right" in s for s in pose.keys( ) ):
            for name in C.RIGHT_JOINT_NAMES:
                print( "Moving " + name + " to " + str( pose[ name ] ) )

        elif all(  "left" in s for s in pose.keys( ) ):
            for name in C.LEFT_JOINT_NAMES:
                print( "Moving " + name + " to " + str( pose[ name ] ) )
            

    def move2pose( self, which_arm, pose, joint_speed = 0.1 ):

        self.print_pose( pose )


        # If which_arm is neither 0 nor 1, assert
        assert which_arm in [ C.RIGHT, C.LEFT] 

        if   which_arm == C.RIGHT:
            # Need to check whether the names contain right on the pose
            assert all( "right" in s for s in pose.keys( ) )

        elif which_arm == C.LEFT:

            # Need to check whether the names contain left on the pose
            assert all( "left" in s for s in pose.keys( ) )

        self.arms[ which_arm ].set_joint_position_speed( joint_speed )
        self.arms[ which_arm ].move_to_joint_positions( pose )            

    def print_q( self, t ):
        q_current_dict = self.arm.joint_angles()
        
        if self.arm_type == RIGHT :
            self.file_Q.write( str( t ) + " right " + ' ' + str(q_current_dict['right_s0']) + ' ' + str(q_current_dict['right_s1']) + ' ' + str(q_current_dict['right_e0']) + ' ' + str(q_current_dict['right_e1']) + ' '+ str(q_current_dict['right_w0']) + ' ' + str(q_current_dict['right_w1']) + ' ' + str(q_current_dict['right_w2']) + '\n')
        elif self.arm_type == LEFT :
            self.file_Q.write( str( t ) + " left " ' ' + str(q_current_dict['left_s0']) + ' ' + str(q_current_dict['left_s1']) + ' ' + str(q_current_dict['left_e0']) + ' ' + str(q_current_dict['left_e1']) + ' '+ str(q_current_dict['left_w0']) + ' ' + str(q_current_dict['left_w1']) + ' ' + str(q_current_dict['left_w2']) + '\n' )


    def grip_command( self ):
        self.grip.close()
   
        rospy.sleep(2)
        self.grip.open()


    def clean_shutdown(self):

        rospy.sleep(2)
        for i in range( 2 ):
            self.arms[ i ].exit_control_mode()


def main():
    
    print("Initializing Controller Node... ")
    
    rospy.init_node("MY_BAXTER_CONTROL")
    ctrl = BaxterControl(  )
    rospy.on_shutdown( ctrl.clean_shutdown )

    ctrl.move2pose( C.RIGHT, C.GRASP_POSE_RIGHT, joint_speed = 0.1 )
    ctrl.move2pose( C.LEFT,  C.GRASP_POSE_LEFT , joint_speed = 0.1 )

    # ctrl.print_joints( )
    # ctrl.grip_command( )



if __name__ == '__main__':
    main()
