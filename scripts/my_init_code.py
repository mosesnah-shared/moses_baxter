#!/usr/bin/python3


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
import sys
import argparse


from   std_msgs.msg       import UInt16, Empty, String
from   sensor_msgs.msg    import JointState
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

# [REF] https://stackoverflow.com/questions/14906764/how-to-redirect-stdout-to-both-file-and-console-with-scripting
class Logger(object):
    def __init__(self, record_data = False):
        self.record_data = record_data
        self.terminal    = sys.stdout

        if record_data == True:
            time_now         = time.strftime( '%Y_%m_%d-%H_%M' )                 # Redirecting the stdout to file
            self.log = open( C.SAVE_DIR + 'baxter_'  + time_now + '.txt' , 'w')
        else:
            self.log = None

    def write(self, message ):
        self.terminal.write( message )

        if self.log is not None:
            self.log.write( message )

    def flush(self):
        # this flush method is needed for python 3 compatibility.
        # this handles the flush command by doing nothing.
        # you might want to specify some extra behavior here.
        pass



def callback( data, start_time ):
    # For the JointState data, we have the following data
    # [1] position
    # [2] velocity
    # [3] effort
    # [4] name
    # [*] etc.
    # [REF] https://stackoverflow.com/questions/57271100/how-to-feed-the-data-obtained-from-rospy-subscriber-data-into-a-variable
    # We are interested at the joint position data, hence check whether the given data are joint data
    # Can add more inputs if you want, by simply concatenating them
    # [REF] https://answers.ros.org/question/332126/passing-multiple-arguments-to-subscriber-callback-function-in-python/
    # We will check this via finding a specific string
    if 'torso_t0' in data.name:


        time = ( rospy.Time.now() - start_time).to_sec()
        for name in C.LEFT_JOINT_NAMES:
            # Find index of the name
            try:
                idx = data.name.index( name )
                rospy.loginfo( "[time] [%.4f] [name] [%s] [value] [%.5f]", time, name, data.position[ idx ]  )
                # if file is not None:
                    # file.write( "time = {}, name = {},  value = {}".format( time, name, data.position[ idx ] ) )
            except:
                NotImplementedError( )

        time = ( rospy.Time.now() - start_time).to_sec()
        for name in C.RIGHT_JOINT_NAMES:
            # Find index of the name
            try:
                idx = data.name.index( name )
                rospy.loginfo( "[time] [%.4f] [name] [%s] [value] [%.5f]", time, name, data.position[ idx ]  )
                # if file is not None:
                #     file.write( "time = {%.4f}, name = {%s},  value = {%.5f}".format( time, name, data.position[ idx ] ) )
            except:
                NotImplementedError( )


class BaxterControl( object ):

    def __init__( self, record_data = True ):   # Using arm_type could be C.LEFT/C.RIGHT/C.BOTH

        self.record_data = record_data           # Boolean, data saved
                                                 # Just define the whole arms in the first place to simplify the code
        self.arms        = range( 2 )            # Since we have both the left/right arms
        self.kins        = range( 2 )            # Since we have both the left/right arms
        self.grips       = range( 2 )            # Since we have both the left/right arms

        self.start_time  = rospy.Time.now()

        # Saving the limb objects
        for idx, name in enumerate( [ "right", "left" ] ):
            self.arms[  idx ] = baxter_interface.limb.Limb( name )
            self.kins[  idx ] = baxter_kinematics(          name )
            self.grips[ idx ] = baxter_interface.Gripper(   name )

            # Initialize the grippers holding force
            self.grips[ idx ].set_holding_force( 0 )


        # Setting the rate of the robot
        self.joint_publish_rate = 1000.0 # Hz
        self.controller_rate    =  500.0 # Hz
        self.missed_cmds        =   20.0 # Missed cycles before triggering timeout


        # set joint state publishing to 1000Hz
        # [REF] http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
        self.joint_state_pub = rospy.Publisher('robot/joint_state_publish_rate', UInt16, queue_size = 10 )
        # self.joint_state_pub.publish( "hello" )
        self.joint_state_pub.publish( self.joint_publish_rate )


        self.robot_init( )  # Robot initialization


        # print( data.position )
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        sys.stdout = Logger( record_data = record_data )

    def robot_init( self ):

        # Initialization of the robot
        # Enable Robot
        rospy.loginfo( "Getting robot state... " )
        self.rs = baxter_interface.RobotEnable( CHECK_VERSION )
        self.init_state = self.rs.state().enabled
        rospy.loginfo("Enabling robot... ")
        self.rs.enable()


    def joint_state_listener( self ):

        # rospy.init_node( 'my_listener', anonymous = False )
        rospy.Subscriber( "robot/joint_states", JointState, callback, self.start_time )



    def joint_impedances( self, pose1, pose2, D ):
        # Inputting the joint inputs
        # Given point
        Kq  =  0.002 * np.array( [   1,   1,   1,  1,  1,  1,  1 ] )
        Bq  =  0.004 * np.array( [   1,   1,   1,  1,  1,  1,  1 ] )

        rate = rospy.Rate( self.controller_rate )

        self.joint_state_pub.publish(self.joint_publish_rate)

        self.arms[ C.RIGHT ].set_command_timeout( (1.0 / self.joint_publish_rate) * self.missed_cmds )

        # Get the initial time
        ts = rospy.Time.now().to_sec( )
        t  = ts

        # print( t, ts, t-ts)
        while t - ts <= 4:
            t        = rospy.Time.now().to_sec()
            # print( t - ts)
            q0 , dq0 = self.get_joint_ref_traj( pose1, pose2, D, t - ts )
            q  , dq  = self.get_joint_current(  )                            # Get current joint postures and velociries

            tau      = np.multiply( np.squeeze( Kq ) , np.squeeze( q0 - q ) ) #- np.multiply( np.squeeze( Bq ), np.squeeze( dq ) )#+ np.multiply( np.squeeze( Bq ) , np.squeeze( dq0 - dq ) )   # Multiply is the hadamard product


            for i in range( 7 ):
                rospy.loginfo( "[time] [%.4f] [name] [%s] [value] [%.5f]", t - ts, C.RIGHT_JOINT_NAMES[ i ] + "_q0" , q0[ i ]  )
                rospy.loginfo( "[time] [%.4f] [name] [%s] [value] [%.5f]", t - ts, C.RIGHT_JOINT_NAMES[ i ] + "_q"  , q[ i ]   )
                rospy.loginfo( "[time] [%.4f] [name] [%s] [value] [%.5f]", t - ts, C.RIGHT_JOINT_NAMES[ i ] + "_tau", tau[ i ] )
                rospy.loginfo( "[time] [%.4f] [name] [%s] [value] [%.5f]", t - ts, C.RIGHT_JOINT_NAMES[ i ] + "_diff", q0[ i ] - q[ i ] )


            # print( t - ts )
            # print( t - ts, tau )
            # tau = np.clip( tau, -0.005, 0.005)
            # print( t-ts, q0 - q, tau )


            tmp = dict( zip( C.RIGHT_JOINT_NAMES, tau ) )


            # print( tmp)
            self.arms[ C.RIGHT ].set_joint_torques( tmp )
            rate.sleep( )


    def get_joint_current( self ):

        # Get only the values of the joint
        tmp_pos = self.arms[ C.RIGHT ].joint_angles( )
        tmp_vel = self.arms[ C.RIGHT ].joint_velocities( )
        return np.array( [ tmp_pos[ key ] for key in C.RIGHT_JOINT_NAMES ] ), np.array( [ tmp_vel[ key ] for key in C.RIGHT_JOINT_NAMES ] )


    def get_joint_ref_traj( self, pose1, pose2, D, t ):
        # Generate the joint trajectory for the control
        # We can define a function handle

        # First of all, get the values of the pose1, pose2 dictionary
        pose_init  = np.array( [ pose1[ key ] for key in C.RIGHT_JOINT_NAMES ] )
        pose_final = np.array( [ pose2[ key ] for key in C.RIGHT_JOINT_NAMES ] )

        # Define the function that gets the time input, and
        # Trim the t so that if it is higher than D, then set it as D
        if t >= D:
            t = D

        tt = t/D  # Normalized time, tau is actually the notation but tau is reserved for torque

        q0  = pose_init + ( pose_final - pose_init ) * (  10 * tt ** 3 - 15 * tt ** 4 +  6 * tt ** 5 )
        dq0 =     1.0/D * ( pose_final - pose_init ) * (  30 * tt ** 2 - 60 * tt ** 3 + 30 * tt ** 4 )

        return q0, dq0


    def move2pose( self, which_arm, pose, joint_speed = 0.1 ):



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


    def grip_command( self ):
        self.grip.close()

        rospy.sleep(2)
        self.grip.open()


    def clean_shutdown(self):

        rospy.sleep(2)
        for i in range( 2 ):
            self.arms[ i ].exit_control_mode()


def main():

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser  = argparse.ArgumentParser( formatter_class = arg_fmt )
    parser.add_argument('-s', '--save_data',
                        dest = 'record_data',   action = 'store_true',
                        help = 'Save the Data')

    args = parser.parse_args(rospy.myargv()[1:])


    print("Initializing Controller Node... ")

    rospy.init_node("MY_BAXTER_CONTROL")
    ctrl = BaxterControl( record_data = args.record_data )
    rospy.on_shutdown( ctrl.clean_shutdown )

    # ctrl.joint_state_listener( )

    ctrl.joint_impedances( C.REST_POSE_RIGHT, C.REST_POSE_RIGHT, 3.0 )
    # ctrl.move2pose( C.LEFT , C.MID_POSE_LEFT,   joint_speed = 0.3 )
    # ctrl.move2pose( C.RIGHT , C.REST_POSE_RIGHT,   joint_speed = 0.3 )
    # ctrl.move2pose( C.LEFT  , C.REST_POSE_LEFT ,   joint_speed = 0.3 )

    # ctrl.move2pose( C.RIGHT, C.GRASP_POSE_RIGHT_WIDER,  joint_speed = 0.3 )
    # ctrl.move2pose( C.LEFT,  C.GRASP_POSE_LEFT , joint_speed = 0.3 )
    # ctrl.print_joints( )
    # ctrl.grip_command( )



if __name__ == '__main__':
    main()
