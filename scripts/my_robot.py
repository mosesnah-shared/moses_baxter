#!/usr/bin/python3
import rospy

# BAXTER Libraries
import baxter_interface
from   baxter_pykdl      import baxter_kinematics

# LOCAL Libraries
from my_utils import GripperConnect

class Baxter( object ):

    def __init__( self, args ):

        # Initialzing a rospy node
        rospy.init_node( "impedance_control"   )

        self.args         = args
        self.start_time   = rospy.Time.now( )
        
        # Saving the crucial objects for the code
        self.arms  = { limb_name : baxter_interface.limb.Limb( limb_name ) for limb_name in [ "right", "left" ]  }
        self.kins  = { limb_name : baxter_kinematics( limb_name )          for limb_name in [ "right", "left" ]  }
        self.grips = { limb_name : baxter_interface.Gripper(   limb_name ) for limb_name in [ "right", "left" ]  }

        # The Control Rate of the Robot
        self.rate        = 100.0   # Hz
        self.missed_cmds = 20.0    # Missed cycles before triggering timeout

        self.robot_init( )

    def robot_init( self ):
        
        print( "[LOG] [INIT IN PROGRESS] ....." )

        self.rs = baxter_interface.RobotEnable( baxter_interface.CHECK_VERSION ) 
        
        # Check whether the state is enabled, this value is usually False
        self.init_state = self.rs.state( ).enabled 

        # Initializing the gripper
        # [Moses C. Nah] You need to separately save the output to use the grippers
        #                Hence, adding this redundant line of Code
        self.grip_ctrls = [ GripperConnect( arm ) for arm in [ "right", "left" ] ]
        
        # Enable the Robot
        self.rs.enable( )

        print( "[LOG] [INIT COMPLETE] Ctrl-c to quit" )
        
        # Setting up the control rate.
        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot will timeout and disable
        # Regardless of which_arm, setting both the left and right arm set_command
        self.control_rate = rospy.Rate( self.rate )      

        for limb_name in [ "right", "left" ]: 
            self.arms[ limb_name ].set_command_timeout( ( 1.0 / self.rate ) * self.missed_cmds )
        
        rospy.on_shutdown( self.clean_shutdown )


    def clean_shutdown( self ):

        print( "[LOG] [SHUTTING DOWN ROBOT]" )

        for limb_name in [ "right", "left" ]: 
            self.arms[ limb_name ].exit_control_mode( ) 

        # [2022.07.29] [Note]
        # Not sure why the following if statement is required, since self.init_state is False and self.rs.state( ).enabled is True
        if not self.init_state and self.rs.state( ).enabled:
            self.rs.disable()

    def get_arm_pose( self, which_arm: str ):
        assert which_arm in [ "right", "left" ]
        return self.arms[ which_arm  ].joint_angles( )

    def get_arm_velocity( self, which_arm: str ):
        assert which_arm in [ "right", "left" ]
        return self.arms[ which_arm  ].joint_velocities( )

    def get_gripper_pos( self, which_arm: str ) :
        assert which_arm in [ "right", "left" ]
        return self.grips[ which_arm ].position( )

    def open_gripper( self  ):
        self.grips[ "right" ].open(  block = False )
        self.grips[  "left" ].open(  block = False )

    def close_gripper( self ):
        self.grips[ "right" ].close(  block = False )
        self.grips[  "left" ].close(  block = False )

    def get_end_effector_pose( self, which_arm : str ):
        """
            pose = {'position': (x, y, z), 'orientation': (x, y, z, w)}
        """
        assert which_arm in [ "right", "left" ]
        return self.arms[ which_arm ].endpoint_pose( )
        
    def get_end_effector_velocity( self, which_arm : str ):
        """
            twist = {'linear': (x, y, z), 'angular': (x, y, z )} 
        """
        assert which_arm in [ "right", "left" ]
        return self.arms[ which_arm ].endpoint_velocity( )        


# For Debugging
if __name__ == "__main__":
     
    my_baxter = Baxter( None )

