import mujoco_py as mjPy

from modules.constants    import Constants as C

import rospy


# BAXTER Libraries
import baxter_interface
import baxter_external_devices          # For keyboard interrupt
from   baxter_interface           import CHECK_VERSION
from   baxter_pykdl               import baxter_kinematics



class Baxter( object ):
    def __init__( self, record_data = False ):

        self.record_data = record_data           # Boolean, data saved
                                                 # Just define the whole arms in the first place to simplify the code
        self.arms        = list( range( 2 ) )    # Since we have both the left/right arms
        self.kins        = list( range( 2 ) )    # Since we have both the left/right arms
        self.grips       = list( range( 2 ) )    # Since we have both the left/right arms

        for idx, name in enumerate( [ "right", "left" ] ):
            self.arms[  idx ] = baxter_interface.limb.Limb( name )
            self.kins[  idx ] = baxter_kinematics(          name )
            self.grips[ idx ] = baxter_interface.Gripper(   name )

            # Initialize the gripper
            # [MOSES] [2021.10.31]
            # There might be a way to initialize the gripper rather than running ''roslaunch moses gripper_setup.launch''.
            # Hence, leaving the comments here for future task. Refer to ``gripper_cuff_control.py'' and ``gripper_setup.launch''.
            # self.grips[ idx ].on_type_changed.connect( self.check_calibration )

        self.robot_init( )

    def robot_init( self ):
        self.rs         = baxter_interface.RobotEnable( CHECK_VERSION )
        self.init_state = self.rs.state().enabled

        print("Enabling robot... ")
        self.rs.enable()

        print("Running. Ctrl-c to quit")

    def clean_shutdown(self):
        """
            Switches out of joint torque mode to exit cleanly
        """

        # print( "\nExiting example..." )

        for i in range( 2 ):
            self.arms[ i ].exit_control_mode()

        # if not self.init_state and self.rs.state().enabled:
        self.rs.disable()
        print( "Disabling robot..." )

def main( ):

    model_name = "baxter.xml"

    mjModel  = mjPy.load_model_from_path( C.MODEL_DIR + model_name )      #  Loading xml model as and save it as "model"
    mjSim    = mjPy.MjSim( mjModel )                                    # Construct the simulation environment and save it as "sim"
    mjData   = mjSim.data                                               # Construct the basic MuJoCo data and save it as "mjData"
    mjViewer = mjPy.MjViewerBasic( mjSim )

    # Run Baxter ROS to get the data
    print("Initializing node... ")
    rospy.init_node("impedance_control_right" )
    my_baxter = Baxter( )

    run_time    = 100
    t           = 0
    sim_step    = 0
    update_rate = 60
    dt          = mjModel.opt.timestep

    while t <= run_time:

        if sim_step % update_rate == 0:

            mjViewer.render( )                                     # Render the simulation


        q_R   = my_baxter.arms[ C.RIGHT ].joint_angles()
        q_L   = my_baxter.arms[ C.LEFT  ].joint_angles()

        # Setting the joint angles
        for joint in C.JOINT_NAMES:

            right_name = "right_" + joint             # [Example] "right_s0"
            left_name  =  "left_" + joint             # [Example] "left_s0"

            idxR = mjModel.joint_name2id( right_name )
            idxL = mjModel.joint_name2id(  left_name )

            mjData.qpos[ idxR ] = q_R[ right_name ]
            mjData.qpos[ idxL ] = q_L[  left_name ]

        mjSim.forward( )
        # mjSim.step( )

        t += dt

        sim_step += 1

    rospy.on_shutdown( my_baxter.clean_shutdown )


if __name__ == "__main__":
    main(  )
