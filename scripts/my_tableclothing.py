#!/usr/bin/env python3

# =========================================================== #
# [Author] Moses C. Nah
# [Email]  mosesnah@mit.edu
# [Date]   2021.10.23
# Code developed and improved from David and Zelin's code.
# =========================================================== #

# This is a heavily revised version of the first shoeshining script.
# 1.  The manipulator is brought to its nominal position above the shoe
# 2.  The positioning impedance controller is switched on
# 3.  The positioning impedance controller brings the arm downwards to the cloth
#     z level.
# 4.  The positioning impedance controller brings the arm forwards to the cloth
#     x position.
# 5.  The grippers close around the cloth.
# 6.  The positioning impedance controller brings the cloth up to the start position
# 7.  The positioning impedance controller brings the cloth back to the start point
# 8.  The shoeshining impedance controller then switches on (we can jump)
# 9.  The cloth is tensioned
# 10. The cloth is brought down against the shoe
# 11. The shining commences.
# 12. When the user hits the stop button, the shining amplitude decreases
#     untill it stops
# 13. Grippers release
# 14. Program stops.

import rospy
import baxter_interface
import numpy        as np
import std_msgs.msg as msg
import threading

from   std_msgs.msg       import UInt16, Empty
from   baxter_interface   import CHECK_VERSION
from   numpy.linalg       import inv, pinv
from   baxter_pykdl       import baxter_kinematics
from   tf.transformations import ( quaternion_from_euler , quaternion_matrix   ,
                                   quaternion_inverse    , quaternion_multiply )


# [BACKUP]
# import math
# from tf.transformations import (euler_from_quaternion, quaternion_from_euler)

class TableCloth( object ):

    def __init__(self):

        # WORKSPACE GEOMETRY PARAMETERS:
        # There are 3 total positions:
        # 1. Start position (located above the shoe)
        # 2. Grasp position (where the robot grasps the cloth in front of the shoe)
        # 3. Task position (nominal position where the shoeshining oscillations)

        self.x_start = 0.93
        #self.y_gap = 0.380
        self.y_gap = 0.600
        self.z_start = 0.22  #(0.080 + 0.14)

        self.x_grasp = 1.10
        self.z_grasp = 0.04 #(Started 30cm above table, we want to be 9cm above table)
        # The left and right arms have some z error. We will treat the left arm as our baseline,
        # and add an offset for the right arm
        self.right_z_grasp_offset = -0.035

        self.x_task_start = self.x_grasp
        self.z_task_start = self.z_grasp # Nominal z shoeshine position
        self.x_task_amp = 0.30
        self.z_task_amp = 0.30


        # self.z_task_end = 0.280

        # Desired orientation quaternion ([x y z w]) - The order may be iffy on this one, but it works
        # ([+/-0.5,0.5,+/-0.5,0.5]) is outwards with cameras facing in.
        gripper_incline_angle = 15.0 # (degrees)
        self.right_desired_orientation = quaternion_from_euler(np.pi/2 + np.deg2rad(gripper_incline_angle), 0.0, np.pi/2)
        self.left_desired_orientation = quaternion_from_euler(-np.pi/2 - np.deg2rad(gripper_incline_angle), 0.0, -np.pi/2)

        # Motion Timing:
        self.grasp_motion_x_time = 3.0 #(sec). Time to move along x for grasp
        self.grasp_motion_z_time = 2.5 #(sec). Time to move along z for grasp
        self.grasp_dwell_time = 1.5 #(sec). Time for grippers to grab onto cloth.
        self.grasp_motion_z_time2 = 5  # (sec). Time to move along z for grasp

        self.y_tension_time = 5.0 #(sec) Time to tension cloth in y before shining
        self.z_tension_time = 5.0 # (sec) Time to tension cloth in z before shining
        self.clothing_time = 2.0 # (sec) Time to wave cloth

        self.z_limbdown_time = 2.0 # (sec) Time to put the limb down

        # Shining motion parameters
        motion_angle = 85.0 # (degrees) This is the angle of motion in the ZY plane, relative to the Y axis
        self.motion_amplitude = 0.07 # (meters)
        self.motion_frequency = 1.5 # (Hz) (1.25 Hz is standard demo speed)

        self.rampdown_duration = 10.0 #(sec) Time to rampdown shining amplitude

        # Compute projected components along Y and Z axes:
        self.motion_y_amp = self.motion_amplitude * np.cos(np.deg2rad(motion_angle))
        self.motion_z_amp = self.motion_amplitude * np.sin(np.deg2rad(motion_angle))

        # Time Helper Variables and Flags (None until set):
        self.grasp_start_time = None
        self.cloth_grasped = False
        self.task_start_time = None
        self.rampdown_start_time = None
        self.task_complete = False

        ################################

        # CONTROLLER SETTINGS:
        self.go_to_neutral = True # Go to neutral position at the beginning of script

        # Projection settings. If everything is false, it will just superimpose
        # with no projection.
        self.right_dynamic_nullspace_projection = False
        self.right_static_nullspace_projection = False
        self.right_zero_joint_stiffness = False

        self.left_dynamic_nullspace_projection = False
        self.left_static_nullspace_projection = False
        self.left_zero_joint_stiffness = False

        ################################

        # IMPEDANCE MATRICES:
        # Grasping motion impedance matrices:
        # Cartesian Stiffness
        self.grasp_KX = np.matrix(np.diag([450, 400, 450, 5, 5, 5]))

        # Cartesian Damping
        self.grasp_BX = np.matrix(np.diag([10, 10, 10, 0.3, 0.5, 0.8]))

        # Joint Space Stiffness
        self.grasp_Kq = np.matrix(np.diag([0.5, 4, 5, 0.7, 5, 0.08, 2]))

        # Joint Space Damping
        self.grasp_Bq = np.matrix(np.diag([1, 4, 2, 1, 0.6, 0.9, 0.9]))

        # Y Pretensioning Cartesian Stiffness Matrix:
        # Pretension with 4 Newtons on either side. This is the same as implementing a
        # 40 N/m spring on the y, and stretching it by 10 cm.
        self.tension_KX = np.matrix(np.diag([0, 40, 0, 0, 0, 0]))
        self.tension_distance = 0.15 # (meters)

        # Shoeshining Impedances:
        # Shoeshining Operational Stiffness: (Relative to desired cartesian Trajextory)
        # Default[200, 250, 250, 5, 8, 5]
        self.task_KX = np.matrix(np.diag([200, 150, 150, 5, 8, 5]))

        # Shoeshining Operational Space Damping (Relative to desired Trajectory)
        self.task_BX = np.matrix(np.diag([10, 10, 10, 0.3, 0.5, 0.8]))

        # Joint Space Stiffness,
        # Default [0.005, 12, 5, 0.005, 0.005, 0.005, 0.005]
        self.task_Kq = np.matrix(np.diag([0.005, 12, 5, 0.005, 0.1, 0.1, 0.005]))

        # Joint Space Damping
        self.task_Bq = np.matrix(np.diag([1, 4, 2, 1, 0.6, 0.9, 0.9]))

        ################################

        # INITIAL JOINT GUESSES:
        # Load initial joint states.
        # (rough estimate. Inverse kinematics will pick it up from here):
        self.right_init = {'right_s0': 0.387,
                  'right_s1': -0.624,
                  'right_e0': 0.662,
                  'right_e1': 1.648,
                  'right_w0': 0.232,
                  'right_w1': -0.704,
                  'right_w2': 0.885}

        self.left_init = {'left_s0': -0.387,
                 'left_s1': -0.624,
                 'left_e0': -0.662,
                 'left_e1': 1.648,
                 'left_w0': -0.232,
                 'left_w1': -0.704,
                 'left_w2': -0.885}

        ################################

        # CONTROLLER SETUP
        # Controller Rates
        self.joint_publish_rate = 1000.0 # Hz
        self.controller_rate = 520.0 # Hz
        self.missed_cmds = 40.0 # Missed cycles before triggering timeout

        # Set joint state publishing rate publisher
        rospy.loginfo("Setting robot joint publishing rate... ")
        self.joint_state_pub = rospy.Publisher('robot/joint_state_publish_rate',
                                        UInt16, queue_size=10)

        # Create Cuff Disable Publisher
        rospy.loginfo("Setting up cuff_publisher... ")
        self.right_cuff_ns = 'robot/limb/right/suppress_cuff_interaction'
        self.right_cuff_disable_pub = rospy.Publisher(self.right_cuff_ns,Empty, queue_size=1)
        self.left_cuff_ns = 'robot/limb/left/suppress_cuff_interaction'
        self.left_cuff_disable_pub = rospy.Publisher(self.left_cuff_ns, Empty, queue_size=1)

        # Create Collision Avoidance Publisher
        self.right_collision_ns = '/robot/limb/right/suppress_collision_avoidance'
        self.right_collision_disable_pub = rospy.Publisher(self.right_collision_ns, Empty, queue_size=1)
        self.left_collision_ns = '/robot/limb/left/suppress_collision_avoidance'
        self.left_collision_disable_pub = rospy.Publisher(self.left_collision_ns, Empty, queue_size=1)

        # Create objects for baxter arms, joint names, and kinematics
        self.left_arm = baxter_interface.limb.Limb("left")
        self.right_arm = baxter_interface.limb.Limb("right")
        self.left_kin = baxter_kinematics('left')
        self.right_kin = baxter_kinematics('right')
        self.left_names = self.left_arm.joint_names()
        self.right_names = self.right_arm.joint_names()
        self.left_grip = baxter_interface.Gripper("left")
        self.right_grip = baxter_interface.Gripper("right")

        ################################
        # Maxmium Speed Issues
        self.filepath_base = "/home/baxterbuddy/ros_ws/src/newmanlab_code/zelin/SpeedData/"
        self.speed_origin_file = open(self.filepath_base + "speed_joint_origin.txt", "w+")
        self.speed_percent_file = open(self.filepath_base +"speed_joint_percent.txt", "w+")
        self.speed_endfactor_max_file = open(self.filepath_base +"speed_endfactor_maximum.txt", "w+")
        self.speed_endfactor_real_file = open(self.filepath_base +"speed_endfactor_origin.txt", "w+")
        self.speed_endfactor_per_file = open(self.filepath_base +"speed_endfactor_percent.txt", "w+")
        self.speed_time_file = open(self.filepath_base +"speed_time_record.txt", "w+")

        self.speed_origin_file.seek(0)
        self.speed_origin_file.truncate()
        self.speed_percent_file.seek(0)
        self.speed_percent_file.truncate()
        self.speed_endfactor_max_file.seek(0)
        self.speed_endfactor_max_file.truncate()
        self.speed_endfactor_real_file.seek(0)
        self.speed_endfactor_real_file.truncate()
        self.speed_endfactor_per_file.seek(0)
        self.speed_endfactor_per_file.truncate()
        self.speed_time_file.seek(0)
        self.speed_time_file.truncate()



        self.speed_joint_max =np.matrix([[2.0, 2.0, 2.0, 2.0, 4.0, 4.0, 4.0]])
        self.speed_joint_origin = []
        # self.speed_joint_difference = []
        self.speed_joint_percent = []

        self.speed_endfactor_max = []
        self.speed_endfactor_real = []
        self.speed_endfactor_percent = []
        self.speed_time = []
        ################################

        # GRIPPER SETUP:
        # Note: At this point, the grippers are already calibrated.
        self.left_grip.set_holding_force(100)
        self.right_grip.set_holding_force(100)

        ################################

        # ENABLE ROBOT
        rospy.loginfo("Getting robot state... ")
        self.rs = baxter_interface.RobotEnable( CHECK_VERSION )
        self.init_state = self.rs.state().enabled
        rospy.loginfo("Enabling robot... ")
        self.rs.enable()

    def move(self):
        # Compute the Equilibrium Angles
        self.compute_equilibrium_angles()

        # Set the joint state publishing rate:
        self.joint_state_pub.publish(self.joint_publish_rate)

        # Start seperate thread for disabling cuff interaction and disabling collision avoidance
        cuff_collision_avoidance_thread = threading.Thread(target=self.disable_cuff_collision_avoidance)
        cuff_collision_avoidance_thread.daemon = True
        cuff_collision_avoidance_thread.start()

        # First, position the arms in the start position.
        self.move_to_start_position()

        self.left_grip.open(block=False)
        self.right_grip.open(block=False)

        # Set the timeout:
        # If the specified number of command cycles are missed, the robot will
        # shut down. This is for safety.
        self.right_arm.set_command_timeout((1.0 / self.controller_rate) * self.missed_cmds)
        self.left_arm.set_command_timeout((1.0 / self.controller_rate) * self.missed_cmds)

        # Start the grasp impedance controller:
        rospy.loginfo("Starting cloth grasping controller:")
        self.grasp_impedance_controller()

        # Start the task impedance controller:
        rospy.loginfo("Starting table clothing controller:")
        self.task_impedance_controller()
        return

    def compute_equilibrium_angles(self):
        # Transform dict into a simple list to feed into the IK solver:
        # Use a list comprehension:
        right_seed = [self.right_init.get(name) for name in self.right_names]
        left_seed = [self.left_init.get(name) for name in self.left_names]

        # Desired start point:
        r_X_desired = [self.x_start, -0.5*self.y_gap, self.z_start]
        l_X_desired = [self.x_start, 0.5*self.y_gap, self.z_start]

        # Compute the inverse kinematics:
        r_eq = self.right_kin.inverse_kinematics(r_X_desired, self.right_desired_orientation.tolist(), right_seed)
        l_eq = self.left_kin.inverse_kinematics(l_X_desired, self.left_desired_orientation.tolist(), left_seed)

        # Reformat the lists as column vectors, and store them
        self.r_equilibrium = np.matrix(r_eq).transpose()
        self.l_equilibrium = np.matrix(l_eq).transpose()

    def disable_cuff_collision_avoidance(self):
        # Suggested pubish rate for collision avoidance setting is 10 Hz
        rate = rospy.Rate(10)

        # This runs in a seperate thread, so we can go on untill program shutdown:
        while not rospy.is_shutdown():
            self.right_collision_disable_pub.publish(Empty())
            self.left_collision_disable_pub.publish(Empty())
            self.right_cuff_disable_pub.publish()
            self.left_cuff_disable_pub.publish()
            rate.sleep()

    def move_to_start_position(self):
        # Start with a move to Neutral command
        if self.go_to_neutral:
            self.right_arm.move_to_neutral()
            self.left_arm.move_to_neutral()

        # Move robot to the computed equilibrium position
        r_angles = dict(zip(self.right_names, self.r_equilibrium))
        l_angles = dict(zip(self.left_names, self.l_equilibrium))
        self.right_arm.move_to_joint_positions(r_angles, 20.0)
        self.left_arm.move_to_joint_positions(l_angles, 20.0)

    def grasp_impedance_controller(self):
        # Set the loop rate:
        rate = rospy.Rate(self.controller_rate)

        # Compute total time for the grasping action
        grasp_controller_duration = self.grasp_motion_x_time + self.grasp_motion_z_time + 2*self.grasp_dwell_time
        self.grasp_start_time = rospy.Time.now()
        elapsed_time = 0.0

        while elapsed_time <= grasp_controller_duration:
            elapsed_time = (rospy.Time.now() - self.grasp_start_time).to_sec()

            # Compute torque commands (returns dicts):
            right_torques, left_torques = self.get_grasp_torque_command(elapsed_time)

            #Command torques to arm:
            self.right_arm.set_joint_torques(right_torques)
            self.left_arm.set_joint_torques(left_torques)
            rate.sleep()

    def task_impedance_controller(self):
        # Set the loop rate:
        rate = rospy.Rate(self.controller_rate)

        # This thread allows us to gracefully stop shoeshining and exit the thread:
        # def get_user_input():
        #     raw_input("\nStarting to Shoeshine. Press Enter to ramp down and stop:")
        #     # Activate rampdown at this point:
        #     self.rampdown_start_time = rospy.Time.now()
        #     # Block until task_complete flag is set to True
        #     rate = rospy.Rate(self.controller_rate)
        #     while self.task_complete is False:
        #         rate.sleep()
        #     return0
        #
        # user_input_thread = threading.Thread(target=get_user_input)
        # # Set thread as a daemon thread, so it gets killed if the program exits
        # user_input_thread.daemon = True
        # user_input_thread.start()

        # Set the timeout:
        # If the specified number of command cycles are missed, the robot will
        # shut down. This is for safety.
        #self.right_arm.set_command_timeout((1.0 / self.controller_rate) * self.missed_cmds)
        #self.left_arm.set_command_timeout((1.0 / self.controller_rate) * self.missed_cmds)

        # Start the clock
        task_controller_duration = self.y_tension_time + self.clothing_time + 1
        self.task_start_time = rospy.Time.now()
        elapsed_time = 0.0
        while elapsed_time <= task_controller_duration:
            # Compute elasped time:
            elapsed_time = (rospy.Time.now() - self.task_start_time).to_sec()

            # Compute torque commands (returns dicts):
            right_torques, left_torques = self.get_task_torque_command(elapsed_time)

            #Command torques to arm:
            self.right_arm.set_joint_torques(right_torques)
            self.left_arm.set_joint_torques(left_torques)
            rate.sleep()

        rospy.loginfo("Speed Data Processing...")
        self.speed_data_process()
        self.speed_data_print()


    ###########################################
    # Grasp Controller torque/position commands
    ###########################################baxterbuddy@baxterbuddy-ThinkPad-W540:~$ cd ros_ws/


    def get_grasp_torque_command(self, time):
        # Get current robot state
        r_x_current, l_x_current = self.get_X_current()
        r_xdot_current, l_xdot_current = self.get_Xdot_current()

        r_q_current,l_q_current, r_q_dict, l_q_dict = self.get_q_current()
        r_qdot_current, l_qdot_current = self.get_qdot_current()

        # Get desired endpoint state (Augmented with tension desired endpoint)
        r_x_desired, l_x_desired = self.get_X_grasp(time)
        r_xdot_desired, l_xdot_desired = self.get_Xdot_grasp(time)

        # Obtain Jacobians
        r_jacobian = self.right_kin.jacobian(joint_values=r_q_dict)
        l_jacobian = self.left_kin.jacobian(joint_values=l_q_dict)

        # Obtain Inverse Inertia Matrices
        if self.right_dynamic_nullspace_projection:
            r_Mq_inv = inv(self.right_kin.inertia(joint_values=r_q_dict))
            l_Mq_inv = inv(self.left_kin.inertia(joint_values=l_q_dict))

        # Compute the torques for the right arm first.
        # First compute the forces felt at the endpoint from the virtual springs
        # Initialize a numpy matrix for this purpose.
        r_endpoint_force = np.matrix([[0.0],
                                      [0.0],
                                      [0.0],
                                      [0.0],
                                      [0.0],
                                      [0.0]])

        # Compute linear part first
        r_endpoint_force[0:3,0] = self.grasp_KX[0:3,0:3]*(r_x_desired[0:3,0] - r_x_current[0:3,0]) + \
                                  self.grasp_BX[0:3,0:3]*(r_xdot_desired[0:3,0] - r_xdot_current[0:3,0])

        # Now, compute rotational part
        # Method: Caccavale_2008 Six-DOF Impedance Control of Dual Arm Cooperative Manipulators
        # Note: Quaternion multiply takes in lists or flat np arrays
        Q_current = [r_x_current[3,0], r_x_current[4,0], r_x_current[5,0], r_x_current[6,0]]
        Q_dr = quaternion_multiply(quaternion_inverse(Q_current), self.right_desired_orientation)

        # R_r is the rotation matrix of the current orientaiton with respect to the base frame
        # Note: quaternion_matrix takes in a flat list, and outputs the homogeneous tranformation matrix
        # we just want the upper left 3x3 of the result.
        R_r = np.asmatrix(quaternion_matrix(Q_current)[0:3,0:3])

        eta_dr = Q_dr[3]
        epsilon_r_dr = np.matrix([[Q_dr[0]],
                                  [Q_dr[1]],
                                  [Q_dr[2]]])

        epsilon_dr = R_r.transpose()*epsilon_r_dr


        K_prime = 2*(eta_dr*np.asmatrix(np.eye(3))  - skew_symm(epsilon_dr)).transpose() * R_r * self.grasp_KX[3:6,3:6] * R_r.transpose()
        r_endpoint_force[3:6,0] = K_prime*epsilon_dr + self.grasp_BX[3:6,3:6]*(r_xdot_desired[3:6,0] - r_xdot_current[3:6,0])

        # Joint torques resulting from endpoint virtual springs
        r_endpoint_torque = np.dot(r_jacobian.T, r_endpoint_force)

        # Torques from joint virtual spring/dampers:
        r_joint_torque = self.grasp_Kq*(self.r_equilibrium - r_q_current) - self.grasp_Bq*r_qdot_current

        # Perform nullspace projection, based on controller settings:
        if self.right_dynamic_nullspace_projection:
            r_joint_torque_proj = np.dot( (np.eye(7) - np.dot(r_jacobian.T, gen_inv(r_jacobian, r_Mq_inv).T)), r_joint_torque)
        elif self.right_static_nullspace_projection:
            r_joint_torque_proj = np.dot( (np.eye(7) - np.dot(r_jacobian.T, pinv(r_jacobian.T))), r_joint_torque)
        elif self.right_zero_joint_stiffness:
            r_joint_torque_proj = 0.0*r_joint_torque
        else:
            r_joint_torque_proj = r_joint_torque

        # Sum the torques from the joint and enpoint virtual springs
        r_torque_cmd_mat = r_endpoint_torque + r_joint_torque_proj

        # Return result as a dict, using a dict comprehension
        r_torque_cmd = {name:np.asscalar(torque) for name,torque in zip(self.right_names, r_torque_cmd_mat)}

        # Compute for left arm
        # Force from virtual springs on endpoint
        # First compute the forces felt at the endpoint from the virtual springs
        l_endpoint_force = np.matrix([[0.0],
                                      [0.0],
                                      [0.0],
                                      [0.0],
                                      [0.0],
                                      [0.0]])

        # Compute linear part first
        l_endpoint_force[0:3,0] = self.grasp_KX[0:3,0:3]*(l_x_desired[0:3,0] - l_x_current[0:3,0]) + self.grasp_BX[0:3,0:3]*(l_xdot_desired[0:3,0] - l_xdot_current[0:3,0])

        # Now, compute rotational part
        # Method: Caccavale_2008 Six-DOF Impedance Control of Dual Arm Cooperative Manipulators
        # Note: Quaternion multiply takes in lists or flat np arrays
        Q_current = [l_x_current[3,0], l_x_current[4,0], l_x_current[5,0], l_x_current[6,0]]
        Q_dr = quaternion_multiply(quaternion_inverse(Q_current), self.left_desired_orientation)

        # R_r is the rotation matrix of the current orientaiton with respect to the base frame
        # Note: quaternion_matrix takes in a flat list, and outputs the homogeneous tranformation matrix
        # we just want the upper left 3x3 of the result.
        R_r = np.asmatrix(quaternion_matrix(Q_current)[0:3,0:3])

        eta_dr = Q_dr[3]
        epsilon_r_dr = np.matrix([[Q_dr[0]],
                                  [Q_dr[1]],
                                  [Q_dr[2]]])

        epsilon_dr = R_r.transpose()*epsilon_r_dr


        K_prime = 2*(eta_dr*np.asmatrix(np.eye(3))  - skew_symm(epsilon_dr)).transpose() * R_r * self.grasp_KX[3:6,3:6] * R_r.transpose()
        l_endpoint_force[3:6,0] = K_prime*epsilon_dr + self.grasp_BX[3:6,3:6]*(l_xdot_desired[3:6,0] - l_xdot_current[3:6,0])

        # Joint torques resulting from endpoint virtual springs
        l_endpoint_torque = np.dot(l_jacobian.T, l_endpoint_force)

        # Torques from joint virtual springs:
        l_joint_torque = self.grasp_Kq*(self.l_equilibrium - l_q_current) - self.grasp_Bq*l_qdot_current

        # Perform nullspace projection, based on controller settings:
        if self.left_dynamic_nullspace_projection:
            l_joint_torque_proj = np.dot( (np.eye(7) - np.dot(l_jacobian.T, gen_inv(l_jacobian, l_Mq_inv).T)), l_joint_torque)
        elif self.left_static_nullspace_projection:
            l_joint_torque_proj = np.dot( (np.eye(7) - np.dot(l_jacobian.T, pinv(l_jacobian.T))), l_joint_torque)
        elif self.left_zero_joint_stiffness:
            l_joint_torque_proj = 0.0*l_joint_torque
        else:
            l_joint_torque_proj = l_joint_torque

        # Sum the torques from the joint and enpoint virtual springs
        l_torque_cmd_mat = l_endpoint_torque + l_joint_torque_proj

        # Return result as a dict, using a dict comprehension
        l_torque_cmd = {name:np.asscalar(torque) for name,torque in zip(self.left_names,l_torque_cmd_mat)}

        return r_torque_cmd, l_torque_cmd

    def get_X_grasp(self, time):
        # Returns desired endpoint X position, and desired tension position
        # Part 1: bring arm downwards
        if time <= self.grasp_motion_z_time:
            r_X_desired = np.matrix([[self.x_start],
                                     [(-0.5)*self.y_gap],
                                     [self.z_start - (time/self.grasp_motion_z_time)*(self.z_start - (self.z_grasp + self.right_z_grasp_offset))],
                                     [self.right_desired_orientation[0]],
                                     [self.right_desired_orientation[1]],
                                     [self.right_desired_orientation[2]],
                                     [self.right_desired_orientation[3]]])

            l_X_desired = np.matrix([[self.x_start],
                                     [(0.5)*self.y_gap],
                                     [self.z_start - (time/self.grasp_motion_z_time)*(self.z_start - self.z_grasp)],
                                     [self.left_desired_orientation[0]],
                                     [self.left_desired_orientation[1]],
                                     [self.left_desired_orientation[2]],
                                     [self.left_desired_orientation[3]]])
            return r_X_desired, l_X_desired

        # Part 2: bring arm towards cloth:
        if time <= (self.grasp_motion_z_time + self.grasp_motion_x_time):
            time = time - self.grasp_motion_z_time
            r_X_desired = np.matrix([[self.x_start + (time/self.grasp_motion_x_time)*(self.x_grasp - self.x_start)],
                                     [(-0.5)*self.y_gap],
                                     [(self.z_grasp + self.right_z_grasp_offset)],
                                     [self.right_desired_orientation[0]],
                                     [self.right_desired_orientation[1]],
                                     [self.right_desired_orientation[2]],
                                     [self.right_desired_orientation[3]]])

            l_X_desired = np.matrix([[self.x_start + (time/self.grasp_motion_x_time)*(self.x_grasp - self.x_start)],
                                     [(0.5)*self.y_gap],
                                     [self.z_grasp],
                                     [self.left_desired_orientation[0]],
                                     [self.left_desired_orientation[1]],
                                     [self.left_desired_orientation[2]],
                                     [self.left_desired_orientation[3]]])

            return r_X_desired, l_X_desired

        # Part 3: Dwell before grasping cloth:
        if time <= (self.grasp_motion_z_time + self.grasp_motion_x_time + self.grasp_dwell_time):

            time = time - (self.grasp_motion_z_time + self.grasp_motion_x_time)
            r_X_desired = np.matrix([[self.x_grasp],
                                     [(-0.5)*self.y_gap],
                                     [(self.z_grasp + self.right_z_grasp_offset)],
                                     [self.right_desired_orientation[0]],
                                     [self.right_desired_orientation[1]],
                                     [self.right_desired_orientation[2]],
                                     [self.right_desired_orientation[3]]])

            l_X_desired = np.matrix([[self.x_grasp],
                                     [(0.5)*self.y_gap],
                                     [self.z_grasp],
                                     [self.left_desired_orientation[0]],
                                     [self.left_desired_orientation[1]],
                                     [self.left_desired_orientation[2]],
                                     [self.left_desired_orientation[3]]])

            return r_X_desired, l_X_desired

        # Part 4: Bring arms up:
        # if time <= (self.grasp_motion_z_time + self.grasp_motion_x_time + self.grasp_dwell_time +
        #             self.grasp_motion_z_time2):
        #     if not self.cloth_grasped:
        #         self.left_grip.close(block=False)
        #         self.right_grip.close(block=False)
        #         self.cloth_grasped = True
        #     time = time - (self.grasp_motion_z_time + self.grasp_motion_x_time + self.grasp_dwell_time)
        #     r_X_desired = np.matrix([[self.x_grasp],
        #                              [(-0.5)*self.y_gap],
        #                              [(self.z_grasp + self.right_z_grasp_offset) + (time/self.grasp_motion_z_time2)*(self.z_task_start - (self.z_grasp + self.right_z_grasp_offset))],
        #                              [self.right_desired_orientation[0]],
        #                              [self.right_desired_orientation[1]],
        #                              [self.right_desired_orientation[2]],
        #                              [self.right_desired_orientation[3]]])
        #
        #     l_X_desired = np.matrix([[self.x_grasp],
        #                              [(0.5)*self.y_gap],
        #                              [self.z_grasp + (time/self.grasp_motion_z_time2)*(self.z_task_start - self.z_grasp)],
        #                              [self.left_desired_orientation[0]],
        #                              [self.left_desired_orientation[1]],
        #                              [self.left_desired_orientation[2]],
        #                              [self.left_desired_orientation[3]]])
        #
        #     return r_X_desired, l_X_desired

        # Part 5: Bring arms back:
        # if time <= (2*self.grasp_motion_z_time + 2*self.grasp_motion_x_time + self.grasp_dwell_time):
        #     time = time - (2*self.grasp_motion_z_time + self.grasp_motion_x_time + self.grasp_dwell_time)
        #     r_X_desired = np.matrix([[self.x_grasp - (time/self.grasp_motion_x_time)*(self.x_grasp - self.x_start)],
        #                              [(-0.5)*self.y_gap],
        #                              [self.z_start],
        #                              [self.right_desired_orientation[0]],
        #                              [self.right_desired_orientation[1]],
        #                              [self.right_desired_orientation[2]],
        #                              [self.right_desired_orientation[3]]])
        #
        #     l_X_desired = np.matrix([[self.x_grasp - (time/self.grasp_motion_x_time)*(self.x_grasp - self.x_start)],
        #                              [(0.5)*self.y_gap],
        #                              [self.z_start],
        #                              [self.left_desired_orientation[0]],
        #                              [self.left_desired_orientation[1]],
        #                              [self.left_desired_orientation[2]],
        #                              [self.left_desired_orientation[3]]])
        #
        #     return r_X_desired, l_X_desired

        # Part 6: Dwell at start point:
        else:
            if not self.cloth_grasped:
                self.left_grip.close(block=False)
                self.right_grip.close(block=False)
                self.cloth_grasped = True

            r_X_desired = np.matrix([[self.x_grasp],
                                     [(-0.5)*self.y_gap],
                                     [self.z_task_start + self.right_z_grasp_offset],
                                     [self.right_desired_orientation[0]],
                                     [self.right_desired_orientation[1]],
                                     [self.right_desired_orientation[2]],
                                     [self.right_desired_orientation[3]]])

            l_X_desired = np.matrix([[self.x_grasp],
                                     [(0.5)*self.y_gap],
                                     [self.z_task_start],
                                     [self.left_desired_orientation[0]],
                                     [self.left_desired_orientation[1]],
                                     [self.left_desired_orientation[2]],
                                     [self.left_desired_orientation[3]]])

            return r_X_desired, l_X_desired

    def get_Xdot_grasp(self, time):
        # First pass: set it to ground.
        r_Xdot_desired = np.matrix([[0],
                                     [0],
                                     [0],
                                     [0],
                                     [0],
                                     [0]])

        l_Xdot_desired = np.matrix([[0],
                                     [0],
                                     [0],
                                     [0],
                                     [0],
                                     [0]])

        return r_Xdot_desired, l_Xdot_desired

    ###########################################
    # Task Controller torque/position commands
    ###########################################

    def get_task_torque_command(self, time):
        # Get current robot state
        r_x_current, l_x_current = self.get_X_current()
        r_xdot_current, l_xdot_current = self.get_Xdot_current()

        r_q_current,l_q_current, r_q_dict, l_q_dict = self.get_q_current()
        r_qdot_current, l_qdot_current = self.get_qdot_current()

        # Get desired endpoint state (Augmented with tension desired endpoint)
        r_x_desired_aug, l_x_desired_aug = self.get_X_task(time)
        r_xdot_desired, l_xdot_desired = self.get_Xdot_task(time)

        # Extract tension states and regular states from augmented vector
        r_x_tension_desired = np.matrix([[0.0],
                                         [r_x_desired_aug[7,0]],
                                         [0.0],
                                         [0.0],
                                         [0.0],
                                         [0.0]])

        l_x_tension_desired = np.matrix([[0.0],
                                         [l_x_desired_aug[7,0]],
                                         [0.0],
                                         [0.0],
                                         [0.0],
                                         [0.0]])

        r_x_desired = r_x_desired_aug[0:7]
        l_x_desired = l_x_desired_aug[0:7]

        # Obtain Jacobians
        r_jacobian = self.right_kin.jacobian(joint_values=r_q_dict)
        l_jacobian = self.left_kin.jacobian(joint_values=l_q_dict)

        # Obtain Inverse Inertia Matrices
        r_Mq_inv = inv(self.right_kin.inertia(joint_values=r_q_dict))
        l_Mq_inv = inv(self.left_kin.inertia(joint_values=l_q_dict))

        # Record Speed Data
        self.speed_joint_origin.append(r_qdot_current.transpose().tolist()[0])

        speed_enfactor_loc = np.dot(r_jacobian, self.speed_joint_max.transpose())
        self.speed_endfactor_max.append(speed_enfactor_loc.transpose().tolist()[0])
        self.speed_endfactor_real.append(r_xdot_current.transpose().tolist()[0])
        self.speed_time.append(time)

        # self.speed_log_file.write("{0[0]:>8.4f} {0[1]:>8.4f} {0[2]:>8.4f} {0[3]:>8.4f} {0[4]:>8.4f} {0[5]:>8.4f} {0[6]:>8.4f}\n"
        #                           .format(r_qdot_current.transpose().tolist()[0]))

        # Compute the torques for the right arm first.
        # First compute the forces felt at the endpoint from the virtual springs
        # Initialize a numpy matrix for this purpose.
        r_endpoint_force = np.matrix([[0.0],
                                      [0.0],
                                      [0.0],
                                      [0.0],
                                      [0.0],
                                      [0.0]])

        # Compute linear part first
        r_endpoint_force[0:3,0] = self.task_KX[0:3,0:3]*(r_x_desired[0:3,0] - r_x_current[0:3,0]) + \
                                  self.tension_KX[0:3,0:3]*(r_x_tension_desired[0:3,0] - r_x_current[0:3,0]) + \
                                  self.task_BX[0:3,0:3]*(r_xdot_desired[0:3,0] - r_xdot_current[0:3,0])

        # Now, compute rotational part
        # Method: Caccavale_2008 Six-DOF Impedance Control of Dual Arm Cooperative Manipulators
        # Note: Quaternion multiply takes in lists or flat np arrays
        Q_current = [r_x_current[3,0], r_x_current[4,0], r_x_current[5,0], r_x_current[6,0]]
        Q_dr = quaternion_multiply(quaternion_inverse(Q_current), self.right_desired_orientation)

        # R_r is the rotation matrix of the current orientaiton with respect to the base frame
        # Note: quaternion_matrix takes in a flat list, and outputs the homogeneous tranformation matrix
        # we just want the upper left 3x3 of the result.
        R_r = np.asmatrix(quaternion_matrix(Q_current)[0:3,0:3])

        eta_dr = Q_dr[3]
        epsilon_r_dr = np.matrix([[Q_dr[0]],
                                  [Q_dr[1]],
                                  [Q_dr[2]]])

        epsilon_dr = R_r.transpose()*epsilon_r_dr


        K_prime = 2*(eta_dr*np.asmatrix(np.eye(3))  - skew_symm(epsilon_dr)).transpose() * R_r * self.task_KX[3:6,3:6] * R_r.transpose()
        r_endpoint_force[3:6,0] = K_prime*epsilon_dr + self.task_BX[3:6,3:6]*(r_xdot_desired[3:6,0] - r_xdot_current[3:6,0])

        # Joint torques resulting from endpoint virtual springs
        r_endpoint_torque = np.dot(r_jacobian.T, r_endpoint_force)

        # Torques from joint virtual spring/dampers:
        r_joint_torque = self.task_Kq*(self.r_equilibrium - r_q_current) - self.task_Bq*r_qdot_current

        # Perform nullspace projection, based on controller settings:
        if self.right_dynamic_nullspace_projection:
            r_joint_torque_proj = np.dot( (np.eye(7) - np.dot(r_jacobian.T, gen_inv(r_jacobian, r_Mq_inv).T)), r_joint_torque)
        elif self.right_static_nullspace_projection:
            r_joint_torque_proj = np.dot( (np.eye(7) - np.dot(r_jacobian.T, pinv(r_jacobian.T))), r_joint_torque)
        elif self.right_zero_joint_stiffness:
            r_joint_torque_proj = 0.0*r_joint_torque
        else:
            r_joint_torque_proj = r_joint_torque

        # Sum the torques from the joint and enpoint virtual springs
        r_torque_cmd_mat = r_endpoint_torque + r_joint_torque_proj

        # Return result as a dict, using a dict comprehension
        r_torque_cmd = {name:np.asscalar(torque) for name,torque in zip(self.right_names, r_torque_cmd_mat)}

        # Compute for left arm
        # Force from virtual springs on endpoint
        # First compute the forces felt at the endpoint from the virtual springs
        l_endpoint_force = np.matrix([[0.0],
                                      [0.0],
                                      [0.0],
                                      [0.0],
                                      [0.0],
                                      [0.0]])

        # Compute linear part first
        l_endpoint_force[0:3,0] = self.task_KX[0:3,0:3]*(l_x_desired[0:3,0] - l_x_current[0:3,0]) + self.tension_KX[0:3,0:3]*(l_x_tension_desired[0:3,0] - l_x_current[0:3,0]) + self.task_BX[0:3,0:3]*(l_xdot_desired[0:3,0] - l_xdot_current[0:3,0])

        # Now, compute rotational part
        # Method: Caccavale_2008 Six-DOF Impedance Control of Dual Arm Cooperative Manipulators
        # Note: Quaternion multiply takes in lists or flat np arrays
        Q_current = [l_x_current[3,0], l_x_current[4,0], l_x_current[5,0], l_x_current[6,0]]
        Q_dr = quaternion_multiply(quaternion_inverse(Q_current), self.left_desired_orientation)

        # R_r is the rotation matrix of the current orientaiton with respect to the base frame
        # Note: quaternion_matrix takes in a flat list, and outputs the homogeneous tranformation matrix
        # we just want the upper left 3x3 of the result.
        R_r = np.asmatrix(quaternion_matrix(Q_current)[0:3,0:3])

        eta_dr = Q_dr[3]
        epsilon_r_dr = np.matrix([[Q_dr[0]],
                                  [Q_dr[1]],
                                  [Q_dr[2]]])

        epsilon_dr = R_r.transpose()*epsilon_r_dr


        K_prime = 2*(eta_dr*np.asmatrix(np.eye(3))  - skew_symm(epsilon_dr)).transpose() * R_r * self.task_KX[3:6,3:6] * R_r.transpose()
        l_endpoint_force[3:6,0] = K_prime*epsilon_dr + self.task_BX[3:6,3:6]*(l_xdot_desired[3:6,0] - l_xdot_current[3:6,0])

        # Joint torques resulting from endpoint virtual springs
        l_endpoint_torque = np.dot(l_jacobian.T, l_endpoint_force)

        # Torques from joint virtual springs:
        l_joint_torque = self.task_Kq*(self.l_equilibrium - l_q_current) - self.task_Bq*l_qdot_current

        # Perform nullspace projection, based on controller settings:
        if self.left_dynamic_nullspace_projection:
            l_joint_torque_proj = np.dot( (np.eye(7) - np.dot(l_jacobian.T, gen_inv(l_jacobian, l_Mq_inv).T)), l_joint_torque)
        elif self.left_static_nullspace_projection:
            l_joint_torque_proj = np.dot( (np.eye(7) - np.dot(l_jacobian.T, pinv(l_jacobian.T))), l_joint_torque)
        elif self.left_zero_joint_stiffness:
            l_joint_torque_proj = 0.0*l_joint_torque
        else:
            l_joint_torque_proj = l_joint_torque

        # Sum the torques from the joint and enpoint virtual springs
        l_torque_cmd_mat = l_endpoint_torque + l_joint_torque_proj

        # Return result as a dict, using a dict comprehension
        l_torque_cmd = {name:np.asscalar(torque) for name,torque in zip(self.left_names,l_torque_cmd_mat)}

        return r_torque_cmd, l_torque_cmd

    def get_X_task(self, time):
        # Returns desired endpoint X position, and desired tension position
        # Part 1: Tension Rampup
        # x_backward = 0.3
        if time <= self.y_tension_time:
            r_X_desired = np.matrix([[self.x_task_start],
                                     [(-0.5)*self.y_gap],
                                     [self.z_task_start],
                                     [self.right_desired_orientation[0]],
                                     [self.right_desired_orientation[1]],
                                     [self.right_desired_orientation[2]],
                                     [self.right_desired_orientation[3]],
                                     [(-0.5)*self.y_gap - (time/self.y_tension_time)*self.tension_distance]])

            l_X_desired = np.matrix([[self.x_task_start],
                                     [(0.5)*self.y_gap],
                                     [self.z_task_start],
                                     [self.left_desired_orientation[0]],
                                     [self.left_desired_orientation[1]],
                                     [self.left_desired_orientation[2]],
                                     [self.left_desired_orientation[3]],
                                     [(0.5)*self.y_gap + (time/self.y_tension_time)*self.tension_distance]])

            return r_X_desired, l_X_desired

        # Part 2: Pressing the Cloth Against the Shoe
        # if time <= (self.y_tension_time + self.z_tension_time):
        #     time = time - self.y_tension_time
        #     r_X_desired = np.matrix([[self.x_task],
        #                              [(-0.5)*self.y_gap],
        #                              [self.z_start - (time/self.z_tension_time)*(self.z_start - self.z_task)],
        #                              [self.right_desired_orientation[0]],
        #                              [self.right_desired_orientation[1]],
        #                              [self.right_desired_orientation[2]],
        #                              [self.right_desired_orientation[3]],
        #                              [(-0.5)*self.y_gap - self.tension_distance]])
        #
        #     l_X_desired = np.matrix([[self.x_task],
        #                              [(0.5)*self.y_gap],
        #                              [self.z_start - (time/self.z_tension_time)*(self.z_start - self.z_task)],
        #                              [self.left_desired_orientation[0]],
        #                              [self.left_desired_orientation[1]],
        #                              [self.left_desired_orientation[2]],
        #                              [self.left_desired_orientation[3]],
        #                              [(0.5)*self.y_gap + self.tension_distance]])
        #
        #     return r_X_desired, l_X_desired

        # Part 2: Clothing:
        if time <= self.y_tension_time + self.clothing_time:
            time = time - self.y_tension_time

            # Implement Rampdown:
            # if self.rampdown_start_time is not None:
            #     rampdown_time = (rospy.Time.now() - self.rampdown_start_time).to_sec()
            # else:
            #     rampdown_time = 0.0
            # adjusted_amp = 1.0 - np.clip((rampdown_time/self.rampdown_duration), None, 1.0)
            # if (np.isclose(adjusted_amp, 0.0, atol=1E-4) and not self.task_complete):
            #     self.task_complete = True

            r_X_desired = np.matrix([[self.x_task_start + self.x_task_amp *
                                      np.sin(1.5 * np.pi * (time/self.clothing_time)) ],
                                     [(-0.5)*self.y_gap],
                                     [self.z_task_start + self.z_task_amp *
                                      (1 - np.cos(2 * np.pi * (time/self.clothing_time))) ],
                                     [self.right_desired_orientation[0]],
                                     [self.right_desired_orientation[1]],
                                     [self.right_desired_orientation[2]],
                                     [self.right_desired_orientation[3]],
                                     [(-0.5)*self.y_gap - self.tension_distance]])

            l_X_desired = np.matrix([[self.x_task_start + self.x_task_amp *
                                      np.sin(1.5 * np.pi * (time/self.clothing_time)) ],
                                     [(0.5)*self.y_gap],
                                     [self.z_task_start + self.z_task_amp *
                                      (1 - np.cos(2 * np.pi * (time/self.clothing_time))) ],
                                     [self.left_desired_orientation[0]],
                                     [self.left_desired_orientation[1]],
                                     [self.left_desired_orientation[2]],
                                     [self.left_desired_orientation[3]],
                                     [(0.5)*self.y_gap + self.tension_distance]])
            return r_X_desired, l_X_desired

		# Part 3: Bring the limb down:
        # if time > self.y_tension_time + self.clothing_time:
        #     time = time - (self.y_tension_time + self.clothing_time)
        #     r_X_desired = np.matrix([[self.x_task - x_backward],
        #                              [(-0.5)*self.y_gap],
        #                              [self.z_task_start - (self.z_task_start - self.z_task_end) * (time/self.z_limbdown_time)],
        #                              [self.right_desired_orientation[0]],
        #                              [self.right_desired_orientation[1]],
        #                              [self.right_desired_orientation[2]],
        #                              [self.right_desired_orientation[3]],
        #                              [(-0.5)*self.y_gap - self.tension_distance]])
        #
        #     l_X_desired = np.matrix([[self.x_task - x_backward],
        #                              [(0.5)*self.y_gap],
        #                              [self.z_task_start - (self.z_task_start - self.z_task_end) * (time/self.z_limbdown_time)],
        #                              [self.left_desired_orientation[0]],
        #                              [self.left_desired_orientation[1]],
        #                              [self.left_desired_orientation[2]],
        #                              [self.left_desired_orientation[3]],
        #                              [(0.5)*self.y_gap + self.tension_distance]])

        else:
            r_X_desired = np.matrix([[self.x_task_start - self.x_task_amp],
                                     [(-0.5) * self.y_gap],
                                     [self.z_task_start],
                                     [self.right_desired_orientation[0]],
                                     [self.right_desired_orientation[1]],
                                     [self.right_desired_orientation[2]],
                                     [self.right_desired_orientation[3]],
                                     [(-0.5) * self.y_gap - self.tension_distance]])

            l_X_desired = np.matrix([[self.x_task_start - self.x_task_amp],
                                     [(0.5) * self.y_gap],
                                     [self.z_task_start],
                                     [self.left_desired_orientation[0]],
                                     [self.left_desired_orientation[1]],
                                     [self.left_desired_orientation[2]],
                                     [self.left_desired_orientation[3]],
                                     [(0.5) * self.y_gap + self.tension_distance]])

            return r_X_desired, l_X_desired

    def get_Xdot_task(self, time):
        # Returns desired endpoint X velocity (Not Augmented with tension position)
        # Part 1: Tension Rampup
        if time <= self.y_tension_time:
            r_Xdot_desired = np.matrix([[0],
                                     [0],
                                     [0],
                                     [0],
                                     [0],
                                     [0]])

            l_Xdot_desired = np.matrix([[0],
                                     [0],
                                     [0],
                                     [0],
                                     [0],
                                     [0]])

            return r_Xdot_desired, l_Xdot_desired

        # Part 2: Pressing the Cloth Against the Shoe
        # if time <= (self.y_tension_time + self.z_tension_time):
        #     time = time - self.y_tension_time
        #     r_Xdot_desired = np.matrix([[0],
        #                                 [0],
        #                                 [-(self.z_start - self.z_task)/self.z_tension_time],
        #                                 [0],
        #                                 [0],
        #                                 [0]])
        #
        #     l_Xdot_desired = np.matrix([[0],
        #                                 [0],
        #                                 [-(self.z_start - self.z_task)/self.z_tension_time],
        #                                 [0],
        #                                 [0],
        #                                 [0]])
        #
        #     return r_Xdot_desired, l_Xdot_desired

        # Part 3: Oscillations:
        if time > self.y_tension_time:
            time = time - self.y_tension_time
            # # Implement Rampdown:
            # if self.rampdown_start_time is not None:
            #     rampdown_time = (rospy.Time.now() - self.rampdown_start_time).to_sec()
            # else:
            #     rampdown_time = 0.0
            # adjusted_amp = 1.0 - np.clip((rampdown_time/self.rampdown_duration), None, 1.0)
            r_Xdot_desired = np.matrix([[0],
                                     [0],
                                     [0],
                                     [0],
                                     [0],
                                     [0]])

            l_Xdot_desired = np.matrix([[0],
                                     [0],
                                     [0],
                                     [0],
                                     [0],
                                     [0]])

            return r_Xdot_desired, l_Xdot_desired

    def get_X_current(self):
        # Returns the current position in cartesian coordinates, and the
        # current orientation as a quaternion.
        # This returns the values as two numpy column matrices
        # i.e. np.matrix([[x], [y], [z], [ix], [jy], [kz], [w]])
        # This funciton returns a tuple (right_position, left_position)

        r_pos = self.right_arm.endpoint_pose()
        l_pos = self.left_arm.endpoint_pose()

        r_X_current = np.matrix([[r_pos['position'].x],
                                 [r_pos['position'].y],
                                 [r_pos['position'].z],
                                 [r_pos['orientation'].x],
                                 [r_pos['orientation'].y],
                                 [r_pos['orientation'].z],
                                 [r_pos['orientation'].w]])

        l_X_current = np.matrix([[l_pos['position'].x],
                                 [l_pos['position'].y],
                                 [l_pos['position'].z],
                                 [l_pos['orientation'].x],
                                 [l_pos['orientation'].y],
                                 [l_pos['orientation'].z],
                                 [l_pos['orientation'].w]])

        #DEBUG
        #print("r_x_current type")
        #if np.floor(rospy.Time.now().to_sec())%3 == 0:
            #print((r_X_current))

        return r_X_current, l_X_current

    def get_Xdot_current(self):
        # Returns current cartesian linear velocity and angular velocity as a
        # numpy column vector.
        # Note: This is unavailible in simulation.
        r_vel = self.right_arm.endpoint_velocity()
        l_vel = self.left_arm.endpoint_velocity()

        # The Baxter interface function enpoint_velocity returns a dict of named
        # tuples as follows: {'linear': (x, y, z), 'angular': (x, y, z)}

        r_Xdot_current = np.matrix([[r_vel['linear'].x],
                                    [r_vel['linear'].y],
                                    [r_vel['linear'].z],
                                    [r_vel['angular'].x],
                                    [r_vel['angular'].y],
                                    [r_vel['angular'].z]])

        l_Xdot_current = np.matrix([[l_vel['linear'].x],
                                    [l_vel['linear'].y],
                                    [l_vel['linear'].z],
                                    [l_vel['angular'].x],
                                    [l_vel['angular'].y],
                                    [l_vel['angular'].z]])


        return r_Xdot_current, l_Xdot_current

    def get_q_current(self):
        # Returns the current joint angles as a numpy column vector, and also
        # returns a dict for KDL purposes

        r_q = self.right_arm.joint_angles() #dict
        l_q = self.left_arm.joint_angles() #dict

        r_q_current = np.matrix([[r_q['right_s0']],
                                 [r_q['right_s1']],
                                 [r_q['right_e0']],
                                 [r_q['right_e1']],
                                 [r_q['right_w0']],
                                 [r_q['right_w1']],
                                 [r_q['right_w2']]])

        l_q_current = np.matrix([[l_q['left_s0']],
                                 [l_q['left_s1']],
                                 [l_q['left_e0']],
                                 [l_q['left_e1']],
                                 [l_q['left_w0']],
                                 [l_q['left_w1']],
                                 [l_q['left_w2']]])

        return r_q_current, l_q_current, r_q, l_q

    def get_qdot_current(self):
        # Returns joint angular velocities
        # This is availible in simulation.
        r_qdot = self.right_arm.joint_velocities()
        l_qdot = self.left_arm.joint_velocities()

        r_qdot_current = np.matrix([[r_qdot['right_s0']],
                                    [r_qdot['right_s1']],
                                    [r_qdot['right_e0']],
                                    [r_qdot['right_e1']],
                                    [r_qdot['right_w0']],
                                    [r_qdot['right_w1']],
                                    [r_qdot['right_w2']]])

        l_qdot_current = np.matrix([[l_qdot['left_s0']],
                                    [l_qdot['left_s1']],
                                    [l_qdot['left_e0']],
                                    [l_qdot['left_e1']],
                                    [l_qdot['left_w0']],
                                    [l_qdot['left_w1']],
                                    [l_qdot['left_w2']]])

        return r_qdot_current, l_qdot_current

    def clean_shutdown(self):
        rospy.loginfo("Releasing Cloth...")
        self.left_grip.open(block=False)
        self.right_grip.open(block=False)

        rospy.loginfo("Resetting Control Mode...")
        self.left_arm.exit_control_mode()
        self.right_arm.exit_control_mode()

        rospy.loginfo("Restoring Default Publish Frequency...")
        self.joint_state_pub.publish(100) # 100Hz default joint state rate

        rospy.loginfo("Moving to Neutral...")
        self.right_arm.move_to_neutral()
        self.left_arm.move_to_neutral()

        # Here, we do not disable the robot at the end.
        return True

    def speed_data_process(self):

        for speed_origin_tmp in self.speed_joint_origin:
            # speed_diff_tmp = []
            speed_per_tmp = []
            for i in range(7):
                # rospy.loginfo(len(self.speed_joint_max.tolist()))
                # rospy.loginfo("i = {}".format(i) )
                speed_abs = np.abs(speed_origin_tmp[i])
                # speed_diff_tmp.append(self.speed_joint_max[i] - speed_abs)
                speed_per_tmp.append(speed_abs / (self.speed_joint_max.tolist()[0])[i] )
            # self.speed_joint_difference.append(speed_diff_tmp)
            self.speed_joint_percent.append(speed_per_tmp)

        for i in range(len(self.speed_endfactor_real)):
            speed_end_max_tmp = self.speed_endfactor_max[i]
            speed_end_real_tmp = self.speed_endfactor_real[i]
            speed_end_per_tmp = []
            for j in range(6):
                if abs(speed_end_max_tmp[j]) > 0:
                    tmp = abs(speed_end_real_tmp[j]) / abs(speed_end_max_tmp[j])
                else:
                    tmp = 0
                speed_end_per_tmp.append(tmp)
            self.speed_endfactor_percent.append(speed_end_per_tmp)

        return

    def speed_data_print(self):
        for speed_ori_tmp in self.speed_joint_origin:
            self.speed_origin_file.write("{0[0]:>8.4f} {0[1]:>8.4f} {0[2]:>8.4f} {0[3]:>8.4f} {0[4]:>8.4f} {0[5]:>8.4f} {0[6]:>8.4f}\n"
                                .format(speed_ori_tmp))

        # for speed_diff_tmp in self.speed_joint_difference:
        #     self.speed_log_file.write("{0[0]:>8.4f} {0[1]:>8.4f} {0[2]:>8.4f} {0[3]:>8.4f} {0[4]:>8.4f} {0[5]:>8.4f} {0[6]:>8.4f}\n"
        #                         .format(speed_diff_tmp))

        for speed_per_tmp in self.speed_joint_percent:
            self.speed_percent_file.write("{0[0]:>8.2%} {0[1]:>8.2%} {0[2]:>8.2%} {0[3]:>8.2%} {0[4]:>8.2%} {0[5]:>8.2%} {0[6]:>8.2%}\n"
                                .format(speed_per_tmp))

        for speed_end_max_tmp in self.speed_endfactor_max:
            self.speed_endfactor_max_file.write("{0[0]:>8.4f} {0[1]:>8.4f} {0[2]:>8.4f} {0[3]:>8.4f} {0[4]:>8.4f} {0[5]:>8.4f}\n"
                                .format(speed_end_max_tmp))

        for speed_end_real_tmp in self.speed_endfactor_real:
            self.speed_endfactor_real_file.write("{0[0]:>8.4f} {0[1]:>8.4f} {0[2]:>8.4f} {0[3]:>8.4f} {0[4]:>8.4f} {0[5]:>8.4f}\n"
                                .format(speed_end_real_tmp))

        for speed_end_per_tmp in self.speed_endfactor_percent:
            self.speed_endfactor_per_file.write("{0[0]:>8.2%} {0[1]:>8.2%} {0[2]:>8.2%} {0[3]:>8.2%} {0[4]:>8.2%} {0[5]:>8.2%}\n"
                                .format(speed_end_per_tmp))

        for time in self.speed_time:
            self.speed_time_file.write("{:>6.3f}\n".format(time))

        return

def main():

    #logging.basicConfig(level=logging.DEBUG, format='[%(levelname)s]: %(message)s')
    #logger = logging.getLogger(__name__)
    #logger.debug("task")

    print( "Initializing Controller Node... " )
    rospy.init_node("table_cloth")

    controller = TableCloth( )

    rospy.on_shutdown(controller.clean_shutdown)

    controller.move()

    print("Done")

# Helper Function for generating a Skew-Symmetric Numpy Matrix
def skew_symm(vector):
    # Assume a 3x1 input numpy matrix
    return np.matrix([[0.0, -1.0*vector[2,0], vector[1,0]],
                      [vector[2,0], 0.0, -1.0*vector[0,0]],
                      [-1.0*vector[1,0], vector[0,0], 0.0]])

def gen_inv(A, Winv):
    # Returns the generalized inverse with W as the inverse of the weighing
    # matrix W.
    # Formula: A^w+ = Winv*A'(A*Winv*A')^-1
    return Winv.dot(A.T).dot( inv(A.dot(Winv).dot(A.T)) )

if __name__ == '__main__':
    main()
