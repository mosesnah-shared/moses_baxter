#!/usr/bin/python3



# Import python modules
import math
import random
import rospy
import argparse
import os
import PyKDL
from std_msgs.msg import (
    UInt16, Empty
)
import baxter_interface
from baxter_pykdl import baxter_kinematics
import numpy as np
from numpy import matrix
from numpy import linalg
from tf.transformations import euler_from_quaternion
from baxter_interface import CHECK_VERSION


class ImpedanceController(object):

    def __init__(self):

        self._desired_time = 30

        # set joint state publishing to 1000Hz
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate', UInt16, queue_size=10)
        self._rate = 1000.0  # Hz
        self._pub_rate.publish(self._rate)
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout

        # create disable cuff publisher
        cuff_ns = 'robot/limb/left/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)        	

        # initialize the limb instance
        self._left_arm = baxter_interface.limb.Limb("left")
        self._left_joint_names = self._left_arm.joint_names()
        self._kin=baxter_kinematics('left')
    
        # enable robot
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

        # initial config
        self._x_ini = 0.6500
        self._y_ini = 0.8372
        self._z_ini = 0.0571	
        

    def move_to_equilibrium(self):
        # move arms to equilibrium pose
        print("Moving to equilibrium pose...")
        equilibrium={'left_s0': -0.640305260394, 
                     'left_s1': -1.13664905996, 
                     'left_e0': -0.0592355456897, 
                     'left_e1': 2.30881311633, 
                     'left_w0': -3.07412390181, 
                     'left_w1': -0.40632903375, 
                     'left_w2': -3.11397363341}
        
        self._left_arm.move_to_joint_positions( equilibrium )

    def move_to_neutral(self):
        """
        Moves the limb to neutral location.
        """
        self._left_arm.move_to_neutral()
        
    def move(self):
        
        # set control rate
        control_rate = rospy.Rate(self._rate)

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and disable
        self._left_arm.set_command_timeout((1.0 / self._rate) * self._missed_cmds)
        start = rospy.Time.now()
        elapsed = (rospy.Time.now() - start).to_sec()

            # loop at specified rate commanding new joint torques
        while elapsed < self._desired_time:
            elapsed = (rospy.Time.now() - start).to_sec() 
            # self._pub_cuff_disable.publish()
            
            # self._pub_rate.publish(self._rate)
            left_torques = self.get_torque_cmd()
            
            self._left_arm.set_joint_torques(dict(zip(self._left_joint_names,left_torques)))
            control_rate.sleep()

                    
    def get_torque_cmd(self):

        ## get robot data ##
        # get current end-effector position and orientation
        l_pos = self._left_arm.endpoint_pose()
        l_Xpos_cur = np.matrix([[l_pos['position'].x], 
                                     [l_pos['position'].y], 
                                     [l_pos['position'].z]])
        l_Xquat_cur = np.matrix([[l_pos['orientation'].x], 
                                      [l_pos['orientation'].y], 
                                      [l_pos['orientation'].z],
                                      [l_pos['orientation'].w]])
        l_u_cur = np.matrix([[l_pos['orientation'].x], 
                                      [l_pos['orientation'].y], 
                                      [l_pos['orientation'].z]])
        l_theta_cur = l_pos['orientation'].z

        # get current end-effector velocity
        l_vel = self._left_arm.endpoint_velocity()
        l_Xdot_cur = np.matrix([[l_vel['linear'].x],
                                        [l_vel['linear'].y],
                                        [l_vel['linear'].z],
                                         [l_vel['angular'].x],
                                        [l_vel['angular'].y],
                                        [l_vel['angular'].z]])
        l_Xdot_cur = np.matrix([[l_vel['linear'].x],
                                        [l_vel['linear'].y],
                                        [l_vel['linear'].z]])
        l_W_cur = np.matrix([[l_vel['angular'].x],
                                     [l_vel['angular'].y],
                                     [l_vel['angular'].z]])

        # get current joint position
        l_q_curDict = self._left_arm.joint_angles()
        l_q_cur = np.matrix([[l_q_curDict['left_s0']],
                                     [l_q_curDict['left_s1']],
                                     [l_q_curDict['left_e0']],
                                     [l_q_curDict['left_e1']],
                                     [l_q_curDict['left_w0']],
                                     [l_q_curDict['left_w1']],
                                     [l_q_curDict['left_w2']]])

        # get current joint position
        l_qdot_curDict = self._left_arm.joint_velocities()
        l_qdot_cur = np.matrix([[l_qdot_curDict['left_s0']],
                                        [l_qdot_curDict['left_s1']],
                                        [l_qdot_curDict['left_e0']],
                                        [l_qdot_curDict['left_e1']],
                                        [l_qdot_curDict['left_w0']],
                                        [l_qdot_curDict['left_w1']],
                                        [l_qdot_curDict['left_w2']]])

        # get Jacobian
        l_Jacobian = self._kin.jacobian()
        l_Jacobian_trans = self._kin.jacobian_transpose() 

        ## set desired parameter ##
        # set desired Cart. position
        #l_Xpos_des = l_Xpos_cur
        l_Xpos_des = np.matrix([[self._x_ini],
                                     [self._y_ini],
                                     [self._z_ini]])

        # set desired Cart. velocity
        l_Xdot_des = np.matrix([[0],
                          [0],
                          [0]])		

        # set stiffness for end effector
        #Kx=np.matrix([[200, 0, 0],
        #	      [0, 200, 0],
        #	      [0, 0, 500]])
        Kx = 320.0	

        # set damping for end effector
        #Bx=np.matrix([[10, 0, 0],
        #	      [0, 10, 0],
        #	      [0, 0, 10]])
        Bx = 10.0	
        
        # set stiffness for joints		
        Kq=np.matrix([[1, 0, 0, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0, 0, 0],
                  [0, 0, 1, 0, 0, 0, 0],
                  [0, 0, 0, 1, 0, 0, 0],
                  [0, 0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 0, 1]])
    
        
        # get external force on end effector

        l_f_elastic = np.matrix([[0.0],
                                      [0.0],
                                      [0.0],
                                      [0.0],
                                      [0.0],
                                      [0.0]])
        l_f_elastic[0:3,0] = Kx * (l_Xpos_des[0:3,0] - l_Xpos_cur[0:3,0]) + Bx * (l_Xdot_des[0:3,0] - l_Xdot_cur[0:3,0])
        #print("\nl_Xpos_des:")
        #print(l_Xpos_des)
        #print("\nl_Xpos_cur:")
        #print(l_Xpos_cur)
        #print("\nl_Xdot_des:")
        #print(l_Xdot_des)
        #print("\nl_Xdot_cur:")
        #print(l_Xdot_cur)
        #print("\nl_f_elastic:")
        #print(l_f_elastic)

        # transform external force in cartesian space to joint torques	
        #torqueTask=np.dot(self._kin.jacobian_transpose(),l_f_elastic)
        torqueTask = np.dot(l_Jacobian_trans, l_f_elastic)
        print("\ntorqueTask:")
        print(torqueTask)
        
        # torque generated from joint stiffness
        #torqueDamp=Kq*(self.get_q_error())
        torqueDamp = Kq*l_qdot_cur
        print("\ntorqueDamp:")
        print(torqueDamp)	

        # total commanded torque
        # commandedTorque = torqueTask
        commandedTorque = torqueTask - torqueDamp
        
        # np.array to array
        torque_cmd=[]
        for i in commandedTorque:
            torque_cmd.append(i)
            		
        return torque_cmd	
    

    def _reset_control_modes(self):
        # reset to normal control conditions
        rate = rospy.Rate(self._rate)
        
        for _ in range(100):
            if rospy.is_shutdown():
                return False
            self._left_arm.exit_control_mode()
            self._pub_rate.publish(100)  # 100Hz default joint state rate
            rate.sleep()
        return True

    def clean_shutdown(self):
        # reset to normal control conditions and shutdown
        print("\nExiting impedance controller")
        self._reset_control_modes()
        self._left_arm.move_to_neutral()
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
                
        return True
                    
def main():
    
    print("Initializing node... ")
    rospy.init_node("impedanceController")
    controller = ImpedanceController()
    rospy.on_shutdown(controller.clean_shutdown)
    controller.move_to_neutral()
    #controller.move_to_equilibrium()	
    controller.move()

if __name__ == '__main__':
    main()
