#!/usr/bin/python3

# Import python modules
import math
import random
import rospy
import argparse
import os
import PyKDL
import tf
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

        self._desired_time = 240

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
        print("Running... Ctrl-c to quit")	
        

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
        self._left_arm.move_to_joint_positions(equilibrium)

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

        self.move_to_neutral()
        l_pos_des_0 = self._left_arm.endpoint_pose()
        print("\nDesired position:")
        print(l_pos_des_0)
        
            # loop at specified rate commanding new joint torques
        while elapsed < self._desired_time:
            elapsed = (rospy.Time.now() - start).to_sec() 
            # self._pub_cuff_disable.publish()
            # self._pub_rate.publish(self._rate)
            left_torques = self.get_torque_cmd(l_pos_des_0)
            self._left_arm.set_joint_torques(dict(zip(self._left_joint_names,left_torques)))
            control_rate.sleep()

                
    def get_torque_cmd(self,l_pos_des_0):

        ## *********************** get robot data *********************** ##
        # get current end-effector position and orientation
        l_pos = self._left_arm.endpoint_pose()
        l_Xpos_cur_0 = np.matrix([[l_pos['position'].x], 
                                     [l_pos['position'].y], 
                                     [l_pos['position'].z]])
        l_Xquat_cur_0 = np.matrix([[l_pos['orientation'].x], 
                                      [l_pos['orientation'].y], 
                                      [l_pos['orientation'].z],
                                      [l_pos['orientation'].w]])
    
        # rotation matrix
        l_H_tcp_0 = tf.transformations.quaternion_matrix([l_pos['orientation'].x, 
                                                   l_pos['orientation'].y, 
                                                   l_pos['orientation'].z,
                                                   l_pos['orientation'].w])
        l_R_tcp_0 = l_H_tcp_0[0:3,0:3]		
        
        # axis angle	
        l_theta_des = 2*np.arccos(l_pos_des_0['orientation'].w)
        l_u_des_0 = np.matrix([[1/(np.sin(l_theta_des/2)) * l_pos_des_0['orientation'].x], 
                                       [1/(np.sin(l_theta_des/2)) * l_pos_des_0['orientation'].y], 
                                       [1/(np.sin(l_theta_des/2)) * l_pos_des_0['orientation'].z]])


        # get current end-effector velocity
        l_vel = self._left_arm.endpoint_velocity()
        l_twist_cur = np.matrix([[l_vel['linear'].x],
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
        l_Jacobian_transp = self._kin.jacobian_transpose() 

        # get Mass matrix
        l_MassMatrix = self._kin.inertia()


        ## *********************** set desired parameter *********************** ##
        # set desired Cart. position
        l_Xpos_des_0 = np.matrix([[l_pos_des_0['position'].x],
                          [l_pos_des_0['position'].y],
                          [l_pos_des_0['position'].z]])

        # set desired orientation
        quat_des_0 = np.matrix([[l_pos_des_0['orientation'].x],
                       [l_pos_des_0['orientation'].y],
                       [l_pos_des_0['orientation'].z],
                       [l_pos_des_0['orientation'].w]])

        # set linear stiffness for end effector
        Kl = np.matrix([[500, 0, 0],
                      [0, 500, 0],
                      [0, 0, 500]])	

        # set angular stiffness for end effector
        Kr = np.matrix([[500, 0, 0],
                    [0, 500, 0],
                    [0, 0, 500]])

        # set stiffness for end effector
        zer = np.zeros([3,3])
        K_up = np.concatenate([Kl,zer],axis=1)
        K_down = np.concatenate([zer,Kr],axis=1)
        K = np.concatenate([K_up,K_down],axis=0)

        # set desired damped behavior (0<=gamma<=1; gamma=0 -> undamped, gamma=1 -> real eigenvalues)
        gamma = 0.17
        D_gamma = np.matrix([[gamma, 0, 0, 0, 0, 0],
                         [0, gamma, 0, 0, 0, 0],
                         [0, 0, gamma, 0, 0, 0],
                         [0, 0, 0, gamma, 0, 0],
                         [0, 0, 0, 0, gamma, 0],
                         [0, 0, 0, 0, 0, gamma]])	
        
        # set stiffness for joints		
        Kq=np.matrix([[0.005, 0, 0, 0, 0, 0, 0],
                  [0, 0.005, 0, 0, 0, 0, 0],
                  [0, 0, 0.005, 0, 0, 0, 0],
                  [0, 0, 0, 0.005, 0, 0, 0],
                  [0, 0, 0, 0, 0.005, 0, 0],
                  [0, 0, 0, 0, 0, 0.005, 0],
                  [0, 0, 0, 0, 0, 0, 0.005]])
        
        
    
        
        ## *********************** calculate elastic wrench on end effector ***********************##

        l_w_elastic = np.matrix([[0.0],
                                         [0.0],
                                         [0.0],
                                         [0.0],
                                         [0.0],
                                         [0.0]])
        l_w_tcp_linK = np.matrix([[0.0],
                                          [0.0],
                                          [0.0],
                                          [0.0],
                                          [0.0],
                                          [0.0]])
        l_w_0_linK = np.matrix([[0.0],
                                        [0.0],
                                        [0.0],
                                        [0.0],
                                        [0.0],
                                        [0.0]])
        l_w_tcp_rotK = np.matrix([[0.0],
                                          [0.0],
                                          [0.0],
                                          [0.0],
                                          [0.0],
                                          [0.0]])
        l_w_0_rotK = np.matrix([[0.0],
                                        [0.0],
                                        [0.0],
                                        [0.0],
                                        [0.0],
                                        [0.0]])

        # get wrench for translational stiffness, based on quaternion potential
        l_w_tcp_linK[0:3,0] = -Kl * np.transpose(l_R_tcp_0) * (l_Xpos_cur_0[0:3,0] - l_Xpos_des_0[0:3,0])
        l_Xpos_cur_des_tcp = np.transpose(l_R_tcp_0) * (l_Xpos_cur_0[0:3,0] - l_Xpos_des_0[0:3,0])
        l_Xpos_cur_des_tcp0 = l_Xpos_cur_des_tcp[0,0]
        l_Xpos_cur_des_tcp1 = l_Xpos_cur_des_tcp[1,0]
        l_Xpos_cur_des_tcp2 = l_Xpos_cur_des_tcp[2,0]
        l_Xdes_tcp_tilde = np.matrix([[0, -l_Xpos_cur_des_tcp2, l_Xpos_cur_des_tcp1],
                                [l_Xpos_cur_des_tcp2, 0, -l_Xpos_cur_des_tcp0],
                                [-l_Xpos_cur_des_tcp1, l_Xpos_cur_des_tcp0, 0]])
        l_w_tcp_linK[3:6,0] = l_Xdes_tcp_tilde * Kl * np.transpose(l_R_tcp_0) * (l_Xpos_cur_0[0:3,0] - l_Xpos_des_0[0:3,0])
        l_w_0_linK[0:3,0] = l_R_tcp_0 * l_w_tcp_linK[0:3,0]
        l_w_0_linK[3:6,0] = l_R_tcp_0 * l_w_tcp_linK[3:6,0]

        # get wrench for rotational stiffness, based on quaternion potential
        l_H_des_0 = tf.transformations.quaternion_matrix([quat_des_0[0,0], 
                                                    quat_des_0[1,0], 
                                                    quat_des_0[2,0],
                                                    quat_des_0[3,0]])
        l_R_des_0 = l_H_des_0[0:3,0:3]
        l_R_des_tcp = np.transpose(l_R_tcp_0) * l_R_des_0
        l_H_des_tcp = np.identity(4)
        l_H_des_tcp[0:3,0:3] = l_R_des_tcp[0:3,0:3]
        quat_des_tcp = tf.transformations.quaternion_from_matrix(l_H_des_tcp)
        I33 = np.identity(3)
        eps_des_tcp = np.matrix([[quat_des_tcp[0]],
                                        [quat_des_tcp[1]],
                                        [quat_des_tcp[2]]])
        eps_des0 = eps_des_tcp[0,0]
        eps_des1 = eps_des_tcp[1,0]
        eps_des2 = eps_des_tcp[2,0]
        epsDesTilde_tcp = np.matrix([[0,-eps_des2,eps_des1],
                         [eps_des2, 0, -eps_des0],
                         [-eps_des1, eps_des0, 0]])
        eta_des = quat_des_tcp[3]
        E = eta_des * I33 - epsDesTilde_tcp
        l_w_tcp_rotK[3:6,0] = 2 * np.transpose(E) * Kr * eps_des_tcp[0:3,0]
        l_w_0_rotK[3:6,0] = l_R_tcp_0 * l_w_tcp_rotK[3:6,0]

        # add wrenches
        l_w_elastic = l_w_0_linK + l_w_0_rotK


        ## *********************** damping design (factorization design) ***********************##		
        Mq_inv = np.linalg.inv(l_MassMatrix)
        Mx_inv_0 = l_Jacobian * Mq_inv * l_Jacobian_transp
        Ad_H_up = np.concatenate([l_R_tcp_0,zer],axis=1)
        Ad_H_down = np.concatenate([zer,l_R_tcp_0],axis=1)
        Ad_H = np.concatenate([Ad_H_up,Ad_H_down],axis=0)
        Ad_H_trans = np.transpose(Ad_H)
        Mx_inv_tcp = Ad_H * Mx_inv_0 * Ad_H_trans	
        # handle singular directions
        u, s, v = np.linalg.svd(Mx_inv_tcp, full_matrices=True)
        s_diag = np.diag(s)
        minSingValue = 0.001
        if s_diag.min() < minSingValue:
            i=0
            for row in range(0,6,1):
                for cell in range(i,i+1):
                    if s_diag[row][cell] < minSingValue:
                        print('\nSingular value in Mx:' ,s_diag[row][cell])
                        s_diag[row][cell] = minSingValue
                i=i+1
        s_inv = np.linalg.inv(s_diag)
        s_inv_sqrt = np.sqrt(s_inv)
        Mx_tcp_sqrt = u * s_inv_sqrt * np.transpose(u)
        # factorization design
        A = Mx_tcp_sqrt
        Kd1 = np.sqrt(K)
        Dl_tcp = A * D_gamma * Kd1 + Kd1 * D_gamma * A
        Dl_0 = Ad_H_trans * Dl_tcp * Ad_H

        tauDampFac = l_Jacobian_transp * Dl_0 * l_twist_cur
        #print("\ntauDampFac:")
        #print(tauDampFac)
        

        ## *********************** calculate torque and command it ***********************##	
        torqueTask = np.dot(l_Jacobian_transp, l_w_elastic)
        #print("\ntorqueTask:")
        #print(torqueTask)
        
        # torque generated from joint stiffness
        torqueDamp = Kq * l_qdot_cur	

        # total commanded torque
        commandedTorque = torqueTask - torqueDamp
        # commandedTorque = torqueTask - tauDampFac - torqueDamp 
        
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
