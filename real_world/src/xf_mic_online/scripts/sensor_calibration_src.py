#!/usr/bin/env python3
#encoding:utf-8

import rospy
import numpy as np
from ob_calibration import *
import queue as Queue
from geometry_msgs.msg import  Pose2D
from numpy import sin,cos
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
def h_merge(matrix_block):
    #Horizontal merging multi-matrices
    result = matrix_block[0]
    for matrix in matrix_block[1:]:
        result = np.concatenate((result, matrix), axis=1)
    return result
def v_merge(matrix_block):
    # Vertical merging matrix
    result = matrix_block[0]
    for matrix in matrix_block[1:]:
        result = np.vstack((result, matrix))
    return result
def get_sensor_pos_ori(path,yaw_angles,relative_pos,relative_ori):
    '''
    :param path: robot position
    :param yaw_angles:  robot yaw angles
    :param relative_pos: robot to mic
    :param relative_ori: robot to mic
    :return: Sensor pose in global frame
    '''
    sensor_pos_set = np.zeros((0,2))
    sensor_ori_set = np.zeros((0,1))
    for i,point in enumerate(path):
        sensor_pos = point+rot_mat(yaw_angles[i])@relative_pos
        sensor_pos_set = np.vstack((sensor_pos_set,sensor_pos))
        sensor_ori = yaw_angles[i]+relative_ori
        sensor_ori_set = np.vstack((sensor_ori_set,sensor_ori))
    return sensor_pos_set,sensor_ori_set
def mic_mea(mic_pos_set,mic_ori_set,src,path,yaw_angles):
    # DoA measurements(Sound source direction of arrival w.r.t mic. array)
    # This could be measured by SRP-PHAT algorithm
    doa = np.zeros((0,2))
    for i in range(len(mic_pos_set)):
        gt = rot_mat(mic_ori_set[i]).T@(src-mic_pos_set[i])/norm(src-mic_pos_set[i])
        doa = np.vstack((doa,gt))
    return doa
def lidar_mea(lidar_pos_set,lidar_ori_set,src,path,yaw_angles):
    # Lidar relative orientation and translation
    # This could be measured by ICP algorithm
    delta_theta = np.zeros((0,1))
    delta_pos   = np.zeros((0,2))
    for i in range(len(lidar_pos_set)-1):
        delta_theta = np.vstack((delta_theta,lidar_ori_set[i+1,0]-lidar_ori_set[i,0])) # yaw_angles[i+1]-yaw_angles[i]
        delta_pos_gt= rot_mat(lidar_ori_set[i,0]).T@(lidar_pos_set[i+1]-lidar_pos_set[i])
        delta_pos = np.vstack((delta_pos,delta_pos_gt))
    return  delta_theta,delta_pos
# Jacobian matrix of measurement model
def h_jacobian(p_sensor,theta_sensor,x_i,theta_i,x_iplus1,theta_iplus1,type,SRC=None,doa_pre = None,symbol = None,b= None,l= None):
    R_i = rot_mat(theta_i)
    R_iplus1 = rot_mat(theta_iplus1)
    if type=="mic":
        delta_M_i = SRC - (x_i + R_i @ p_sensor)
        delta_M_iplus1 = SRC - (x_iplus1 + R_iplus1 @ p_sensor)
        T_i = np.zeros((2, 3))
        T_i[:2, :2] =(R_iplus1 @ rot_mat(theta_sensor)).T / norm(delta_M_iplus1) ** 3 @ (-R_iplus1 * norm(delta_M_iplus1) ** 2 + np.array(
            [[delta_M_iplus1[0, 0] ** 2, delta_M_iplus1[0, 0] * delta_M_iplus1[1, 0]],
             [delta_M_iplus1[0, 0] * delta_M_iplus1[1, 0], delta_M_iplus1[1, 0] ** 2]]) @ R_iplus1)
        T_i[:2, 2:3] = rot_mat(theta_sensor, True).T @ R_iplus1.T @ (delta_M_iplus1) / norm(delta_M_iplus1)

    elif type=="lidar":
        T_i = np.zeros((3, 3))
        T_i[:2, :2] = (R_i @ rot_mat(theta_sensor)).T @ (R_iplus1 - R_i)
        T_i[:2, 2:3] = rot_mat(theta_sensor, True).T @ R_i.T @ (R_iplus1 @ p_sensor + x_iplus1 - R_i @ p_sensor - x_i)
    return T_i
def quaternion_to_angle(orientation):
    w, x, y, z = orientation.w,orientation.x,orientation.y,orientation.z
    theta = 2 * np.arctan2(z, w)
    return theta

# EKF for estimation sensors parameters
class e_kalman_filter(object):
    def __init__(self, Q, x0, P0,sensor):
        self.Q = Q
        self.x_est = x0
        self.P_est = P0
        self.type = sensor
        self.SRC = np.array([1.2,2.4]).reshape(-1, 1)  

    def update(self,robot_pose_last,robot_pose_cur,data):
        self.x_pred = self.x_est
        self.P_pred = self.P_est
        self.p_sensor = self.x_pred[0:2].reshape(-1, 1)
        self.theta_sensor = self.x_pred[2, 0]

        self.x_i = robot_pose_last[:2].reshape(-1, 1)
        self.x_iplus1 = robot_pose_cur[:2].reshape(-1, 1)
        self.theta_i = robot_pose_last[2]
        self.theta_iplus1 = robot_pose_cur[2]

        mea_model_diff = np.zeros((0, 1))
        if self.type == "lidar":
            delta_pos = (rot_mat(self.theta_i) @ rot_mat(self.theta_sensor)).T@ \
                        (rot_mat(self.theta_iplus1)@self.p_sensor+self.x_iplus1-rot_mat(self.theta_i)@self.p_sensor-self.x_i)
            mea_model_diff = np.vstack((mea_model_diff, 
                                        (rot_mat(data[5]).T@((data[3:5]-data[:2]).reshape(-1, 1)) - delta_pos)
                                        ))
            mea_model_diff = np.vstack((mea_model_diff, (data[5]-data[2]) - (robot_pose_cur[-1]-robot_pose_last[-1]).reshape(-1, 1)))
            self.H = h_jacobian(self.p_sensor,self.theta_sensor,self.x_i,self.theta_i,self.x_iplus1,self.theta_iplus1,self.type)
        elif self.type == "mic":
            #self.SRC = self.x_pred[3:5].reshape(-1, 1)
            doa = (rot_mat(self.theta_i) @ rot_mat(self.theta_sensor)).T @ (self.SRC - (self.x_i + rot_mat(self.theta_i) @ self.p_sensor)) / norm(
                self.SRC - (self.x_i + rot_mat(self.theta_i) @ self.p_sensor))
            doa_next = (rot_mat(self.theta_iplus1) @ rot_mat(self.theta_sensor)).T @ (self.SRC - (self.x_iplus1 + rot_mat(self.theta_iplus1) @ self.p_sensor)) / norm(
                self.SRC - (self.x_iplus1 + rot_mat(self.theta_iplus1) @ self.p_sensor))

            mea_model_diff = np.vstack((mea_model_diff, (np.array([np.cos(data[1]),np.sin(data[1])]).reshape(-1, 1) - doa_next)))
            self.H = h_jacobian(self.p_sensor,self.theta_sensor,self.x_i,self.theta_i,self.x_iplus1,self.theta_iplus1,self.type,self.SRC,[doa,doa_next,data[0],data[1]])
        
        # Kalman gain
        self.K = self.P_pred @ self.H.T @ np.linalg.inv(self.H @ self.P_pred @ self.H.T + self.Q)
        # Posterior probability distribution
        self.x_est = self.x_pred + self.K @ mea_model_diff
        if self.x_pred[2] > 2*np.pi:
            self.x_pred[2] = self.x_pred[2] - 2*np.pi
        elif self.x_pred[2] < 0:
            self.x_pred[2] = self.x_pred[2] + 2*np.pi
        self.P_est = (np.eye(len(self.x_est)) - self.K @ self.H) @ self.P_pred

class calibrate(object):
    def __init__(self):
        # used for save the bag data
        # self.all_robot_pose = np.zeros((0,2))
        # self.all_lidar_pose = np.zeros((0,2))
        # self.test = np.zeros((0,4))
        # self.test_for_lidar = np.zeros((0,6))

        self.lidar_call_count = 0
        self.mic_call_count = 0

        self.mic_call_count_100 = 0
        
        self.mic_mea =  np.zeros(2)                # [theta_last theta_cur]
        self.lidar_mea =  np.zeros(6)              # [pose_last  pose_cur]
        self.mic_update = False
        self.lidar_update = False
        self.robot_pose_for_mic = np.zeros(3)      # last record --> [x y theta]
        self.robot_pose_for_lidar = np.zeros(3)
        self.robot_pose_cur = np.zeros(3)
        
        # ground truth
        self.mic_pos_theta = np.array([0.12,0.001,270/180*np.pi])
        self.lidar_pos_theta = np.array([-0.04,0.001,0/180*np.pi])
        self.sound_source_pos = np.array([1.2,2.4])          

        # plot
        self.x_error_mic = np.zeros((0,1))
        self.x_error_lidar = np.zeros((0,1))
        self.y_error_mic = np.zeros((0,1))
        self.y_error_lidar = np.zeros((0,1))
        self.theta_error_mic = np.zeros((0,1))
        self.theta_error_lidar = np.zeros((0,1))
        # self.x_error_src = np.zeros((0,1))
        # self.y_error_src = np.zeros((0,1))

        self.Q_mic   = np.diag([0.055,0.088,])
        self.Q_lidar = np.diag([5e-6, 8e-7, 4e-5])
        self.x0_mic   = np.array([0.3,0.3,220/180*np.pi]).reshape(-1, 1)  # initial value
        self.x0_lidar = np.array([0.5,0.5,50/180*np.pi]).reshape(-1, 1)
        self.P0 = np.eye(3)
        self.P0_lidar = np.eye(3)
        self.mic_ekf   = e_kalman_filter(self.Q_mic, self.x0_mic, self.P0, "mic")
        self.lidar_ekf = e_kalman_filter(self.Q_lidar, self.x0_lidar, self.P0_lidar, "lidar")
        self.lidar_cali_pos_rec = self.x0_lidar[:2].reshape(-1)    # lidar record
        self.swich_lidar_pos = False
        self.doa_sub = rospy.Subscriber('/src_pos2', Header, self.mic_mea_update, tcp_nodelay=True, queue_size=5)
        self.pos_sub = rospy.Subscriber('/odom', Odometry, self.pose_update,tcp_nodelay=True, queue_size=5)
        self.lidar_sub = rospy.Subscriber('/pose2D', Pose2D, self.lidar_mea_update,tcp_nodelay=True, queue_size=5)
        self.run()
    
    def lidar_mea_update(self,msg):
        self.lidar_call_count +=1
        if self.lidar_call_count == 5:
            # self.all_lidar_pose = v_merge([self.all_lidar_pose, np.array([msg.x,msg.y]).reshape((-1,2))])
            self.lidar_mea[:2] = self.lidar_mea[3:5]
            self.lidar_mea[2]  = self.lidar_mea[5]
            self.lidar_mea[3:6]= np.array([msg.x,msg.y,msg.theta]) 
            self.lidar_update = True
            self.lidar_call_count =0

    def mic_mea_update(self,msg):
        self.mic_call_count +=1
        # self.mic_call_count_100 +=1
        if self.mic_call_count== 1:
            self.mic_mea[0] = self.mic_mea[1]                   # last doa measurement
            self.mic_mea[1] = float(msg.frame_id)/180.0*np.pi            # newest doa
            # if self.mic_call_count_100 > 100:
            self.mic_update = True
            self.mic_call_count = 0

    def pose_update(self,msg):
        position = msg.pose.pose.position
        #print("odom",position)
        orientation = msg.pose.pose.orientation
        theta = quaternion_to_angle(orientation)
        if theta<0:
            theta = theta+2*np.pi
        elif theta>2*np.pi:
            theta = theta-2*np.pi
        self.robot_pose_cur = np.array([position.x,position.y,theta])
        # self.all_robot_pose = v_merge([self.all_robot_pose, self.robot_pose_cur[:2].reshape((-1,2))])

    def run(self):
        while not rospy.is_shutdown():
            if self.mic_update:
                # self.test = v_merge([self.test, np.append(self.robot_pose_cur,self.mic_mea[1])])
                if self.swich_lidar_pos:
                    rob_pose_lidar_frame = self.lidar_mea[3:6].reshape(-1,1)-self.lidar_ekf.x_est     # vector
                    # print(self.lidar_ekf.x_est[2,0])
                    # print(rob_pose_lidar_frame[:2])
                    # print(self.lidar_ekf.x_est[:2])
                    rob_pos_base_frame  = (rot_mat(self.lidar_ekf.x_est[2,0]) @ rob_pose_lidar_frame[:2]+self.lidar_ekf.x_est[:2]).reshape(-1)
                    rob_ori_base_frame  = self.lidar_ekf.x_est[2,0] + rob_pose_lidar_frame[2,0]
                    rob_pose_base_frame = np.append(rob_pos_base_frame,rob_ori_base_frame)
                    self.mic_ekf.update(self.robot_pose_for_mic,rob_pose_base_frame,self.mic_mea)
                    self.robot_pose_for_mic = rob_pose_base_frame
                else:
                    self.mic_ekf.update(self.robot_pose_for_mic,self.robot_pose_cur,self.mic_mea)
                    self.robot_pose_for_mic = self.robot_pose_cur
                self.mic_update = False

                error_mic = (self.mic_ekf.x_est)[:3].reshape(-1)-self.mic_pos_theta
                # error_src = (self.mic_ekf.x_est)[3:5].reshape(-1)-self.sound_source_pos
                self.x_error_mic = np.vstack((self.x_error_mic,error_mic[0]))
                self.y_error_mic = np.vstack((self.y_error_mic,error_mic[1]))
                self.theta_error_mic = np.vstack((self.theta_error_mic, error_mic[2]/np.pi*180))
                print(len(self.x_error_mic))
                # self.x_error_src = np.vstack((self.x_error_src,error_src[0]))
                # self.y_error_src = np.vstack((self.y_error_src,error_src[1]))

            if self.lidar_update:
            	# self.test_for_lidar =  v_merge([self.test_for_lidar, np.append(self.robot_pose_cur,self.lidar_mea[3:6])])
                self.lidar_ekf.update(self.robot_pose_for_lidar,self.robot_pose_cur,self.lidar_mea)
                self.robot_pose_for_lidar = self.robot_pose_cur
                self.lidar_update = False
                est_result  = self.lidar_ekf.x_est.reshape(-1)
                error_lidar = est_result  -self.lidar_pos_theta
                self.x_error_lidar = np.vstack((self.x_error_lidar,error_lidar[0]))
                self.y_error_lidar = np.vstack((self.y_error_lidar,error_lidar[1]))
                self.theta_error_lidar = np.vstack((self.theta_error_lidar, error_lidar[2]/np.pi*180))

                self.lidar_cali_pos_rec = np.vstack((self.lidar_cali_pos_rec,est_result[:2]))
                if len(self.lidar_cali_pos_rec) > 15:
                    pos_diff  = self.lidar_cali_pos_rec[1:]- self.lidar_cali_pos_rec[:-1]
                    std = np.sqrt(np.var(norm(pos_diff,axis=1)))
                    if std>1e-7 and std < 5e-2:
                        self.swich_lidar_pos = True
                        print("Trans to lidar pos")
                    self.lidar_cali_pos_rec = np.delete(self.lidar_cali_pos_rec,0,0)

if __name__ == '__main__':
    rospy.init_node('calibration', anonymous=True)
    calibrator = calibrate()
    while not rospy.is_shutdown():
        rospy.spin()
    show_calibration_figure = True
    # show the results
    if show_calibration_figure:
        # print(f"calibration result: pos {mic_ekf.x_est[:2]}, theta {mic_ekf.x_est[2]/np.pi*180}")
        print(f"calibration result: pos {calibrator.lidar_ekf.x_est[:2]},\n theta: {calibrator.lidar_ekf.x_est[2]/np.pi*180}")
        # figure
        print("mic pos error:",norm(calibrator.mic_ekf.x_est[:2].reshape(-1)-calibrator.mic_pos_theta[:2]))
        print("mic ang error:",abs(calibrator.mic_ekf.x_est[2]-calibrator.mic_pos_theta[2])/np.pi*180)
        print("lidar pos error:",norm(calibrator.lidar_ekf.x_est[:2].reshape(-1)-calibrator.lidar_pos_theta[:2]))
        print("lidar ang error:",abs(calibrator.lidar_ekf.x_est[2]-calibrator.lidar_pos_theta[2])/np.pi*180)
        # print("src pos error:",norm(calibrator.mic_ekf.x_est[3:5].reshape(-1)-calibrator.sound_source_pos))
        #Curve of error
        fig, axes = plt.subplots(2, 5, figsize=(10, 8))
        axes[0, 0].plot(range(len(calibrator.x_error_mic)), calibrator.x_error_mic, 'r-')
        axes[0, 0].set_title('mic-x error(m)')
        axes[0, 1].plot(range(len(calibrator.y_error_mic)), calibrator.y_error_mic, 'g-')
        axes[0, 1].set_title('mic-y error(m)')
        axes[0, 2].plot(range(len(calibrator.theta_error_mic)), calibrator.theta_error_mic, 'b-')
        axes[0, 2].set_title(r'mic-$\theta$ error($\circ$)')
        # axes[0, 3].plot(range(len(calibrator.x_error_src)), calibrator.x_error_src, 'b-')
        axes[0, 3].set_title('src-x error(m)')
        # axes[0, 4].plot(range(len(calibrator.y_error_src)), calibrator.y_error_src, 'b-')
        axes[0, 4].set_title('src-y error(m)')

        axes[1, 0].plot(range(len(calibrator.x_error_lidar)), calibrator.x_error_lidar, 'r-')
        axes[1, 0].set_title('lidar-x error(m)')
        axes[1, 1].plot(range(len(calibrator.y_error_lidar)), calibrator.y_error_lidar, 'g-')
        axes[1, 1].set_title('lidar-y error(m)')
        axes[1, 2].plot(range(len(calibrator.theta_error_lidar)), calibrator.theta_error_lidar, 'b-')
        axes[1, 2].set_title(r'lidar-$\theta$ error($\circ$)')

        for ax in axes.flatten():
            ax.axhline(y=0, color='k', linestyle='--')
        plt.tight_layout()
        plt.show()
    data_save_mic = h_merge([calibrator.x_error_mic.T,calibrator.y_error_mic.T,calibrator.theta_error_mic.T])
    data_save_lidar = h_merge([calibrator.x_error_lidar.T,calibrator.y_error_lidar.T,calibrator.theta_error_lidar.T])
