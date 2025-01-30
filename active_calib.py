# -*- coding: utf -8 -*-
# @ Author : Jiang WANG

import numpy as np
import matplotlib.pyplot as plt
from  scipy.linalg import norm
from ob_traj_opt import *
from scipy.io import savemat
from utils import *

delta = 0.01                # sampling time

# g.t.
mic_pos_theta, lidar_pos_theta,src = get_gt()

show_calibration_figure = config['settings']['show_calibration_figure']

data_dict = {}
Monte_Carlo_result = np.zeros((0,6))

# Monte Carlo simulation
for iter in range(200):
    # EKF initialization
    mic_noise,lidar_pos_noise,lidar_theta_noise,wheel_noise,x0,P0,x0_for_lidar,P0_for_lidar,Q_mic,Q_lidar = get_ekf_parameter()
    mic_ekf = e_kalman_filter(Q_mic, x0, P0,"mic",src.reshape(-1,1))
    lidar_ekf = e_kalman_filter(Q_lidar, x0_for_lidar, P0_for_lidar,"lidar")

    # Record Ekf Error iteration parameters (for plot)
    x_error_mic,x_P_mic,x_error_lidar,x_P_lidar,\
    y_error_mic,y_P_mic,y_error_lidar,y_P_lidar,\
    theta_error_mic,theta_P_mic,theta_error_lidar,theta_P_lidar= record_variable()

    # Traj. Initialization
    control_points = np.array([[0,0],[1.0,1], [2.0,2], [3.0,3], [4,4],[5,5],
                            [6,6],[7,7],[8,8],[9,9]],dtype="float32")/5.0
    all_path      = np.zeros((0,2))
    all_yaw_angle = np.zeros(0)
    all_mic_theta = np.zeros(0)
    all_lidar_theta = np.zeros(0)

    # Used for judge EKF state
    delta_x = np.zeros((6,1))
    converge = False
    diverge_count = 0
    diverge  = False

    # Calibration
    print(f"{iter+1}-ROUND EST RESULTS")
    for segment in range(1,10-3+1):
        # Update traj
        ext_para = [x0,x0_for_lidar]
        path,yaw_angles,control_points = optimize_bspline(delta,segment,control_points,ext_para,src)    # generate bspline path
        all_path = np.vstack((all_path,path))
        all_yaw_angle = np.append(all_yaw_angle,yaw_angles)
        
        # Sensor measurement
        doa_mea,lidar_pos_mea,lidar_theta_mea = get_measurement(path,yaw_angles,wheel_noise,mic_pos_theta,src,mic_noise,lidar_pos_theta,lidar_pos_noise,lidar_theta_noise)
            
        # ekf measurements update
        mic_ekf.z     = [doa_mea[1:]]
        mic_ekf.path  = path
        mic_ekf.yaw_angles = yaw_angles 
        lidar_ekf.z = [lidar_pos_mea,lidar_theta_mea]
        lidar_ekf.path= path
        lidar_ekf.yaw_angles = yaw_angles 
        for i  in range(len(doa_mea)-1):
            # EKF update with the newest data
            # Record Estimate error
            error_mic   = mic_ekf.x_est.reshape(-1)-mic_pos_theta
            error_lidar = lidar_ekf.x_est.reshape(-1) - lidar_pos_theta
            x_error_mic,y_error_mic,theta_error_mic        = get_xytheta_error(x_error_mic,y_error_mic,theta_error_mic,error_mic)
            x_error_lidar, y_error_lidar,theta_error_lidar = get_xytheta_error(x_error_lidar, y_error_lidar,theta_error_lidar,error_lidar)
            x_P_mic,y_P_mic, theta_P_mic        = get_xytheta_P(x_P_mic,y_P_mic, theta_P_mic, mic_ekf.P_est)
            x_P_lidar,y_P_lidar, theta_P_lidar  = get_xytheta_P(x_P_lidar,y_P_lidar, theta_P_lidar, lidar_ekf.P_est)
            
            pre_mic = mic_ekf.x_est
            pre_lidar = lidar_ekf.x_est

            mic_ekf.update(i)
            lidar_ekf.update(i)

            delta_x[:3]  = mic_ekf.x_est-pre_mic
            delta_x[2]   = signed_angle_difference(mic_ekf.x_est[2]/np.pi*180,pre_mic[2]/np.pi*180)
            delta_x[3:6] = lidar_ekf.x_est-pre_lidar
            delta_x[5]   = signed_angle_difference(lidar_ekf.x_est[2]/np.pi*180,pre_lidar[2]/np.pi*180)

            norm_delta   = norm(delta_x)
            if norm_delta < 5e-4:
                converge = True
                break
            elif norm_delta > 10:
                diverge_count +=1
                if diverge_count > 15:
                    diverge  = True
                    print(f"segment: {segment}, round: {i}, norm: {norm_delta}")
                    break
        x0 = mic_ekf.x_est
        x0_for_lidar=lidar_ekf.x_est
        if converge == True:
            print("converge!**********************")
            break
        elif diverge == True:
            print("diverge!**********************")
            diverge  = False
            break

    print("mic pos error:",norm(mic_ekf.x_est[:2].reshape(-1)-mic_pos_theta[:2]))
    print("mic ang error:",abs(mic_ekf.x_est[2]-mic_pos_theta[2])/np.pi*180)
    print("lidar pos error:",norm(lidar_ekf.x_est[:2].reshape(-1)-lidar_pos_theta[:2]))
    print("lidar ang error:",abs(lidar_ekf.x_est[2]-lidar_pos_theta[2])/np.pi*180)

    lidar_data = h_merge([x_error_lidar,y_error_lidar,theta_error_lidar,x_P_lidar,y_P_lidar,theta_P_lidar])
    mic_data   = h_merge([x_error_mic,y_error_mic,theta_error_mic,x_P_mic,y_P_mic,theta_P_mic])
    data_dict[f"lidar_{iter+1}"] = lidar_data
    data_dict[f"mic_{iter+1}"]   = mic_data
    Monte_Carlo_result = v_merge([Monte_Carlo_result,
                                  h_merge([mic_data[-1,:3].reshape(1,-1),lidar_data[-1,:3].reshape(1,-1)])
                                  ])
    # show the results
    if show_calibration_figure:
        plot_traj(all_path,all_yaw_angle,control_points)
        plot_cali_result(x_error_mic,x_P_mic,y_error_mic,y_P_mic,theta_error_mic,theta_P_mic,
                        x_error_lidar,x_P_lidar,y_error_lidar,y_P_lidar,theta_error_lidar,theta_P_lidar)
        data = np.zeros((len(x_error_mic),6))
        data[:,0] = x_error_mic.reshape(-1)
        data[:,1] = y_error_mic.reshape(-1)
        data[:,2] = theta_error_mic.reshape(-1)
        data[:,3] = x_error_lidar.reshape(-1)
        data[:,4] = y_error_lidar.reshape(-1)
        data[:,5] = theta_error_lidar.reshape(-1)

final_cali_results = norm(Monte_Carlo_result,axis=0)/np.sqrt(len(Monte_Carlo_result))
print(f"mic_error:{final_cali_results[0]: .6f},{final_cali_results[1]: .6f},{final_cali_results[2]: .6f}")
print(f"lidar_error:{final_cali_results[3]: .6f},{final_cali_results[4]: .6f},{final_cali_results[5]: .6f}")

