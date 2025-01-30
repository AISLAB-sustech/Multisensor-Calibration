# -*- coding: utf -8 -*-
# @ Author : Jiang WANG

import numpy as np
from scipy.linalg import norm
from scipy.optimize import minimize
import matplotlib.pyplot as plt

def rot_mat(theta,de=False):
    '''
    :param theta:
    :param de: derivative (True/False)
    :return: rotation matrix
    '''
    if de:
        matrix = np.array([[-np.sin(theta),-np.cos(theta)],
              [np.cos(theta),-np.sin(theta)]])
    else:
        matrix = np.array([[np.cos(theta),-np.sin(theta)],
              [np.sin(theta),np.cos(theta)]])
    return matrix
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
def Jacbian(delta,SRC,theta_M,p_M,theta_L,p_L,x_i,theta_i,x_iplus1,theta_iplus1):
    '''
    :param delta: simulation time interval
    :param SRC: sound source position
    :param theta_M: robot to mic orientation
    :param p_M:     robot to mic translation
    :param theta_L: robot to lidar orientation
    :param p_L:     robot to lidar translation
    :param x_i:     robot position in global frame
    :param theta_i: robot oritation in global frame
    :param v_i:     robot speed
    :param x_iplus1:next robot position
    :param theta_iplus1: next robot oritation
    :return:
    '''
    R_i = rot_mat(theta_i)
    R_iplus1 = rot_mat(theta_iplus1)
    delta_M_i = SRC - (x_i + R_i @ p_M)
    delta_M_iplus1 = SRC - (x_iplus1 + R_iplus1 @ p_M)
    # Jacobian matrix w.r.t sensors parameters
    T_i = np.zeros((5, 6))
    # w.r.t MIC parameters    
    T_i[:2, :2] =(R_i @ rot_mat(theta_M)).T / norm(delta_M_i) ** 3 @ (-R_i * norm(delta_M_i) ** 2 + np.array(
    [[delta_M_i[0, 0] ** 2, delta_M_i[0, 0] * delta_M_i[1, 0]],
        [delta_M_i[0, 0] * delta_M_i[1, 0], delta_M_i[1, 0] ** 2]]) @ R_i)
    T_i[:2, 2:3] = rot_mat(theta_M, True).T @ R_i.T @ (delta_M_i) / norm(delta_M_i)

    # w.r.t Lidar parameters
    T_i[2:4, 3:5] = (R_i @ rot_mat(theta_L)).T @ (R_iplus1 - R_i)
    T_i[2:4, 5:6] = rot_mat(theta_L, True).T @ R_i.T @ (R_iplus1 @ p_L + x_iplus1 - R_i @ p_L - x_i)
    return T_i

def bspline(control_points,delta):
    k = 3
    path = np.zeros((0, 2))
    yaw_angles = np.array([])
    # parameter of B-spline, see thesis
    M0 = np.array([[1, 0, 0, 0],
                   [-3, 3, 0, 0],
                   [3, -9 / 2, 3 / 2, 0],
                   [-1, 7 / 4, -11 / 12, 1 / 6]])
    M1 = np.array([[1 / 4, 7 / 12, 1 / 6, 0],
                   [-3 / 4, 1 / 4, 1 / 2, 0],
                   [3 / 4, -5 / 4, 1 / 2, 0],
                   [-1 / 4, 7 / 12, -1 / 2, 1 / 6]])
    M_n_1 = np.array([[1 / 6, 2 / 3, 1 / 6, 0],
                      [-1 / 2, 0, 1 / 2, 0],
                      [1 / 2, -1, 1 / 2, 0],
                      [-1 / 6, 1 / 2, -7 / 12, 1 / 4]])
    M_n = np.array([[1 / 6, 7 / 12, 1 / 4, 0],
                    [-1 / 2, -1 / 4, 3 / 4, 0],
                    [1 / 2, -5 / 4, 3 / 4, 0],
                    [-1 / 6, 11 / 12, -7 / 4, 1]])
    M_else = np.array([[1 / 6, 2 / 3, 1 / 6, 0],
                       [-1 / 2, 0, 1 / 2, 0],
                       [1 / 2, -1, 1 / 2, 0],
                       [-1 / 6, 1 / 2, -1 / 2, 1 / 6]])
    for segment in range(1, len(control_points) - k + 1):
        # different parameter in different segments
        if segment == 1:
            M = M0
        elif segment == 2:
            M = M1
        elif segment == len(control_points) - k - 1:
            M = M_n_1
        elif segment == len(control_points) - k:
            M = M_n
        else:
            M = M_else
        # four control points corresponding the current curve segment
        control_point = control_points[segment - 1:segment + 3]
        # B-spline position/yaw angle/velocity/acceleration （global frame）
        for i in np.arange(0, 1, delta):
            p = np.array([1, i, i ** 2, i ** 3]) @ M @ control_point
            v =  1*np.array([0, 1, 2 * i, 3 * (i ** 2)]) @ M @ control_point
            path = np.vstack((path, p))
            yaw = np.arctan2(v[1], v[0])
            yaw_angles = np.append(yaw_angles, yaw)
    return path,yaw_angles
def constraint_angle(control_points,pre_point,ext_para,curve_seg):
    delta = 0.2
    control_points = np.append(pre_point, control_points).reshape(-1, 2)
    path, yaw_angles= bspline(control_points, delta )
    constraints = []
    for i in range(1,len(yaw_angles)):
        constraints.append(50/180.0*np.pi-abs(yaw_angles[i]-yaw_angles[i-1]))
    return constraints
def constraint_pos1(control_points,pre_point,ext_para,curve_seg):
    constraints = []
    for i in range(len(control_points)):
        constraints.append(control_points[i])
    return constraints
def constraint_pos2(control_points,pre_point,ext_para,curve_seg):
    constraints = []
    for i in range(len(control_points)):
        constraints.append(2-control_points[i])
    return constraints
def max_negative_min_eigenvalue(control_point,pre_point,ext_para,src):
    control_points = np.append(pre_point,control_point).reshape(-1,2)
    delta = 0.5                                                                # Downsampling
    path, yaw_angles = bspline(control_points, delta)
    # calibration parameter
    p_M = ext_para[0][0:2].reshape(-1,1)                              
    theta_M = ext_para[0][2,0]   
    SRC = src.reshape(-1,1)                  
    p_L = ext_para[1][0:2].reshape(-1,1)                               
    theta_L = ext_para[1][2,0]    
    opt_jac = np.zeros((0,6))   
    # jacobian matrix
    for i in range(len(path) - 1):
        x_i = path[i].reshape(2, 1)
        theta_i = yaw_angles[i]
        x_iplus1 = path[i + 1].reshape(2, 1)
        theta_iplus1 = yaw_angles[i + 1]
        T_i = Jacbian(delta, SRC, theta_M, p_M, theta_L, p_L, x_i, theta_i, x_iplus1, theta_iplus1)
        opt_jac = v_merge([opt_jac,T_i])
    Full_jac = opt_jac.T @ opt_jac
    _, s, _ = np.linalg.svd(Full_jac)
    save_value(s.min())
    return -s.min()         

min_eigen = []
def save_value(value):
    global min_eigen
    min_eigen.append(value)

def optimize_bspline(delta,curve_seg,pre_point,ext_para,src):
    global min_eigen
    if curve_seg == 1:
        control_points = pre_point[1:].reshape(-1)
        pre_point      = pre_point[0]
    else:
        control_points = pre_point[curve_seg+2:].reshape(-1)
        pre_point      = pre_point[:curve_seg+2]
    constraint = [{'type': 'ineq', 'fun': constraint_angle,'args':(pre_point,ext_para,curve_seg)},
                  {'type': 'ineq', 'fun': constraint_pos1,'args':(pre_point,ext_para,curve_seg)},
                  {'type': 'ineq', 'fun': constraint_pos2,'args':(pre_point,ext_para,curve_seg)}]
    result = minimize(max_negative_min_eigenvalue, control_points,args=(pre_point,ext_para,src),constraints=constraint, method="COBYLA")
    control_points = result.x

    control_points = np.append(pre_point,control_points).reshape(-1,2)
    path,yaw_angles = bspline(control_points,delta)
    path = path[int(1.0/delta*(curve_seg-1)):int(1.0/delta*(curve_seg))]
    yaw_angles = yaw_angles[int(1.0/delta*(curve_seg-1)):int(1.0/delta*(curve_seg))]

    # plt.plot(min_eigen,label='min eigen value')
    # plt.xlabel('X/Trajectory Optimization Iteration')
    # plt.ylabel('Y/Min Eigen Value')
    # plt.show()
    return  path,yaw_angles,control_points


