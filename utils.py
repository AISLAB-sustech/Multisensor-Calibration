import yaml
import numpy as np
from ob_traj_opt import *

with open("config.yaml", "r") as file:
    config = yaml.safe_load(file)

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

def mic_mea(mic_pos_set,mic_ori_set,src):
    # DoA measurements(Sound source direction of arrival w.r.t mic. array)
    # This could be measured by SRP-PHAT algorithm
    doa = np.zeros((0,2))
    for i in range(len(mic_pos_set)):
        gt = rot_mat(mic_ori_set[i]).T@(src-mic_pos_set[i])/norm(src-mic_pos_set[i])
        doa = np.vstack((doa,gt))
    return doa

def lidar_mea(lidar_pos_set,lidar_ori_set):
    # Lidar relative orientation and translation
    # This could be measured by ICP algorithm
    delta_theta = np.zeros((0,1))
    delta_pos   = np.zeros((0,2))
    for i in range(len(lidar_pos_set)-1):
        delta_theta = np.vstack((delta_theta,lidar_ori_set[i+1,0]-lidar_ori_set[i,0]))
        delta_pos_gt= rot_mat(lidar_ori_set[i,0]).T@(lidar_pos_set[i+1]-lidar_pos_set[i])
        delta_pos = np.vstack((delta_pos,delta_pos_gt))
    return  delta_theta,delta_pos

# Jacobian matrix of measurement model
def h_jacobian(p_sensor,theta_sensor,x_i,theta_i,x_iplus1,theta_iplus1,type,SRC=None):
    R_i = rot_mat(theta_i)
    R_iplus1 = rot_mat(theta_iplus1)
    if type=="mic":
        delta_M_i = SRC - (x_i + R_i @ p_sensor)
        T_i = np.zeros((2, 3))
        T_i[:2, :2] =(R_i @ rot_mat(theta_sensor)).T / norm(delta_M_i) ** 3 @ (-R_i * norm(delta_M_i) ** 2 + np.array(
            [[delta_M_i[0, 0] ** 2, delta_M_i[0, 0] * delta_M_i[1, 0]],
             [delta_M_i[0, 0] * delta_M_i[1, 0], delta_M_i[1, 0] ** 2]]) @ R_i)
        T_i[:2, 2:3] = rot_mat(theta_sensor, True).T @ R_i.T @ (delta_M_i) / norm(delta_M_i)
    elif type=="lidar":
        T_i = np.zeros((3, 3))
        T_i[:2, :2] = (R_i @ rot_mat(theta_sensor)).T @ (R_iplus1 - R_i)
        T_i[:2, 2:3] = rot_mat(theta_sensor, True).T @ R_i.T @ (R_iplus1 @ p_sensor + x_iplus1 - R_i @ p_sensor - x_i)
    return T_i

def add_wheel_noise(path,yaw_angle,wheel_noise):
    delta_pos = path[0]
    delta_pos = v_merge([delta_pos, path[1:]-path[:-1]])

    delta_yaw = yaw_angle[1:]-yaw_angle[:-1]
    delta_yaw = np.insert(delta_yaw,0,yaw_angle[0])

    path_with_noise = np.zeros_like(path)
    yaw_angle_with_noise = np.zeros_like(yaw_angle)

    for i in range(len(path)):
        if i == 0:
            path_with_noise[i,0] = delta_pos[i,0] + np.random.normal(0,abs(wheel_noise*delta_pos[i,0]))
            path_with_noise[i,1] = delta_pos[i,1] + np.random.normal(0,abs(wheel_noise*delta_pos[i,1]))
            yaw_angle_with_noise[i] = delta_yaw[i] + np.random.normal(0,abs(wheel_noise*delta_yaw[i]))
        else:
            path_with_noise[i,0] = path_with_noise[i-1,0]+ delta_pos[i,0] + np.random.normal(0,abs(wheel_noise*delta_pos[i,0]))
            path_with_noise[i,1] = path_with_noise[i-1,1]+ delta_pos[i,1] + np.random.normal(0,abs(wheel_noise*delta_pos[i,1]))
            yaw_angle_with_noise[i] = yaw_angle_with_noise[i-1]+ delta_yaw[i] + np.random.normal(0,abs(wheel_noise*delta_yaw[i]))
    return path_with_noise,yaw_angle_with_noise
# EKF for estimation sensors parameters
class e_kalman_filter(object):
    def __init__(self, Q, x0, P0,sensor,src=None):
        self.Q = Q
        self.z = None
        self.x_est = x0
        self.P_est = P0
        self.path = None
        self.yaw_angles=None
        self.type = sensor
        self.SRC = src

    def update(self,k):
        self.x_pred = self.x_est
        self.P_pred = self.P_est
        self.p_sensor = self.x_pred[0:2].reshape(-1, 1)
        self.theta_sensor = self.x_pred[2, 0]

        self.x_i = self.path[k].reshape(-1, 1)
        self.x_iplus1 = self.path[k + 1].reshape(-1, 1)
        self.theta_i = self.yaw_angles[k]
        self.theta_iplus1 = self.yaw_angles[k + 1]

        mea_model_diff = np.zeros((0, 1))
        if self.type == "lidar":
            delta_pos = (rot_mat(self.theta_i) @ rot_mat(self.theta_sensor)).T@ \
                        (rot_mat(self.theta_iplus1)@self.p_sensor+self.x_iplus1-rot_mat(self.theta_i)@self.p_sensor-self.x_i)
            mea_model_diff = np.vstack((mea_model_diff, (self.z[0][k].reshape(-1, 1) - delta_pos)))
            mea_model_diff = np.vstack((mea_model_diff, (self.z[1][k] - (self.yaw_angles[k+1]-self.yaw_angles[k])).reshape(-1, 1)))
            self.H = h_jacobian(self.p_sensor,self.theta_sensor,self.x_i,self.theta_i,self.x_iplus1,self.theta_iplus1,self.type)
        elif self.type == "mic":
            doa = (rot_mat(self.theta_i) @ rot_mat(self.theta_sensor)).T @ (self.SRC - (self.x_i + rot_mat(self.theta_i) @ self.p_sensor)) / norm(
                self.SRC - (self.x_i + rot_mat(self.theta_i) @ self.p_sensor))
            mea_model_diff = np.vstack((mea_model_diff, (self.z[0][k].reshape(-1, 1) - doa)))
            self.H = h_jacobian(self.p_sensor,self.theta_sensor,self.x_i,self.theta_i,self.x_iplus1,self.theta_iplus1,self.type,self.SRC)

        # Kalman gain
        self.K = self.P_pred @ self.H.T @ np.linalg.inv(self.H @ self.P_pred @ self.H.T + self.Q)
        # Posterior probability distribution
        self.x_est = self.x_pred + self.K @ mea_model_diff
        if self.x_pred[2] > 2*np.pi:
            self.x_pred[2] = self.x_pred[2] - 2*np.pi
        elif self.x_pred[2] < 0:
            self.x_pred[2] = self.x_pred[2] + 2*np.pi
        self.P_est = (np.eye(len(self.x_est)) - self.K @ self.H) @ self.P_pred

def angle_difference(angle1, angle2):
    diff = abs(angle1 - angle2) % 360
    return min(diff, 360 - diff)

def signed_angle_difference(angle1, angle2):
    diff = (angle2 - angle1) % 360
    if diff > 180:
        diff -= 360
    return diff

def get_gt():
    mic_pos_theta    = np.array(config['ground true']['mic_pos_theta'])
    mic_pos_theta[2] = mic_pos_theta[2]/180*np.pi
    lidar_pos_theta = np.array(config['ground true']['lidar_pos_theta'])
    lidar_pos_theta[2] = lidar_pos_theta[2]/180*np.pi
    src = np.array(config['settings']['sound_source'])     # a known sound source position
    return mic_pos_theta, lidar_pos_theta,src

def get_ekf_parameter():
    mic_noise = config['settings']['mic_noise']
    lidar_pos_noise = config['settings']['lidar_pos_noise']
    lidar_theta_noise = config['settings']['lidar_theta_noise']/180*np.pi
    wheel_noise = config['settings']['wheel_noise']/100.0

    x0 = np.array([0.0,0.0,0/180*np.pi]).reshape(-1,1)      # initial value
    P0 = np.eye(3)
    # np.diag([0.00013443, 0.00017343])                    # measurement covariance 
    # 2 degree: np.diag([0.00080698, 0.000197])
    # 4 degree: np.diag([0.00248057, 0.00150222])
    # 6 degree: np.diag([0.00795189, 0.00103064]) 
    # 8 degree: np.diag([0.01032782, 0.00549124])        
    Q_mic =    np.diag([0.01159162, 0.01293283]) 
    Q_lidar =  np.diag([lidar_pos_noise**2,lidar_pos_noise**2,lidar_theta_noise**2])
    return mic_noise,lidar_pos_noise,lidar_theta_noise,wheel_noise,x0,P0,x0,P0,Q_mic,Q_lidar

def record_variable():
    x_error_mic   = np.zeros((0,1))
    x_P_mic       = np.zeros((0,1))
    x_error_lidar = np.zeros((0,1))
    x_P_lidar      = np.zeros((0,1))
    y_error_mic   = np.zeros((0,1))
    y_P_mic      = np.zeros((0,1))
    y_error_lidar = np.zeros((0,1))
    y_P_lidar      = np.zeros((0,1))
    theta_error_mic   = np.zeros((0,1))
    theta_P_mic      = np.zeros((0,1))
    theta_error_lidar = np.zeros((0,1))
    theta_P_lidar     = np.zeros((0,1))
    return x_error_mic,x_P_mic,x_error_lidar,x_P_lidar,y_error_mic,y_P_mic,y_error_lidar,y_P_lidar,theta_error_mic,theta_P_mic,theta_error_lidar,theta_P_lidar

def generate_circle_point(point_A,point_B):
    center_x, center_y = (point_A[0] + point_B[0]) / 2, (point_A[1] + point_B[1]) / 2
    radius = norm(point_B-point_A)/2
    angle = np.arctan2(point_B[1] - point_A[1], point_B[0] - point_A[0])
    points = []
    yaw_angles = np.array([])
    for angle_offset in np.linspace(0,180,700):
        current_angle = angle+np.pi - angle_offset/180*np.pi
        x = center_x + radius * np.cos(current_angle)
        y = center_y + radius * np.sin(current_angle)
        points.append([x, y])
        yaw_angles = np.append(yaw_angles, current_angle-np.pi/2)
    return np.array(points[::-1]),yaw_angles[::-1]

def get_measurement(path,yaw_angles,wheel_noise,mic_pos_theta,src,mic_noise,lidar_pos_theta,lidar_pos_noise,lidar_theta_noise):
    # Wheel encoder measurement
    path_with_noise,yaw_angle_with_noise = add_wheel_noise(path,yaw_angles,wheel_noise)
    
    # Sensors parameter in global frame
    mic_pos_set,mic_ori_set = get_sensor_pos_ori(path_with_noise,yaw_angle_with_noise,mic_pos_theta[:2],mic_pos_theta[2])
    lidar_pos_set,lidar_ori_set = get_sensor_pos_ori(path_with_noise,yaw_angle_with_noise,lidar_pos_theta[:2],lidar_pos_theta[2])

    # Theoretical measurements：
    doa = mic_mea(mic_pos_set,mic_ori_set,src)                                     # mic
    lidar_delta_theta,lidar_delta_pos = lidar_mea(lidar_pos_set,lidar_ori_set)     # lidar

    # measurements with Gussion noise
    mic_theta = np.arctan2(doa[:,1],doa[:,0])/np.pi*180+np.random.normal(0,mic_noise,(len(doa)))
    doa_mea = np.zeros_like(doa)
    doa_mea[:,0]=np.cos(mic_theta/180*np.pi)
    doa_mea[:,1]=np.sin(mic_theta/180*np.pi)
    lidar_pos_mea = lidar_delta_pos+np.random.normal(0,lidar_pos_noise,(len(lidar_delta_pos),2))
    lidar_theta_mea = lidar_delta_theta+np.random.normal(0,lidar_theta_noise,(len(lidar_delta_theta),1))
    return doa_mea,lidar_pos_mea,lidar_theta_mea

def get_xytheta_error(x_error,y_error,theta_error,mea_error):
    x_error = np.vstack((x_error,mea_error[0]))
    y_error = np.vstack((y_error,mea_error[1]))
    theta_diff = (mea_error[2]/np.pi*180) % 360
    if theta_diff > 180:
        theta_diff -= 360
    theta_error = np.vstack((theta_error, theta_diff))
    # theta_error = np.vstack((theta_error, min(theta_diff,360-theta_diff)))
    return x_error,y_error,theta_error

def get_xytheta_P(x_P,y_P,theta_P,P_est):
    x_P     = np.vstack((x_P , P_est[0,0]))
    y_P     = np.vstack((y_P , P_est[1,1]))
    theta_P = np.vstack((theta_P, P_est[2,2]))
    return x_P,y_P, theta_P

def plot_traj(all_path,all_yaw_angle,control_points):
    x_points=all_path[:,0]
    y_points=all_path[:,1]

    plt.plot(control_points[:, 0], control_points[:, 1], 'ro-', label='Control Points')
    plt.plot(x_points, y_points, '-', label='B-spline Curve')
    plt.quiver(x_points[::50], y_points[::50], np.cos(all_yaw_angle[::50]), np.sin(all_yaw_angle[::50]), color='blue', scale=15)
    plt.legend()
    plt.axis('equal')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('B-spline Curve')
    plt.grid()
    plt.show()

def plot_cali_result(x_error_mic,x_P_mic,y_error_mic,y_P_mic,theta_error_mic,theta_P_mic,x_error_lidar,x_P_lidar,y_error_lidar,y_P_lidar,theta_error_lidar,theta_P_lidar):
    #Curve of error
    fig, axes = plt.subplots(2, 3, figsize=(10, 8))
    axes[0, 0].plot(range(len(x_error_mic)), x_error_mic, 'r-')
    axes[0, 0].plot(range(len(x_error_mic)), x_error_mic+3*np.sqrt(x_P_mic), 'k-')
    axes[0, 0].plot(range(len(x_error_mic)), x_error_mic-3*np.sqrt(x_P_mic), 'k-')
    axes[0, 0].set_title('mic-x error(m)')

    axes[0, 1].plot(range(len(y_error_mic)), y_error_mic, 'g-')
    axes[0, 1].plot(range(len(y_error_mic)), y_error_mic+3*np.sqrt(y_P_mic), 'k-')
    axes[0, 1].plot(range(len(y_error_mic)), y_error_mic-3*np.sqrt(y_P_mic), 'k-')
    axes[0, 1].set_title('mic-y error(m)')

    axes[0, 2].plot(range(len(theta_error_mic)), theta_error_mic, 'b-')
    axes[0, 2].plot(range(len(theta_error_mic)), theta_error_mic+3*np.sqrt(theta_P_mic/np.pi*180), 'k-')
    axes[0, 2].plot(range(len(theta_error_mic)), theta_error_mic-3*np.sqrt(theta_P_mic/np.pi*180), 'k-')
    axes[0, 2].set_title(r'mic-$\theta$ error($\circ$)')

    axes[1, 0].plot(range(len(x_error_lidar)), x_error_lidar, 'r-')
    axes[1, 0].plot(range(len(x_error_lidar)), x_error_lidar+3*np.sqrt(x_P_lidar), 'k-')
    axes[1, 0].plot(range(len(x_error_lidar)), x_error_lidar-3*np.sqrt(x_P_lidar), 'k-')
    axes[1, 0].set_title('lidar-x error(m)')

    axes[1, 1].plot(range(len(y_error_lidar)), y_error_lidar, 'g-')
    axes[1, 1].plot(range(len(y_error_lidar)), y_error_lidar+3*np.sqrt(y_P_lidar), 'k-')
    axes[1, 1].plot(range(len(y_error_lidar)), y_error_lidar-3*np.sqrt(y_P_lidar), 'k-')
    axes[1, 1].set_title('lidar-y error(m)')

    axes[1, 2].plot(range(len(theta_error_lidar)), theta_error_lidar, 'b-')
    axes[1, 2].plot(range(len(theta_error_lidar)), theta_error_lidar+3*np.sqrt(theta_P_lidar/np.pi*180), 'k-')
    axes[1, 2].plot(range(len(theta_error_lidar)), theta_error_lidar-3*np.sqrt(theta_P_lidar/np.pi*180), 'k-')
    axes[1, 2].set_title(r'lidar-$\theta$ error($\circ$)')

    for ax in axes.flatten():
        ax.axhline(y=0, color='k', linestyle='--')
    plt.tight_layout()
    plt.show()