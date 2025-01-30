#ifndef PID_POSITION_FOLLOW_H
#define PID_POSITION_FOLLOW_H

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

#include "utility.h"
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2/utils.h>

#include "cubic_spline/cubic_spline_ros.h"
#include "utility.h"
#include <Eigen/Eigen>
#include <chrono>

class RobotCtrl {
public:
  RobotCtrl();
  ~RobotCtrl() = default;
  void JointstateCallback(const sensor_msgs::JointStateConstPtr &msg);
  void GimbalCtrl();
  void GlobalPathCallback(const nav_msgs::PathConstPtr &msg);

  void FollowTraj(const geometry_msgs::PoseStamped &robot_pose,
                  const nav_msgs::Path &traj, geometry_msgs::Twist &cmd_vel);
  void FindNearstPose(geometry_msgs::PoseStamped &robot_pose,
                      nav_msgs::Path &path, int &prune_index,
                      double prune_ahead_dist);
  void SegmentsCallback(const std_msgs::Int32MultiArray &msg);
  void Plan(const ros::TimerEvent &event);

private:
  ros::Publisher cmd_vel_pub_;
  ros::Publisher local_path_pub_;
  ros::Publisher segment_pub_;

  ros::Subscriber global_path_sub_;
  ros::Subscriber segments_sub_;
  ros::Timer plan_timer_;

  ros::ServiceServer planner_server_;

  std::shared_ptr<tf::TransformListener> tf_listener_;
  tf::StampedTransform global2path_transform_;

  nav_msgs::Path global_path_;
  std::vector<int> segments_;

  bool plan_ = false;
  bool initialized_ = false;
  int prune_index_ = 0;

  double max_x_speed_;
  double max_y_speed_;

  double set_yaw_speed_;

  double p_x_value_;
  double i_x_value_;
  double d_x_value_;

  double p_yaw_value_;
  double i_yaw_value_;
  double d_yaw_value_;

  int plan_freq_;
  double goal_dist_tolerance_;
  double prune_ahead_dist_;

  double last_error_distance_;
  double integral_error_distance_;
  double last_error_yaw_;
  double integral_error_yaw_;

  std::string global_frame_;

  double yaw_; //机器人航向角
};

double normalizeRadian(const double angle) {
  double n_angle = std::fmod(angle, 2 * M_PI);
  n_angle = n_angle > M_PI ? n_angle - 2 * M_PI
                           : n_angle < -M_PI ? 2 * M_PI + n_angle : n_angle;
  return n_angle;
}

double ABS_limit(double value, double limit) {
  if (value < limit && value > -limit) {
    return 0;
  } else {
    return value;
  }
}

#endif
