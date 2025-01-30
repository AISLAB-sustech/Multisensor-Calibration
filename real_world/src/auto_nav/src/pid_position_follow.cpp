#include "pid_position_follow.h"

RobotCtrl::RobotCtrl() {
  ros::NodeHandle nh("~");
  ;
  nh.param<double>("max_x_speed", max_x_speed_, 1.0);
  nh.param<double>("max_y_speed", max_y_speed_, 0.0);
  nh.param<double>("set_yaw_speed", set_yaw_speed_, 0.2);

  nh.param<double>("p_x_value", p_x_value_, 0.5);
  nh.param<double>("i_x_value", i_x_value_, 1);
  nh.param<double>("d_x_value", d_x_value_, 1);
  nh.param<double>("p_yaw_value", p_yaw_value_, 0.5);
  nh.param<double>("i_yaw_value", i_yaw_value_, 1);
  nh.param<double>("d_yaw_value", d_yaw_value_, 1);

  nh.param<int>("plan_frequency", plan_freq_, 30);
  nh.param<double>("goal_dist_tolerance", goal_dist_tolerance_, 0.2);
  nh.param<double>("prune_ahead_distance", prune_ahead_dist_, 0.5);
  nh.param<std::string>("global_frame", global_frame_, "map");

  local_path_pub_ = nh.advertise<nav_msgs::Path>("path", 5);
  global_path_sub_ =
      nh.subscribe("/optimized_path", 5, &RobotCtrl::GlobalPathCallback, this);
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/base_vel", 10);
  segment_pub_ = nh.advertise<std_msgs::Int32>("/segmented_path", 1);
  segments_sub_ =
      nh.subscribe("/segments", 1, &RobotCtrl::SegmentsCallback, this);
  tf_listener_ = std::make_shared<tf::TransformListener>();
  plan_timer_ =
      nh.createTimer(ros::Duration(1.0 / plan_freq_), &RobotCtrl::Plan, this);

  std::vector<int> temp_segments;
  for (int i = 0; i < 1000; i++) {
    temp_segments.push_back(1);
  }
  segments_ = temp_segments;
  last_error_distance_ = 0.0;
  integral_error_distance_ = 0.0;
  last_error_yaw_ = 0.0;
  integral_error_yaw_ = 0.0;
}

void RobotCtrl::SegmentsCallback(const std_msgs::Int32MultiArray &msg) {
  if (!msg.data.empty()) {
    std::vector<int> temp_segments;
    for (int i = 0; i < msg.data.size(); i++) {
      temp_segments.push_back(msg.data[i]);
    }
    segments_ = temp_segments;
  }
}

void RobotCtrl::Plan(const ros::TimerEvent &event) {

  if (plan_) {
    auto begin = std::chrono::steady_clock::now();
    auto start = ros::Time::now();
    // 1. Update the transform from global path frame to local planner frame
    UpdateTransform(tf_listener_, global_frame_, global_path_.header.frame_id,
                    global_path_.header.stamp,
                    global2path_transform_); // source_time needs decided
    std::cout << ros::Time::now() - start << std::endl;

    // 2. Get current robot pose in global path frame
    geometry_msgs::PoseStamped robot_pose;
    GetGlobalRobotPose(tf_listener_, global_path_.header.frame_id, robot_pose);

    // 3. Check if robot has already arrived with given distance tolerance
    if (GetEuclideanDistance(robot_pose, global_path_.poses.back()) <=
            goal_dist_tolerance_ ||
        prune_index_ == global_path_.poses.size() - 1) {
      plan_ = false;
      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = 0;
      cmd_vel.linear.y = 0;
      cmd_vel.angular.z = 0;
      cmd_vel.linear.z = 1; // bool success or not
      cmd_vel_pub_.publish(cmd_vel);
      ROS_INFO("Planning Success!");
      return;
    }

    // 4. Get prune index from given global path
    FindNearstPose(robot_pose, global_path_, prune_index_, prune_ahead_dist_);

    // 5. Generate the prune path and transform it into local planner frame
    nav_msgs::Path prune_path, local_path;

    local_path.header.frame_id = global_frame_;
    prune_path.header.frame_id = global_frame_;

    geometry_msgs::PoseStamped tmp_pose;
    tmp_pose.header.frame_id = global_frame_;

    TransformPose(global2path_transform_, robot_pose, tmp_pose);
    prune_path.poses.push_back(tmp_pose);

    int i = prune_index_;
    ROS_INFO("Current prune index: %d, Pose: (%f, %f)", prune_index_,
             robot_pose.pose.position.x, robot_pose.pose.position.y);
    while (i < global_path_.poses.size() && i - prune_index_ < 20) {

      TransformPose(global2path_transform_, global_path_.poses[i], tmp_pose);
      prune_path.poses.push_back(tmp_pose);
      i++;
    }
    // for (int i = prune_index_; i < global_path_.poses.size(); i++){
    //     TransformPose(global2path_transform_, global_path_.poses[i],
    //     tmp_pose); prune_path.poses.push_back(tmp_pose);

    // }

    // 6. Generate the cubic spline trajectory from above prune path
    GenTraj(prune_path, local_path);
    local_path_pub_.publish(local_path);

    // 7. Follow the trajectory and calculate the velocity
    geometry_msgs::Twist cmd_vel;
    FollowTraj(robot_pose, local_path, cmd_vel);
    cmd_vel_pub_.publish(cmd_vel);

    // 8. Publish the current segment index
    std_msgs::Int32 segment_index;
    segment_index.data = segments_[prune_index_];
    segment_pub_.publish(segment_index);

    auto plan_time = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - begin);
    ROS_INFO("Planning takes %f ms and passed %d/%d.",
             plan_time.count() / 1000., prune_index_,
             static_cast<int>(global_path_.poses.size()));
  } else {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    cmd_vel.linear.z = 0; // bool success or not
    cmd_vel_pub_.publish(cmd_vel);
  }
}

void RobotCtrl::FindNearstPose(geometry_msgs::PoseStamped &robot_pose,
                               nav_msgs::Path &path, int &prune_index,
                               double prune_ahead_dist) {
  double min_dist = std::numeric_limits<double>::max(); // 用于找到最小距离
  int closest_index = prune_index; // 从当前prune_index开始检查

  // 遍历路径点，找到最近的点
  for (int i = prune_index; i < path.poses.size(); ++i) {
    double current_dist = GetEuclideanDistance(robot_pose, path.poses[i]);
    if (current_dist < min_dist) {
      min_dist = current_dist;
      closest_index = i;
    }
  }

  // 更新prune_index为最近点的索引，只有当找到的点确实更近时才更新
  if (closest_index > prune_index) {
    prune_index = closest_index;
  }

  ROS_INFO("Updated prune index to: %d", prune_index);
}

void RobotCtrl::FollowTraj(const geometry_msgs::PoseStamped &robot_pose,
                           const nav_msgs::Path &traj,
                           geometry_msgs::Twist &cmd_vel) {

  geometry_msgs::PoseStamped robot_pose_1;
  GetGlobalRobotPose(tf_listener_, global_path_.header.frame_id, robot_pose_1);
  yaw_ = tf::getYaw(robot_pose_1.pose.orientation);

  double target_angle =
      atan2((traj.poses[1].pose.position.y - robot_pose.pose.position.y),
            (traj.poses[1].pose.position.x - robot_pose.pose.position.x));

  double diff_yaw = target_angle - yaw_;
  if (diff_yaw > M_PI) {
    diff_yaw -= 2 * M_PI;
  } else if (diff_yaw < -M_PI) {
    diff_yaw += 2 * M_PI;
  }

  double diff_distance = GetEuclideanDistance(robot_pose, traj.poses[1]);

  integral_error_distance_ += diff_distance;
  double derivative_error_distance = diff_distance - last_error_distance_;
  last_error_distance_ = diff_distance;

  integral_error_yaw_ += diff_yaw;
  double derivative_error_yaw = diff_yaw - last_error_yaw_;
  last_error_yaw_ = diff_yaw;

  double vx = max_x_speed_ * (p_x_value_ * diff_distance +
                              i_x_value_ * integral_error_distance_ +
                              d_x_value_ * derivative_error_distance);
  double omega = set_yaw_speed_ *
                 (p_yaw_value_ * diff_yaw + i_yaw_value_ * integral_error_yaw_ +
                  d_yaw_value_ * derivative_error_yaw);

  cmd_vel.linear.x = vx;
  cmd_vel.angular.z = omega;
  cmd_vel.linear.y = 0;
  std::cout << "Set velocities: vx=" << vx << " omega=" << omega << std::endl;
}

void RobotCtrl::GlobalPathCallback(const nav_msgs::PathConstPtr &msg) {
  if (!msg->poses.empty()) {
    global_path_ = *msg;
    prune_index_ = 0;
    plan_ = true;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pid_position_follow");
  RobotCtrl robotctrl;
  ros::spin();
  return 0;
}
