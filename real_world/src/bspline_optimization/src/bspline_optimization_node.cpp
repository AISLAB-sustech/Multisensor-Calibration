// // optimizer_node.cpp
#include <bspline_optimization_node.hpp>
#include <fstream>

// #include "optimizer.h"
namespace bspline_ekf {
BSplineOptimizationNode::BSplineOptimizationNode() : nh_("~") {
  optimizer_ = new BSplineOptimization();
  odom_sub_ =
      nh_.subscribe("/odom", 1, &BSplineOptimizationNode::odomCallback, this);
  segment_sub_ = nh_.subscribe("/segmented_path", 1,
                               &BSplineOptimizationNode::segmentCallback, this);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/optimized_path", 200);
  full_path_pub_ = nh_.advertise<nav_msgs::Path>("/full_path", 700);
  segments_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/segments", 1);
  ext_para_ = {0.3, 0.2, 15.0 / 180 * 3.1415, 0.5, 0.3, 50.0 / 180.0 * 3.1415,
               1.2, 2.4};
  control_points_ = {0, 0, 0.2, 0.2, 0.4, 0.4, 0.6, 0.6, 0.8, 0.8,
                     1, 1, 1.2, 1.2, 1.4, 1.4, 1.6, 1.6, 1.8, 1.8};
  current_segment_ = 1;
  ROS_INFO("Optimizing segment %d", current_segment_);
  vector<double> result_points;
  optimizer_->optimize(0.01, ext_para_, current_segment_, control_points_,
                       result_points, path_);
  control_points_ = result_points;
  timer_ = nh_.createTimer(ros::Duration(0.1),
                           &BSplineOptimizationNode::timerCallback, this);
  ROS_INFO("Optimization done");
  ROS_INFO("Optimizer node initialized");
}

void BSplineOptimizationNode::segmentCallback(const std_msgs::Int32 &msg) {
  if (current_segment_ < msg.data) {
    // Update the current segment
    current_segment_ = msg.data;
    ROS_INFO("Optimizing segment %d", current_segment_);
    vector<double> result_points;
    optimizer_->optimize(0.01, ext_para_, current_segment_, control_points_,
                         result_points, path_);
    control_points_ = result_points;
    ROS_INFO("Optimization done");
  }
}

void BSplineOptimizationNode::odomCallback(const nav_msgs::Odometry &msg) {
  odom_count_++;
  if (odom_count_ % 3 != 0) {
    return;
  }
  std_msgs::Int32MultiArray segments;
  for (auto &point : path_) {
    if (point.seg == current_segment_ || point.seg == current_segment_ + 1) {
      segments.data.push_back(point.seg);
    }
  }

  nav_msgs::Path path;
  nav_msgs::Path full_path;
  int count = 0;
  for (auto &point : path_) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = point.point(0);
    pose.pose.position.y = point.point(1);
    pose.pose.position.z = 0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(point.yaw_angle);
    if (point.seg == current_segment_ || point.seg == current_segment_ + 1) {
      if (count < 10 && current_segment_ == 1) {
        count++;
      } else {
        path.poses.push_back(pose);
      }
    }
    full_path.poses.push_back(pose);
  }

  path.header.frame_id = "odom";
  path.header.stamp = ros::Time::now();
  full_path.header.frame_id = "odom";
  full_path.header.stamp = ros::Time::now();
  segments_pub_.publish(segments);
  path_pub_.publish(path);
  full_path_pub_.publish(full_path);
}

void BSplineOptimizationNode::timerCallback(const ros::TimerEvent &event) {
  updateTransform("base_link", "microphone_link", transform_microphone_);
  updateTransform("base_link", "lidar_link", transform_lidar_);
  // updateTransform("map", "microphone_src", transform_microphone_src_);
  ext_para_.M_x = transform_microphone_.getOrigin().x();
  ext_para_.M_y = transform_microphone_.getOrigin().y();
  ext_para_.M_theta = tf::getYaw(transform_microphone_.getRotation());
  ext_para_.L_x = transform_lidar_.getOrigin().x();
  ext_para_.L_y = transform_lidar_.getOrigin().y();
  ext_para_.L_theta = tf::getYaw(transform_lidar_.getRotation());
  // ext_para_.SRC_x = transform_microphone_src_.getOrigin().x();
  // ext_para_.SRC_y = transform_microphone_src_.getOrigin().y();
  ROS_INFO("[BSplineOptimizationNode]M_x: %f, M_y: %f, M_theta: %f",
           ext_para_.M_x, ext_para_.M_y, ext_para_.M_theta);
  ROS_INFO("[BSplineOptimizationNode]L_x: %f, L_y: %f, L_theta: %f",
           ext_para_.L_x, ext_para_.L_y, ext_para_.L_theta);
}

void BSplineOptimizationNode::updateTransform(
    const std::string &target_frame, const std::string &source_frame,
    tf::StampedTransform &transform_) {
  try {
    listener_.waitForTransform(target_frame, source_frame, ros::Time(0),
                               ros::Duration(3.0));
    listener_.lookupTransform(target_frame, source_frame, ros::Time(0),
                              transform_);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}
} // namespace bspline_ekf

int main(int argc, char **argv) {
  ros::init(argc, argv, "optimizer_node");
  bspline_ekf::BSplineOptimizationNode optimization_node;
  ros::spin();
  return 0;
}