#include <bspline_optimization.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace bspline_ekf {
class BSplineOptimizationNode {
public:
  BSplineOptimizationNode();
  ~BSplineOptimizationNode() = default;

private:
  // ros
  ros::NodeHandle nh_;
  BSplineOptimization *optimizer_;
  ros::Subscriber segment_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher path_pub_;
  ros::Publisher full_path_pub_;
  ros::Publisher segments_pub_;
  tf::TransformListener listener_;
  tf::StampedTransform transform_microphone_;
  tf::StampedTransform transform_lidar_;
  tf::StampedTransform transform_microphone_src_;
  ros::Timer timer_;
  // variables
  int current_segment_;
  int odom_count_;
  vector<SegmentedPathPoint> path_;
  vector<double> control_points_;
  ExtPara ext_para_;
  // methods
  void segmentCallback(const std_msgs::Int32 &msg);
  void updateTransform(const std::string &target_frame,
                       const std::string &source_frame,
                       tf::StampedTransform &transform_);
  void timerCallback(const ros::TimerEvent &event);
  void odomCallback(const nav_msgs::Odometry &msg);
};
} // namespace bspline_ekf