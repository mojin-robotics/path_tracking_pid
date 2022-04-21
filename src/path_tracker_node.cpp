#include <path_tracking_pid/path_tracking_pid_local_planner.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "path_tracker");
  ROS_INFO("Hello Path Tracker");

  auto tplp = new path_tracking_pid::TrackingPidLocalPlanner();

  // Call initialize with empty costmap that we don't update
  // Subscribe to Paths, convert those to vector of PoseStamped for setPlan
  // Call computeVelocities at some configurable rate?

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  std::vector<geometry_msgs::PoseStamped> plan = std::vector<geometry_msgs::PoseStamped>();
  tplp->setPlan(plan);
  // tplp->cancel();

  ros::spin();

  ROS_INFO("Bye-bye Path Tracker");

  return 0;
}