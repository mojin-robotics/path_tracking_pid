#include <path_tracking_pid/path_tracking_pid_local_planner.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <amr_road_network_msgs/SegmentPath.h>
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>

inline double translation_distance(const geometry_msgs::PoseStamped a, const geometry_msgs::PoseStamped b)
{
  assert(a.header.frame_id == b.header.frame_id);
  return hypot(b.pose.position.x - a.pose.position.x, b.pose.position.y - a.pose.position.y);
}

inline double calculate_yaw(geometry_msgs::Quaternion quaternion)
{
  // calculate theta
  double roll, pitch, yaw;
  tf::Quaternion q = tf::Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

inline double rotation_distance(const geometry_msgs::PoseStamped a, const geometry_msgs::PoseStamped b)
{
  assert(a.header.frame_id == b.header.frame_id);
  return calculate_yaw(b.pose.orientation) - calculate_yaw(a.pose.orientation);
}

uint closestIndex(geometry_msgs::PoseStamped global_pose_msg, std::vector<geometry_msgs::PoseStamped> global_plan_)
{
  // Step 1: Find the (index of the) waypoint closest to our current pose (global_pose_msg).
  //  Stop searching further when the distance starts increasing

  size_t start_index = 0;

  // find waypoint closest to robot position
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < global_plan_.size(); i++)
  {
    auto new_trans_dist = translation_distance(global_pose_msg, global_plan_[i]);
    auto new_rot_dist = rotation_distance(global_pose_msg, global_plan_[i]);
    ROS_DEBUG_STREAM("extractWaypoints: i=" << i << ", new_trans_distance = " << new_trans_dist << ", new_rot_distance = " << new_rot_dist << ", min_dist = " << min_dist);
    if (new_trans_dist <= min_dist)
    {
      if (abs(new_rot_dist) < 0.1) // goal_tolerance_rotation_ was a variable
      {
        start_index = i;
        min_dist = new_trans_dist;
      }
      else
      {
        ROS_DEBUG_STREAM("Point " << i << " at x=" << global_plan_[i].pose.position.x << ", y=" << global_plan_[i].pose.position.y << " is close in translation but has a different orientation (dist:" << new_rot_dist << ") so not actually close");
      }
    }
    else
    {
      ROS_DEBUG_STREAM("Path poses are getting further away, so closest has been found");
      break; // Distance is getting bigger again, stop searching
    }
  }
  return start_index;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "path_tracker");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ROS_INFO("Hello Path Tracker");

  auto tplp = new path_tracking_pid::TrackingPidLocalPlanner();

  // Call computeVelocities at some configurable rate?

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  costmap_2d::Costmap2DROS costmap("empty_costmap", tfBuffer);

  std::string map_frame_;
  std::string base_link_frame_;

  nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
  map_frame_ = costmap.getGlobalFrameID(); //TODO: Not sure if this works if we init the costmap like this
  ROS_INFO_STREAM("base_link_frame=" << base_link_frame_ << ", map_frame=" << map_frame_);

  bool has_plan = false;

  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);

  auto receivePath = [&](const nav_msgs::PathConstPtr &path)
  {
    if (path->poses.size())
    {
      ROS_INFO_STREAM("Setting path of length " << path->poses.size());
      has_plan = tplp->setPlan(path->poses);
    }
    else
    {
      ROS_WARN_STREAM("Ignoring path of length, cancelling current" << path->poses.size());
      tplp->cancel();
    }
  };
  auto path_sub = nh.subscribe<nav_msgs::Path>("path", 1, receivePath);
  auto sliced_path_pub = pnh.advertise<nav_msgs::Path>("sliced_path", 1);

  auto receiveSegmentPath = [&](const amr_road_network_msgs::SegmentPathConstPtr &path)
  {
    ROS_INFO_STREAM("Received SegmentPath");
    if (path->waypoints.poses.size())
    {
      ROS_INFO_STREAM("Received SegmentPath of length " << path->waypoints.poses.size() << " with target at index " << path->target_index);
      // Starting and Ending iterators
      int skip = 15;
      auto start = path->waypoints.poses.begin(); // TODO: offset to skip the poses behind the robot.


      geometry_msgs::PoseStamped current_pose;
      if (!costmap.getRobotPose(current_pose))
      {
        ROS_WARN("Could not retrieve up to date robot pose from costmap!");
        return;
      }
      int start_index = closestIndex(current_pose, path->waypoints.poses);

      if(path->target_index > skip)
      {
        start = start + skip;
        start_index = start_index + skip;
      }
      auto end = path->waypoints.poses.begin() + path->target_index + 1;

      std::vector<geometry_msgs::PoseStamped> sliced_path(end - start);
      copy(start, end, sliced_path.begin());
      nav_msgs::Path sliced_nav_path;
      sliced_nav_path.header = path->waypoints.header;
      sliced_nav_path.poses = sliced_path;
      sliced_path_pub.publish(sliced_nav_path); 

      if (sliced_path.size())
      {
        ROS_INFO_STREAM("Setting sliced plan of length " << sliced_path.size() << " starting at " << start_index);
        has_plan = tplp->setPlan(sliced_path);
      }
      else
      {
        ROS_WARN_STREAM("Ignoring sliced SegmentPath of length " << sliced_path.size());
        // if (has_plan)
        // {
        //   tplp->cancel();
        // }
      }
    }
    else
    {
      ROS_WARN_STREAM("Ignoring SegmentPath of length " << path->waypoints.poses.size());
      // if(has_plan)
      // {
      //   tplp->cancel();
      // }
    }
  };
  auto segment_sub = nh.subscribe<amr_road_network_msgs::SegmentPath>("segment_path", 1, receiveSegmentPath);

  tplp->initialize("path_tracker", &tfBuffer, &costmap);

  auto timerCallback = [&](const ros::TimerEvent &)
  {
    bool goal_reached = tplp->isGoalReached(0.0, 0.0); // The tolerances are ignored by implementation
    if (has_plan && !goal_reached)
    {
      geometry_msgs::TwistStamped cmd_vel;
      geometry_msgs::PoseStamped current_pose;
      geometry_msgs::TwistStamped velocity;
      std::string message;

      tplp->computeVelocityCommands(current_pose, velocity, cmd_vel, message); // The current_pose and velocty are ignored by implementation

      ROS_INFO_COND(!message.empty(), message.c_str());

      cmd_vel_pub.publish(cmd_vel.twist);
    }
    else
    {
      ROS_DEBUG_STREAM("Nothing to do: we have " << has_plan << " plan and goal reached = " << goal_reached);
    }

    if(has_plan && goal_reached)
    {
      has_plan = false;
      ROS_INFO("Goal reached");
    }
  };
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);

  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}