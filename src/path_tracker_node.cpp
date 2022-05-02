#include <path_tracking_pid/path_tracking_pid_local_planner.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <amr_road_network_msgs/SegmentPath.h>
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>


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
      if(path->target_index > skip)
      {
        start = start + skip;
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
        has_plan = tplp->setPlan(sliced_path);
      }
      else
      {
        ROS_WARN_STREAM("Ignoring sliced SegmentPath of length " << sliced_path.size());
        if (has_plan)
        {
          tplp->cancel();
        }
      }
    }
    else
    {
      ROS_WARN_STREAM("Ignoring SegmentPath of length " << path->waypoints.poses.size());
      if(has_plan)
      {
        tplp->cancel();
      }
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