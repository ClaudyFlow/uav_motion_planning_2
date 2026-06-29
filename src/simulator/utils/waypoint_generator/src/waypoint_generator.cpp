#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
// ROS1: #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

#include <boost/format.hpp>
#include <deque>
#include <eigen3/Eigen/Eigen>

#include "sample_waypoints.h"

using namespace std;
using bfmt = boost::format;

// ROS1: ros::Publisher pub1;
rclcpp::Publisher pub1;
// ROS1: ros::Publisher pub2;
rclcpp::Publisher pub2;
// ROS1: ros::Publisher pub3;
rclcpp::Publisher pub3;

string waypoint_type;
bool is_odom_ready;
nav_msgs::Odometry odom;
nav_msgs::Path waypoints;

// series waypoint needed
std::deque<nav_msgs::Path> waypointSegments;
ros::Time trigged_time;

// ROS1: void load_seg(ros::NodeHandle& nh, int segid, const ros::Time& time_base) {
void load_seg(rclcpp::Node::SharedPtr nh, int segid, const ros::Time& time_base) {
  std::string seg_str = boost::str(bfmt("seg%d/") % segid);
  double yaw;
  double time_of_start = 0.0;
  ROS_INFO("Getting segment %d", segid);
  ROS_ASSERT(nh.getParam(seg_str + "yaw", yaw));
  ROS_ASSERT_MSG((yaw > -3.1499999) && (yaw < 3.14999999), "yaw=%.3f", yaw);
  ROS_ASSERT(nh.getParam(seg_str + "time_of_start", time_of_start));
  ROS_ASSERT(time_of_start >= 0.0);

  std::vector<double> ptx;
  std::vector<double> pty;
  std::vector<double> ptz;

  ROS_ASSERT(nh.getParam(seg_str + "x", ptx));
  ROS_ASSERT(nh.getParam(seg_str + "y", pty));
  ROS_ASSERT(nh.getParam(seg_str + "z", ptz));

  ROS_ASSERT(ptx.size());
  ROS_ASSERT(ptx.size() == pty.size() && ptx.size() == ptz.size());

  nav_msgs::Path path_msg;

  path_msg.header.stamp = time_base + ros::Duration(time_of_start);

// ROS1:   double baseyaw = tf::getYaw(odom.pose.pose.orientation);
  double baseyaw = tf::getYaw(odom.pose.pose.orientation);

  for (size_t k = 0; k < ptx.size(); ++k) {
    geometry_msgs::PoseStamped pt;
// ROS1:     pt.pose.orientation = tf::createQuaternionMsgFromYaw(baseyaw + yaw);
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(baseyaw + yaw);
    Eigen::Vector2d dp(ptx.at(k), pty.at(k));
    Eigen::Vector2d rdp;
    rdp.x() =
        std::cos(-baseyaw - yaw) * dp.x() + std::sin(-baseyaw - yaw) * dp.y();
    rdp.y() =
        -std::sin(-baseyaw - yaw) * dp.x() + std::cos(-baseyaw - yaw) * dp.y();
    pt.pose.position.x = rdp.x() + odom.pose.pose.position.x;
    pt.pose.position.y = rdp.y() + odom.pose.pose.position.y;
    pt.pose.position.z = ptz.at(k) + odom.pose.pose.position.z;
    path_msg.poses.push_back(pt);
  }

  waypointSegments.push_back(path_msg);
}

// ROS1: void load_waypoints(ros::NodeHandle& nh, const ros::Time& time_base) {
void load_waypoints(rclcpp::Node::SharedPtr nh, const ros::Time& time_base) {
  int seg_cnt = 0;
  waypointSegments.clear();
  ROS_ASSERT(nh.getParam("segment_cnt", seg_cnt));
  for (int i = 0; i < seg_cnt; ++i) {
    load_seg(nh, i, time_base);
    if (i > 0) {
      ROS_ASSERT(waypointSegments[i - 1].header.stamp <
                 waypointSegments[i].header.stamp);
    }
  }
  ROS_INFO("Overall load %zu segments", waypointSegments.size());
}

void publish_waypoints() {
  waypoints.header.frame_id = std::string("world");
// ROS1:   waypoints.header.stamp = ros::Time::now();
  waypoints.header.stamp = rclcpp::Clock().now();
  pub1.publish(waypoints);
  geometry_msgs::PoseStamped init_pose;
  init_pose.header = odom.header;
  init_pose.pose = odom.pose.pose;
  waypoints.poses.insert(waypoints.poses.begin(), init_pose);
  // pub2.publish(waypoints);
  waypoints.poses.clear();
}

void publish_waypoints_vis() {
  nav_msgs::Path wp_vis = waypoints;
  geometry_msgs::PoseArray poseArray;
  poseArray.header.frame_id = std::string("world");
// ROS1:   poseArray.header.stamp = ros::Time::now();
  poseArray.header.stamp = rclcpp::Clock().now();

  {
    geometry_msgs::Pose init_pose;
    init_pose = odom.pose.pose;
    poseArray.poses.push_back(init_pose);
  }

  for (auto it = waypoints.poses.begin(); it != waypoints.poses.end(); ++it) {
    geometry_msgs::Pose p;
    p = it->pose;
    poseArray.poses.push_back(p);
  }
  pub2.publish(poseArray);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  is_odom_ready = true;
  odom = *msg;

  if (waypointSegments.size()) {
    ros::Time expected_time = waypointSegments.front().header.stamp;
    if (odom.header.stamp >= expected_time) {
      waypoints = waypointSegments.front();

      std::stringstream ss;
      ss << bfmt("Series send %.3f from start:\n") % trigged_time.toSec();
      for (auto& pose_stamped : waypoints.poses) {
        ss << bfmt("P[%.2f, %.2f, %.2f] q(%.2f,%.2f,%.2f,%.2f)") %
                  pose_stamped.pose.position.x % pose_stamped.pose.position.y %
                  pose_stamped.pose.position.z %
                  pose_stamped.pose.orientation.w %
                  pose_stamped.pose.orientation.x %
                  pose_stamped.pose.orientation.y %
                  pose_stamped.pose.orientation.z
           << std::endl;
      }
      ROS_INFO_STREAM(ss.str());

      publish_waypoints_vis();
      publish_waypoints();

      waypointSegments.pop_front();
    }
  }
}

void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  /*    if (!is_odom_ready) {
          ROS_ERROR("[waypoint_generator] No odom!");
          return;
      }*/

// ROS1:   trigged_time = ros::Time::now();  // odom.header.stamp;
  trigged_time = rclcpp::Clock().now();  // odom.header.stamp;
  // ROS_ASSERT(trigged_time > ros::Time(0));

  ros::NodeHandle n("~");
// ROS1:   n.param("waypoint_type", waypoint_type, string("manual"));
n->declare_parameter<waypoint_type>("waypoint_type", string("manual");
n->get_parameter("waypoint_type", waypoint_type);

  if (waypoint_type == string("circle")) {
    waypoints = circle();
    publish_waypoints_vis();
    publish_waypoints();
  } else if (waypoint_type == string("eight")) {
    waypoints = eight();
    publish_waypoints_vis();
    publish_waypoints();
  } else if (waypoint_type == string("points")) {
    waypoints = point();
    publish_waypoints_vis();
    publish_waypoints();
  } else if (waypoint_type == string("series")) {
    load_waypoints(n, trigged_time);
  } else if (waypoint_type == string("manual-lonely-waypoint")) {
    if (msg->pose.position.z > -0.1) {
      // if height > 0, it's a valid goal;
      geometry_msgs::PoseStamped pt = *msg;
      waypoints.poses.clear();
      waypoints.poses.push_back(pt);
      publish_waypoints_vis();
      publish_waypoints();
    } else {
      ROS_WARN(
          "[waypoint_generator] invalid goal in manual-lonely-waypoint mode.");
    }
  } else {
    if (msg->pose.position.z > 0) {
      // if height > 0, it's a normal goal;
      geometry_msgs::PoseStamped pt = *msg;
      if (waypoint_type == string("noyaw")) {
// ROS1:         double yaw = tf::getYaw(odom.pose.pose.orientation);
        double yaw = tf::getYaw(odom.pose.pose.orientation);
// ROS1:         pt.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        pt.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      }
      waypoints.poses.push_back(pt);
      publish_waypoints_vis();
    } else if (msg->pose.position.z > -1.0) {
      // if 0 > height > -1.0, remove last goal;
      if (waypoints.poses.size() >= 1) {
        waypoints.poses.erase(std::prev(waypoints.poses.end()));
      }
      publish_waypoints_vis();
    } else {
      // if -1.0 > height, end of input
      if (waypoints.poses.size() >= 1) {
        publish_waypoints_vis();
        publish_waypoints();
      }
    }
  }
}

void traj_start_trigger_callback(const geometry_msgs::PoseStamped& msg) {
  if (!is_odom_ready) {
    ROS_ERROR("[waypoint_generator] No odom!");
    return;
  }

  ROS_WARN("[waypoint_generator] Trigger!");
  trigged_time = odom.header.stamp;
  ROS_ASSERT(trigged_time > ros::Time(0));

  ros::NodeHandle n("~");
// ROS1:   n.param("waypoint_type", waypoint_type, string("manual"));
n->declare_parameter<waypoint_type>("waypoint_type", string("manual");
n->get_parameter("waypoint_type", waypoint_type);

  ROS_ERROR_STREAM("Pattern " << waypoint_type << " generated!");
  if (waypoint_type == string("free")) {
    waypoints = point();
    publish_waypoints_vis();
    publish_waypoints();
  } else if (waypoint_type == string("circle")) {
    waypoints = circle();
    publish_waypoints_vis();
    publish_waypoints();
  } else if (waypoint_type == string("eight")) {
    waypoints = eight();
    publish_waypoints_vis();
    publish_waypoints();
  } else if (waypoint_type == string("point")) {
    waypoints = point();
    publish_waypoints_vis();
    publish_waypoints();
  } else if (waypoint_type == string("series")) {
    load_waypoints(n, trigged_time);
  }
}

int main(int argc, char** argv) {
// ROS1:   ros::init(argc, argv, "waypoint_generator");
  rclcpp::init(argc, argv, "waypoint_generator");
  ros::NodeHandle n("~");

// ROS1:   n.param("waypoint_type", waypoint_type, string("manual"));
n->declare_parameter<waypoint_type>("waypoint_type", string("manual");
n->get_parameter("waypoint_type", waypoint_type);

// ROS1:   ros::Subscriber sub1 = n.subscribe("odom", 10, odom_callback);
  rclcpp::Subscriber sub1 = n.subscribe("odom", 10, odom_callback);
// ROS1:   ros::Subscriber sub2 = n.subscribe("goal", 10, goal_callback);
  rclcpp::Subscriber sub2 = n.subscribe("goal", 10, goal_callback);
// ROS1:   ros::Subscriber sub3 =
  rclcpp::Subscriber sub3 =
      n.subscribe("traj_start_trigger", 10, traj_start_trigger_callback);

  pub1 = n.advertise<nav_msgs::Path>("waypoints", 50);
  pub2 = n.advertise<geometry_msgs::PoseArray>("waypoints_vis", 10);

  trigged_time = ros::Time(0);

// ROS1:   ros::spin();
  rclcpp::spin(nh);

  return 0;
}