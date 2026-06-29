// ROS1: #pragma region include
#include <rclcpp/rclcpp.hpp>
#pragma region include::project
#include "path_searching/astar.hh"
#pragma endregion include::project
#pragma region include::third
#include <visualization_msgs/Marker.h>
#pragma endregion include::third
#pragma region include::standard

#pragma endregion include::standard
#pragma endregion include

std::shared_ptr<path_searching::AStar> astar_;

// ROS1: ros::Subscriber goal_sub;
rclcpp::Subscriber goal_sub;
// ROS1: ros::Subscriber odom_sub;
rclcpp::Subscriber odom_sub;

// ROS1: ros::Publisher path_pub;
rclcpp::Publisher path_pub;

nav_msgs::Odometry::ConstPtr odom_;
std::vector<Eigen::Vector3d> path;

void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom) { odom_ = odom; }

void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  Eigen::Vector3d end_pt(msg->pose.position.x, msg->pose.position.y,
                         msg->pose.position.z);
  Eigen::Vector3d start_pt(odom_->pose.pose.position.x,
                           odom_->pose.pose.position.y,
                           odom_->pose.pose.position.z);
  std::cout << "Start point: " << start_pt.transpose() << std::endl;
  std::cout << "End point: " << end_pt.transpose() << std::endl;
  int success = astar_->search(start_pt, end_pt, path);

  if (success == 1) {
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "world";
// ROS1:     path_marker.header.stamp = ros::Time::now();
    path_marker.header.stamp = rclcpp::Clock().now();

    path_marker.ns = "astar/path";
    path_marker.id = 0;

    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;

    path_marker.pose.orientation.w = 1.0;

    path_marker.scale.x = 0.1;
    path_marker.scale.y = 0.1;
    path_marker.scale.z = 0.1;

    path_marker.color.a = 1.0;
    path_marker.color.r = 1.0;
    path_marker.color.g = 0.0;
    path_marker.color.b = 0.0;

    for (int i = 0; i < path.size(); i++) {
      geometry_msgs::Point pt;
      pt.x = path[i][0];
      pt.y = path[i][1];
      pt.z = path[i][2];
      path_marker.points.push_back(pt);
    }

    path_pub.publish(path_marker);
  } else {
    std::cout << "Path not found!" << std::endl;
  }
  path.clear();
  astar_->reset();
}

int main(int argc, char** argv) {
// ROS1:   ros::init(argc, argv, "test_astar_searching");
  rclcpp::init(argc, argv, "test_astar_searching");
// ROS1:   ros::NodeHandle nh("~");
  rclcpp::Node::SharedPtr nh("~");

// ROS1:   ros::Subscriber goal_sub =
  rclcpp::Subscriber goal_sub =
      nh.subscribe<geometry_msgs::PoseStamped>("/goal", 10, &GoalCallback);
// ROS1:   ros::Subscriber odom_sub =
  rclcpp::Subscriber odom_sub =
      nh.subscribe<nav_msgs::Odometry>("/visual_slam/odom", 10, &OdomCallback);

  path_pub = nh.advertise<visualization_msgs::Marker>("path", 10);

  GridMap::Ptr grid_map = std::make_shared<GridMap>();
  grid_map->initMap(nh);

  astar_ = std::make_shared<path_searching::AStar>();

  astar_->setParam(nh);
  astar_->setGridMap(grid_map);
  astar_->init();

// ROS1:   ros::spin();
  rclcpp::spin(nh);

  return 0;
}