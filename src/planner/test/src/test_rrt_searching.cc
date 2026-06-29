// ROS1: #pragma region include
#include <rclcpp/rclcpp.hpp>
#pragma region include::project
#include "path_searching/rrt.hh"
#pragma endregion include::project
#pragma region include::third
#include <visualization_msgs/Marker.h>
#pragma endregion include::third
#pragma region include::standard

#pragma endregion include::standard
#pragma endregion include


path_searching::RRT::Ptr rrt_;

// ROS1: ros::Subscriber goal_sub;
rclcpp::Subscriber goal_sub;
// ROS1: ros::Subscriber odom_sub;
rclcpp::Subscriber odom_sub;

// ROS1: ros::Publisher path_pub;
rclcpp::Publisher path_pub;
// ROS1: ros::Publisher rrt_tree_vertices_pub;
rclcpp::Publisher rrt_tree_vertices_pub;
// ROS1: ros::Publisher rrt_tree_edges_pub;
rclcpp::Publisher rrt_tree_edges_pub;

nav_msgs::Odometry::ConstPtr odom_;
std::vector<Eigen::Vector3d> path;
std::vector<Eigen::Vector3d> vertices;
std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges;

void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom) { odom_ = odom; }

void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  Eigen::Vector3d end_pt(msg->pose.position.x, msg->pose.position.y,
                         msg->pose.position.z);
  Eigen::Vector3d start_pt(odom_->pose.pose.position.x,
                           odom_->pose.pose.position.y,
                           odom_->pose.pose.position.z);
  std::cout << "Start point: " << start_pt.transpose() << std::endl;
  std::cout << "End point: " << end_pt.transpose() << std::endl;
  int success = rrt_->search(start_pt, end_pt, path);

  if (success == 1) {
    // visualize searching-tree-vertices
    visualization_msgs::Marker vertices_marker;
    vertices_marker.header.frame_id = "world";
// ROS1:     vertices_marker.header.stamp = ros::Time::now();
    vertices_marker.header.stamp = rclcpp::Clock().now();

    vertices_marker.ns = "rrt/vertices";
    vertices_marker.id = 0;
    vertices_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    vertices_marker.action = visualization_msgs::Marker::ADD;

    vertices_marker.pose.orientation.w = 1.0;

    vertices_marker.scale.x = 0.1;
    vertices_marker.scale.y = 0.1;
    vertices_marker.scale.z = 0.1;

    vertices_marker.color.a = 1.0;
    vertices_marker.color.r = 0.0;
    vertices_marker.color.g = 1.0;
    vertices_marker.color.b = 0.0;

    rrt_->getWholeTree(vertices, edges);

    // for (int i = 0; i < vertices.size(); i++)
    // {
    //   geometry_msgs::Point pt;
    //   pt.x = vertices[i][0];
    //   pt.y = vertices[i][1];
    //   pt.z = vertices[i][2];
    //   vertices_marker.points.push_back(pt);
    // }

    // rrt_tree_vertices_pub.publish(vertices_marker);

    // visualize searching-tree-edges
    visualization_msgs::Marker edges_marker;
    edges_marker.header.frame_id = "world";
// ROS1:     edges_marker.header.stamp = ros::Time::now();
    edges_marker.header.stamp = rclcpp::Clock().now();

    edges_marker.ns = "rrt/edges";
    edges_marker.id = 0;
    edges_marker.type = visualization_msgs::Marker::LINE_LIST;
    edges_marker.action = visualization_msgs::Marker::ADD;

    edges_marker.pose.orientation.w = 1.0;

    edges_marker.scale.x = 0.05;
    edges_marker.scale.y = 0.05;
    edges_marker.scale.z = 0.05;

    edges_marker.color.a = 1.0;
    edges_marker.color.r = 0.0;
    edges_marker.color.g = 1.0;
    edges_marker.color.b = 0.0;

    for (int i = 0; i < edges.size(); i++) {
      geometry_msgs::Point start_pt, end_pt;
      start_pt.x = edges[i].first[0];
      start_pt.y = edges[i].first[1];
      start_pt.z = edges[i].first[2];
      end_pt.x = edges[i].second[0];
      end_pt.y = edges[i].second[1];
      end_pt.z = edges[i].second[2];
      edges_marker.points.push_back(start_pt);
      edges_marker.points.push_back(end_pt);
    }

    rrt_tree_edges_pub.publish(edges_marker);

    // visualize path
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "world";
// ROS1:     path_marker.header.stamp = ros::Time::now();
    path_marker.header.stamp = rclcpp::Clock().now();

    path_marker.ns = "rrt/path";
    path_marker.id = 0;

    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;

    path_marker.pose.orientation.w = 1.0;

    path_marker.scale.x = 0.2;
    path_marker.scale.y = 0.2;
    path_marker.scale.z = 0.2;

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
  vertices.clear();
  edges.clear();

  rrt_->reset();
}

int main(int argc, char** argv) {
// ROS1:   ros::init(argc, argv, "test_rrt_searching");
  rclcpp::init(argc, argv, "test_rrt_searching");
// ROS1:   ros::NodeHandle nh("~");
  rclcpp::Node::SharedPtr nh("~");

// ROS1:   ros::Subscriber goal_sub =
  rclcpp::Subscriber goal_sub =
      nh.subscribe<geometry_msgs::PoseStamped>("/goal", 10, &GoalCallback);
// ROS1:   ros::Subscriber odom_sub =
  rclcpp::Subscriber odom_sub =
      nh.subscribe<nav_msgs::Odometry>("/visual_slam/odom", 10, &OdomCallback);

  path_pub = nh.advertise<visualization_msgs::Marker>("path", 10);
  rrt_tree_vertices_pub =
      nh.advertise<visualization_msgs::Marker>("rrt_tree_vertices", 10);
  rrt_tree_edges_pub =
      nh.advertise<visualization_msgs::Marker>("rrt_tree_edges", 10);

  GridMap::Ptr grid_map = std::make_shared<GridMap>();
  grid_map->initMap(nh);

  rrt_ = std::make_shared<path_searching::RRT>();

  rrt_->setParam(nh);
  rrt_->setGridMap(grid_map);
  rrt_->init();

// ROS1:   ros::spin();
  rclcpp::spin(nh);
  return 0;
}