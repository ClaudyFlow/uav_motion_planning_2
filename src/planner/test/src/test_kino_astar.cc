// ROS1: #pragma region include
#include <rclcpp/rclcpp.hpp>
#pragma region include::project
#include "path_searching/kino_astar.hh"
#pragma endregion include::project
#pragma region include::third

#pragma endregion include::third
#pragma region include::standard

#pragma endregion include::standard
#pragma endregion include

int main(int argc, char** argv) {
// ROS1:   ros::init(argc, argv, "test_kino_astar_node");
  rclcpp::init(argc, argv, "test_kino_astar_node");
// ROS1:   ros::NodeHandle nh;
  rclcpp::Node::SharedPtr nh;

  GridMap::Ptr grid_map = std::make_shared<GridMap>();
  grid_map->initMap(nh);

  path_searching::KinoAStar::Ptr kino_astar_;
  kino_astar_ = std::make_shared<path_searching::KinoAStar>();

  kino_astar_->setParam(nh);
  cout << "KinoAStar parameters set" << endl;
  kino_astar_->setGridMap(grid_map);
  cout << "Grid map set" << endl;
  kino_astar_->init();
  cout << "KinoAStar initialized" << endl;
  Eigen::Vector3d start_pt(0, 0, 1.0);
  Eigen::Vector3d start_vel(1, 1, 0);
  Eigen::Vector3d end_pt(5, 5, 0.8);
  Eigen::Vector3d end_vel(0, 0, 0);
  std::vector<Eigen::Vector3d> path;
  kino_astar_->search(start_pt, start_vel, end_pt, end_vel, path);

  for (int i = 0; i < path.size(); i++) {
    std::cout << "Point " << i << ": " << path[i].transpose() << std::endl;
  };

// ROS1:   ros::spin();
  rclcpp::spin(nh);

  return 0;
}