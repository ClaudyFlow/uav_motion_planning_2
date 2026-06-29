// ROS1: #pragma region include
#include <rclcpp/rclcpp.hpp>
#pragma region include::project
#include "path_searching/astar.hh"
#pragma endregion include::project
#pragma region include::third

#pragma endregion include::third
#pragma region include::standard

#pragma endregion include::standard
#pragma endregion include


int main(int argc, char** argv) {
// ROS1:   ros::init(argc, argv, "test_astar_node");
  rclcpp::init(argc, argv, "test_astar_node");
// ROS1:   ros::NodeHandle nh;
  rclcpp::Node::SharedPtr nh;

  GridMap::Ptr grid_map = std::make_shared<GridMap>();
  grid_map->initMap(nh);

  std::shared_ptr<path_searching::AStar> astar_;
  astar_ = std::make_shared<path_searching::AStar>();

  astar_->setParam(nh);
  cout << "AStar parameters set" << endl;
  astar_->setGridMap(grid_map);
  cout << "Grid map set" << endl;
  astar_->init();
  cout << "AStar initialized" << endl;
  Eigen::Vector3d start_pt(0, 0, 0);
  Eigen::Vector3d end_pt(10, 10, 1);
  std::vector<Eigen::Vector3d> path;
  astar_->search(start_pt, end_pt, path);

  for (int i = 0; i < path.size(); i++) {
    std::cout << "Point " << i << ": " << path[i].transpose() << std::endl;
  };

// ROS1:   ros::spin();
  rclcpp::spin(nh);

  return 0;
}