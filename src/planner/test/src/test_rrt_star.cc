#pragma region include
#pragma region include::project
#include "path_searching/rrt_star.hh"
#pragma endregion include::project
#pragma region include::third

#pragma endregion include::third
#pragma region include::standard

#pragma endregion include::standard
#pragma endregion include

int main(int argc, char** argv) {
// ROS1:   ros::init(argc, argv, "test_rrt_star_node");
  rclcpp::init(argc, argv, "test_rrt_star_node");
// ROS1:   ros::NodeHandle nh;
  rclcpp::Node::SharedPtr nh;

  GridMap::Ptr grid_map = std::make_shared<GridMap>();
  grid_map->initMap(nh);

  path_searching::RRTStar::Ptr rrt_star_;
  rrt_star_ = std::make_shared<path_searching::RRTStar>();

  rrt_star_->setParam(nh);
  cout << "RRTStar parameters set" << endl;
  rrt_star_->setGridMap(grid_map);
  cout << "Grid map set" << endl;
  rrt_star_->init();
  cout << "RRTStar initialized" << endl;
  Eigen::Vector3d start_pt(0, 0, 0.1);
  Eigen::Vector3d end_pt(10, 10, 1);
  std::vector<Eigen::Vector3d> path;
  rrt_star_->search(start_pt, end_pt, path);

  for (int i = 0; i < path.size(); i++) {
    std::cout << "Point " << i << ": " << path[i].transpose() << std::endl;
  };

// ROS1:   ros::spin();
  rclcpp::spin(nh);
  return 0;
}