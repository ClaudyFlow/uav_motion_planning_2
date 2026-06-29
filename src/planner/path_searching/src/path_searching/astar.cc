// ROS1: #pragma region include
#include <rclcpp/rclcpp.hpp>
#pragma region include_header
#include "path_searching/astar.hh"
#pragma endregion include_header
#pragma region include_project

#pragma endregion include_project
#pragma region include_third

#pragma endregion include_third
#pragma region include_standard

#pragma endregion include_standard
#pragma endregion include

namespace path_searching {

// ROS1: void AStar::setParam(ros::NodeHandle& nh) {
void AStar::setParam(rclcpp::Node::SharedPtr nh) {
// ROS1:   nh.param("astar/resolution", resolution_, 0.1);
nh->declare_parameter<resolution_>("astar/resolution", 0.1);
nh->get_parameter("astar/resolution", resolution_);
// ROS1:   nh.param("astar/lambda_heu", lambda_, 1.0);
nh->declare_parameter<lambda_>("astar/lambda_heu", 1.0);
nh->get_parameter("astar/lambda_heu", lambda_);
// ROS1:   nh.param("astar/allocated_node_num", allocated_node_num_, 100000);
nh->declare_parameter<allocated_node_num_>("astar/allocated_node_num", 100000);
nh->get_parameter("astar/allocated_node_num", allocated_node_num_);
}

void AStar::init() {
  this->inv_resolution_ = 1.0 / resolution_;
  this->tie_breaker_ = 1.0 + 1e-4;  // default value
  path_node_pool_.resize(
      allocated_node_num_);  // allocate memory for path_node_pool_
  for (int i = 0; i < allocated_node_num_; i++) {
    path_node_pool_[i] = new AStarNode();
  }

  if (!grid_map_) {
    std::cerr << "Error: grid_map_ is not set." << std::endl;
    return;
  }

  use_node_num_ = 0;
  grid_map_->getRegion(origin_, map_size_);
  resolution_ = grid_map_->getResolution();

  std::cout << "origin: " << origin_.transpose() << std::endl;
  std::cout << "map_size: " << map_size_.transpose() << std::endl;
  std::cout << "resolution: " << resolution_ << std::endl;
  std::cout << "lambda_heu: " << lambda_ << std::endl;
  std::cout << "allocated_node_num: " << allocated_node_num_ << std::endl;
}

int AStar::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt,
                  std::vector<Eigen::Vector3d>& path) {
  // if end_pt is out of map, return NO_PATH_FOUND
// ROS1:   ros::Time start_time = ros::Time::now();
  ros::Time start_time = rclcpp::Clock().now();
  if (grid_map_->isInMap(end_pt) == false) {
    std::cerr << "Error: end_pt is out of map." << std::endl;
    return NO_PATH_FOUND;
  }
  // push start_node into open_list
  AStarNode* start_node = path_node_pool_[use_node_num_];  // bug fixed
  start_node->g_cost = 0.0;
  start_node->position = start_pt;
  start_node->parent = nullptr;
  start_node->f_cost = lambda_ * getDiagonalHeu(start_pt, end_pt);
  start_node->node_state = AStarNode::NodeStateEnum::IN_OPEN_LIST;

  open_list_.push(start_node);
  expanded_nodes_.insert(start_node->position, start_node);
  use_node_num_ += 1;

  while (!open_list_.empty()) {
    // pop node from open_list with lowest f_cost
    // and add it into close_list
    AStarNode* current_node = open_list_.top();
    open_list_.pop();
    current_node->node_state = AStarNode::NodeStateEnum::IN_CLOSE_LIST;
    close_list_.insert(current_node->position, current_node);

    // if current_node is end_pt, retrieve path and return success
    if (abs(current_node->position(0) - end_pt(0)) < resolution_ &&
        abs(current_node->position(1) - end_pt(1)) < resolution_ &&
        abs(current_node->position(2) - end_pt(2)) < resolution_) {
// ROS1:       ros::Time end_time = ros::Time::now();
      ros::Time end_time = rclcpp::Clock().now();
      std::cout << "reached end_pt" << std::endl;
      std::cout << "total_time: " << (end_time - start_time).toSec()
                << " seconds" << std::endl;
      AStarNode* end_node = current_node;
      std::cout << "use_node_num: " << use_node_num_ << std::endl;
      std::cout << "total_cost:" << end_node->g_cost << std::endl;
      retrievePath(end_node, path);
      return REACH_END;
    }

    for (double x = -resolution_; x <= resolution_; x += resolution_)
      for (double y = -resolution_; y <= resolution_; y += resolution_)
        for (double z = -resolution_; z <= resolution_; z += resolution_) {
          Eigen::Vector3d neighbor_pos =
              current_node->position + Eigen::Vector3d(x, y, z);
          // std::cout << "neighbor_pos: " << neighbor_pos.transpose() <<
          // std::endl;

          // if neighbor is out of map, skip it
          if (grid_map_->isInMap(neighbor_pos) == false) {
            // std::cout << "out of map" << std::endl;
            continue;
          }

          // if neighbor is obstacle, skip it
          if (grid_map_->getInflateOccupancy(neighbor_pos) == true) {
            // std::cout << "known obstacle" << std::endl;
            continue;
          }

          // if neighbor is already in close_list, skip it
          if (close_list_.find(neighbor_pos) != nullptr) {
            // std::cout << "already in close_list" << std::endl;
            continue;
          }

          double delta_pos = Eigen::Vector3d(x, y, z).norm();
          double tmp_g_cost = current_node->g_cost + delta_pos;
          AStarNode* tmp_node = expanded_nodes_.find(neighbor_pos);

          if (tmp_node == nullptr) {
            AStarNode* neighbor_node = path_node_pool_[use_node_num_];
            use_node_num_ += 1;
            neighbor_node->g_cost = tmp_g_cost;
            neighbor_node->position = neighbor_pos;
            neighbor_node->parent = current_node;
            neighbor_node->f_cost =
                neighbor_node->g_cost +
                lambda_ * getDiagonalHeu(neighbor_pos, end_pt);
            neighbor_node->node_state = AStarNode::NodeStateEnum::IN_OPEN_LIST;
            open_list_.push(neighbor_node);
            expanded_nodes_.insert(neighbor_node->position, neighbor_node);
            if (use_node_num_ >= allocated_node_num_) {
              std::cerr << "Error: allocated_node_num is too small."
                        << std::endl;
              return NO_PATH_FOUND;
            }
          } else if (tmp_g_cost < tmp_node->g_cost) {
            tmp_node->g_cost = tmp_g_cost;
            tmp_node->parent = current_node;
            tmp_node->f_cost =
                tmp_node->g_cost +
                lambda_ * getDiagonalHeu(tmp_node->position, end_pt);
          }
        }
  }

  std::cout << "no path found!" << std::endl;
  std::cout << "use_node_num: " << use_node_num_ << std::endl;
  return NO_PATH_FOUND;
}

double AStar::getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
  return tie_breaker_ * (x1 - x2).norm();
}

double AStar::getDiagonalHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
  double dx = std::abs(x1(0) - x2(0));
  double dy = std::abs(x1(1) - x2(1));
  double dz = std::abs(x1(2) - x2(2));
  double min_xyz = std::min({dx, dy, dz});
  double h = dx + dy + dz + (std::sqrt(3) - 3) * min_xyz;
  return tie_breaker_ * h;
}

// Eigen::Vector3i AStar::posToIndex(Eigen::Vector3d pos) {
//   Eigen::Vector3i index;
//   index(0) =  std::floor((pos(0) - origin_(0)) * inv_resolution_); // fix bug
//   index(1) =  std::floor((pos(1) - origin_(1)) * inv_resolution_);
//   index(2) =  std::floor((pos(2) - origin_(2)) * inv_resolution_);
//   return index;
// }

void AStar::retrievePath(AStarNode* end_node,
                         std::vector<Eigen::Vector3d>& path) {
  AStarNode* current_node = end_node;
  while (current_node->parent != nullptr) {
    path.push_back(current_node->position);
    current_node = current_node->parent;
  }
  path.push_back(current_node->position);
  std::reverse(path.begin(), path.end());
}

void AStar::setGridMap(GridMap::Ptr& grid_map) { this->grid_map_ = grid_map; }

void AStar::reset() {
  this->expanded_nodes_.clear();
  this->close_list_.clear();
  this->path_.clear();

  std::priority_queue<AStarNode*, std::vector<AStarNode*>, AStarNodeComparator>
      empty_queue;
  this->open_list_.swap(empty_queue);

  use_node_num_ = 0;
  for (int i = 0; i < allocated_node_num_; i++) {
    AStarNode* node = path_node_pool_[i];
    node->parent = nullptr;
    node->g_cost = std::numeric_limits<double>::infinity();
    node->f_cost = std::numeric_limits<double>::infinity();
    node->node_state = AStarNode::NodeStateEnum::NOT_EXPANDED;
  }
}

}  // namespace path_searching