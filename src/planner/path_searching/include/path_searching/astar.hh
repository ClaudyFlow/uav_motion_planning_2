// ROS1: #ifndef SRC_PLANNER_PATH_SEARCHING_INCLUDE_PATH_SEARCHING_ASTAR
#include <rclcpp/rclcpp.hpp>
#define SRC_PLANNER_PATH_SEARCHING_INCLUDE_PATH_SEARCHING_ASTAR

#pragma region include
#pragma region include_project
#include "plan_env/grid_map.hh"
#pragma endregion include_project
#pragma region include_third

#pragma endregion include_third
#pragma region include_standard
#include <unordered_map>
#pragma endregion include_standard
#pragma endregion include

#pragma region namespace
namespace path_searching {

#pragma region class
class AStarNode {
#pragma region public
#pragma region public_constructor
 public:
  AStarNode() {
    g_cost = std::numeric_limits<double>::infinity();
    f_cost = std::numeric_limits<double>::infinity();
    node_state = NodeStateEnum::NOT_EXPANDED;
    parent = nullptr;
  }
  ~AStarNode() = default;
#pragma endregion public_constructor
#pragma region public_enum
 public:
  enum class NodeStateEnum {
    NOT_EXPANDED,
    IN_OPEN_LIST,
    IN_CLOSE_LIST,
  };  // enum class NodeStateEnum
#pragma endregion public_enum
#pragma region public_variable
 public:
  Eigen::Vector3d position;
  double f_cost;
  double g_cost;
  NodeStateEnum node_state;
  AStarNode* parent;
#pragma endregion public_variable
#pragma endregion public
};  // class AStarNode
#pragma endregion class

#pragma region class
class AStarNodeComparator {
#pragma region public
#pragma region public_constructor
 public:
  AStarNodeComparator() = default;
  ~AStarNodeComparator() = default;
#pragma endregion public_constructor
#pragma region public_functor
 public:
  bool operator()(const AStarNode* node1, const AStarNode* node2) const {
    return (node1->f_cost > node2->f_cost);
  }
#pragma endregion public_functor
#pragma endregion public
};  // class AStarNodeComparator
#pragma endregion class

#pragma region class
class NodeHashTable {
#pragma region public
#pragma region public_constructor
 public:
  NodeHashTable() = default;
  ~NodeHashTable() = default;
#pragma endregion public_constructor
#pragma region public_function
 public:
  inline void insert(Eigen::Vector3d pos, AStarNode* node) {
    node_table_.insert(std::make_pair(pos, node));
  }

  inline AStarNode* find(Eigen::Vector3d pos) {
    auto it = node_table_.find(pos);
    if (it != node_table_.end()) {
      return it->second;
    } else {
      return nullptr;
    }
  }

  inline void clear() { node_table_.clear(); }
#pragma endregion public_function
#pragma endregion public

#pragma region private
#pragma region private_template
 private:
  template <typename T>
  struct hash_func {
    size_t operator()(T const& x) const {
      size_t seed = 0;
      for (size_t i = 0; i < x.size(); ++i) {
        auto elem = *(x.data() + i);
        seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 +
                (seed << 6) + (seed >> 2);
      }
      return seed;
    }
  };
#pragma endregion private_template
#pragma region private_variable
 private:
  // don't use Eigen::Vector3i index as key, because different position may have
  // the same index
  std::unordered_map<Eigen::Vector3d, AStarNode*, hash_func<Eigen::Vector3d>>
      node_table_;
#pragma endregion private_variable
#pragma endregion private
};  // class NodeHashTable
#pragma endregion class

#pragma region class
class AStar {
#pragma region public
#pragma region public_constructor
 public:
  AStar() = default;
  ~AStar() {
    for (int i = 0; i < allocated_node_num_;
         i++) {  // free memory for path_node_pool_
      delete path_node_pool_[i];
    }
  }
#pragma endregion public_constructor
#pragma region public_enum
 public:
  enum { REACH_END = 1, NO_PATH_FOUND = 2 };
#pragma endregion public_enum
#pragma region public_function
 public:
// ROS1:   void setParam(ros::NodeHandle& nh);
  void setParam(rclcpp::Node::SharedPtr nh);
  void init();
  void setGridMap(GridMap::Ptr& grid_map);
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt,
             std::vector<Eigen::Vector3d>& path);
  void reset();

#pragma endregion public_function
#pragma endregion public

#pragma region private
#pragma region private_function
 private:
  /* helper function */
  // Eigen::Vector3i posToIndex(Eigen::Vector3d pos);
  double getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
  double getDiagonalHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
  void retrievePath(AStarNode* end_node, std::vector<Eigen::Vector3d>& path);
#pragma endregion private_function
#pragma region private_variable
 private:
  /* main data structure */
  std::priority_queue<AStarNode*, std::vector<AStarNode*>, AStarNodeComparator>
      open_list_;
  NodeHashTable close_list_;
  NodeHashTable expanded_nodes_;
  std::vector<AStarNode*> path_node_pool_;
  std::vector<Eigen::Vector3d> path_;

  /* main search parameters */
  int allocated_node_num_;  // pre-allocated the nodes num
  int use_node_num_;        // number of the nodes expanded
  double lambda_;           // weight for the heuristic term
  double tie_breaker_;      // enhance the search velocity

  /* main map parameters */
  Eigen::Vector3d origin_;
  Eigen::Vector3d map_size_;
  double resolution_;
  double inv_resolution_;
  GridMap::Ptr grid_map_;
#pragma endregion private_variable
#pragma endregion private
};  // class AStar
#pragma endregion class

}  // namespace path_searching
#pragma endregion namespace

#endif /* SRC_PLANNER_PATH_SEARCHING_INCLUDE_PATH_SEARCHING_ASTAR */