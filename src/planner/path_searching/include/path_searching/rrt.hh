// ROS1: #ifndef SRC_PLANNER_PATH_SEARCHING_INCLUDE_PATH_SEARCHING_RRT
#include <rclcpp/rclcpp.hpp>
#define SRC_PLANNER_PATH_SEARCHING_INCLUDE_PATH_SEARCHING_RRT

#pragma region include
#pragma region include_project
#include "kdtree/kdtree.hh"
#include "plan_env/grid_map.hh"
#pragma endregion include_project
#pragma region include_third

#pragma endregion include_third
#pragma region include_standard
#include <queue>
#include <random>
#pragma endregion include_standard
#pragma endregion include

namespace path_searching {

class RRTNode {
 public:
  Eigen::Vector3d position;
  double g_cost;                   // for total_cost
  RRTNode* parent;                 // for retrieve path
  std::vector<RRTNode*> children;  // for get whole tree

  RRTNode() {
    g_cost = std::numeric_limits<double>::infinity();
    parent = nullptr;
  }

  ~RRTNode() = default;

};  // class RRTNode

class RRT {
 private:
  /* main data structure */
  kdtree* kdtree_;
  std::vector<RRTNode*> path_node_pool_;

  /* main search parameters */
  int max_tree_node_num_;
  int use_node_num_;
  double step_length_;
  double max_allowed_time_;
  double tolerance_;
  double search_radius_;

  /* main map parameters */
  GridMap::Ptr grid_map_;
  Eigen::Vector3d origin_;
  Eigen::Vector3d map_size_;
  double resolution_;

  enum { REACH_END = 1, NO_PATH_FOUND = 2 };

  /* main helper functions */
  Eigen::Vector3d getRandomNode();
  Eigen::Vector3d Step(Eigen::Vector3d from, Eigen::Vector3d to,
                       double step_length);
  bool isCollisionFree(Eigen::Vector3d from, Eigen::Vector3d to,
                       double map_resolution);
  void retrievePath(RRTNode* node, std::vector<Eigen::Vector3d>& path);

 public:
  /* main interface */
// ROS1:   void setParam(ros::NodeHandle& nh);
  void setParam(rclcpp::Node::SharedPtr nh);
  void setGridMap(GridMap::Ptr& grid_map);
  void getWholeTree(
      std::vector<Eigen::Vector3d>& vertices,
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& edges);

  /* main init and reset function */
  void init();
  void reset();

  /* main search algorithm */
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt,
             std::vector<Eigen::Vector3d>& path);

  RRT() {};
  ~RRT();

  typedef std::shared_ptr<RRT> Ptr;

};  // class RRT

}  // namespace path_searching

#endif /* SRC_PLANNER_PATH_SEARCHING_INCLUDE_PATH_SEARCHING_RRT */