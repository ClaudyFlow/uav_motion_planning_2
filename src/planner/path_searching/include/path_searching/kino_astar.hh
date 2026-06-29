// ROS1: #ifndef SRC_PLANNER_PATH_SEARCHING_INCLUDE_PATH_SEARCHING_KINO_ASTAR
#include <rclcpp/rclcpp.hpp>
#define SRC_PLANNER_PATH_SEARCHING_INCLUDE_PATH_SEARCHING_KINO_ASTAR

#pragma region include
#pragma region include_project
#include "plan_env/grid_map.hh"
#pragma endregion include_project
#pragma region include_third
#include <pcl/kdtree/kdtree_flann.h>
#include <visualization_msgs/Marker.h>
#pragma endregion include_third
#pragma region include_standard
#include <unordered_map>
#pragma endregion include_standard
#pragma endregion include

#pragma region namespace
namespace path_searching {

#pragma region class
class KinoAStarNode {
 public:
  KinoAStarNode() {
    input = Eigen::Vector3d::Zero();
    f_cost = std::numeric_limits<double>::infinity();
    g_cost = std::numeric_limits<double>::infinity();
    duration = std::numeric_limits<double>::infinity();
    node_state = NodeStateEnum::NOT_EXPANDED;
    parent = nullptr;
  }
  ~KinoAStarNode() = default;

 public:
  enum class NodeStateEnum {
    NOT_EXPANDED,
    IN_OPEN_LIST,
    IN_CLOSE_LIST,
  };

 public:
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d input;
  Eigen::Vector3i
      index;  // index of the node in the occupancy grid, used for pruning
  double f_cost;
  double g_cost;
  double duration;
  NodeStateEnum node_state;
  KinoAStarNode* parent;

};  // class KinoAStarNode
#pragma endregion class

#pragma region class
class KinoAStarNodeComparator {
 public:
  KinoAStarNodeComparator() = default;
  ~KinoAStarNodeComparator() = default;

 public:
  bool operator()(KinoAStarNode* node1, KinoAStarNode* node2) {
    return (node1->f_cost > node2->f_cost);
  }
};  // class KinoAStarNodeComparator
#pragma endregion class

template <typename T>
struct vector3i_hash {
  size_t operator()(const T& vector) const {
    size_t seed = 0;
    for (size_t i = 0; i < vector.size(); ++i) {
      auto elem = *(vector.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }

    return seed;
  }
};  // struct vector3i_hash

#pragma region class
class KinoAStarNodeHashTable {
 private:
  std::unordered_map<Eigen::Vector3i, KinoAStarNode*,
                     vector3i_hash<Eigen::Vector3i>>
      data_table_;

 public:
  void insert(Eigen::Vector3i index, KinoAStarNode* node) {
    data_table_.insert(std::make_pair(index, node));
  }

  KinoAStarNode* find(Eigen::Vector3i index) {
    auto iter = data_table_.find(index);
    if (iter != data_table_.end()) {
      return iter->second;
    } else {
      return nullptr;
    }
  }

  inline void clear() { data_table_.clear(); }

  KinoAStarNodeHashTable() = default;
  ~KinoAStarNodeHashTable() = default;

};  // class KinoAStarNodeHashTable
#pragma endregion class

#pragma region class
class KinoAStar {
 public:
  KinoAStar() = default;
  ~KinoAStar();
// ROS1:   void setParam(ros::NodeHandle& nh);
  void setParam(rclcpp::Node::SharedPtr nh);
  void init();
  void setGridMap(GridMap::Ptr& grid_map);
  // second-order interator model
  // acceleration-controller motion planning
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
             Eigen::Vector3d end_pt, Eigen::Vector3d end_vel,
             std::vector<Eigen::Vector3d>& path);
  void reset();
  void visPathNodes(std::vector<Eigen::Vector3d>& path_nodes_list);
  void visEllipsoid(std::vector<Eigen::Vector3d>& path_nodes_list,
                    std::vector<Eigen::Matrix3d>& rot_list);

  typedef std::shared_ptr<KinoAStar> Ptr;

 private:
  /* main data structure */
  std::priority_queue<KinoAStarNode*, std::vector<KinoAStarNode*>,
                      KinoAStarNodeComparator>
      open_list_;
  KinoAStarNodeHashTable close_list_;
  KinoAStarNodeHashTable expanded_list_;
  std::vector<KinoAStarNode*> path_node_pool_;
  std::vector<Eigen::Matrix3d> rot_list;

  /* main search parameters */
  int allocated_node_num_;
  int use_node_num_;
  int collision_check_type_;
  double acc_res_;
  double rou_;
  double lambda_heu_;
  double tie_breaker_;
  double goal_tolerance_;
  double step_size_;
  double max_vel_;
  double max_accel_;
  double sample_tau_;

  Eigen::Matrix<double, 3, 4> shot_coef_;
  Eigen::Matrix<double, 3, 4> vel_coef_;
  Eigen::Matrix<double, 3, 4> acc_coef_;

  /* main robot parameters */
  double robot_r_;
  double robot_h_;

  /* main visulization parameters */
// ROS1:   ros::Publisher path_node_pub_;
  rclcpp::Publisher path_node_pub_;
  visualization_msgs::Marker path_node_marker_;

// ROS1:   ros::Publisher elliposid_pub_;
  rclcpp::Publisher elliposid_pub_;

  enum { REACH_END = 1, NO_PATH_FOUND = 2 };

  /* mian map parameters: grid map */
  Eigen::Vector3d origin_;
  Eigen::Vector3d map_size_;
  double resolution_;
  double inv_resolution_;
  GridMap::Ptr grid_map_;
  /* main map parameters: cloud map */
  std::vector<Eigen::Vector3d> obs_;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
// ROS1:   ros::Subscriber local_cloud_sub_;
  rclcpp::Subscriber local_cloud_sub_;

  /* main helper functions */
  Eigen::Vector3i posToIndex(Eigen::Vector3d pos);
  double getHeuristicCost(Eigen::Vector3d x1, Eigen::Vector3d v1,
                          Eigen::Vector3d x2, Eigen::Vector3d v2,
                          double& optimal_time);
  std::vector<double> cubic(double a, double b, double c, double d);
  std::vector<double> quartic(double a, double b, double c, double d, double e);
  bool computeShotTraj(Eigen::Vector3d x1, Eigen::Vector3d v1,
                       Eigen::Vector3d x2, Eigen::Vector3d v2,
                       double optimal_time);
  std::vector<KinoAStarNode*> retrievePath(
      KinoAStarNode* end_node, std::vector<Eigen::Vector3d>& path_nodes_list);
  void samplePath(std::vector<KinoAStarNode*> path_pool,
                  std::vector<Eigen::Vector3d>& path);
  void sampleEllipsoid(std::vector<KinoAStarNode*> path_pool,
                       std::vector<Eigen::Vector3d>& path,
                       std::vector<Eigen::Matrix3d>& rot_list);
  void StateTransit(Eigen::Matrix<double, 6, 1>& x0,
                    Eigen::Matrix<double, 6, 1>& xt, Eigen::Vector3d ut,
                    double dt);

  /* main se3 helper functions */
  void localCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  bool isCollisionFree(Eigen::Vector3d pt, Eigen::Vector3d acc);
  pcl::PointCloud<pcl::PointXYZ> toPCL(const std::vector<Eigen::Vector3d>& obs);

};  // class KinoAStar
#pragma endregion class

}  // namespace path_searching
#pragma endregion namespace
#endif /* SRC_PLANNER_PATH_SEARCHING_INCLUDE_PATH_SEARCHING_KINO_ASTAR */