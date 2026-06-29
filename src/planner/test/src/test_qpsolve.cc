// ROS1: #pragma region include
#include <rclcpp/rclcpp.hpp>
#pragma region include::project
#include "traj_optimization/minimum_control.hh"
#pragma endregion include::project
#pragma region include::third
// ROS1: #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#pragma endregion include::third
#pragma region include::standard

#pragma endregion include::standard
#pragma endregion include

int main(int argc, char** argv) {
// ROS1:   ros::init(argc, argv, "test_qp_solve");
  rclcpp::init(argc, argv, "test_qp_solve");
// ROS1:   ros::NodeHandle nh;
  rclcpp::Node::SharedPtr nh;

  traj_optimization::MinimumControl::Ptr min_jerk_ptr =
      std::make_shared<traj_optimization::MinimumControl>();

  Eigen::VectorXd pos_1d(4);
  pos_1d << 1.0, 2.0, 3.0, 4.0;
  Eigen::Vector2d bound_vel;
  bound_vel << 0.0, 0.0;
  Eigen::Vector2d bound_acc;
  bound_acc << 0.0, 0.0;
  Eigen::VectorXd time_vec(3);
  time_vec << 1.0, 1.0, 1.0;
  min_jerk_ptr->solve(pos_1d, bound_vel, bound_acc, time_vec);

// ROS1:   ros::spin();
  rclcpp::spin(nh);
  return 0;
}