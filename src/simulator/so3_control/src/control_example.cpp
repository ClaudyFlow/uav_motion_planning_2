#include <quadrotor_msgs/PositionCommand.h>
// ROS1: #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
int main(int argc, char** argv) {
// ROS1:   ros::init(argc, argv, "quad_sim_example");
  rclcpp::init(argc, argv, "quad_sim_example");
// ROS1:   ros::NodeHandle nh("~");
  rclcpp::Node::SharedPtr nh("~");

// ROS1:   ros::Publisher cmd_pub =
  rclcpp::Publisher cmd_pub =
      nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);

  ros::Duration(2.0).sleep();

// ROS1:   while (ros::ok()) {
  while (rclcpp::ok()) {
    /*** example 1: position control ***/
    std::cout << "\033[42m"
              << "Position Control to (2,0,1) meters"
              << "\033[0m" << std::endl;
    for (int i = 0; i < 500; i++) {
      quadrotor_msgs::PositionCommand cmd;
      cmd.position.x = 2.0;
      cmd.position.y = 0.0;
      cmd.position.z = 1.0;
      cmd_pub.publish(cmd);

      ros::Duration(0.01).sleep();
// ROS1:       ros::spinOnce();
      rclcpp::spin_some(nh);
    }

    /*** example 2: velocity control ***/
    std::cout << "\033[42m"
              << "Velocity Control to (-1,0,0) meters/second"
              << "\033[0m" << std::endl;
    for (int i = 0; i < 500; i++) {
      quadrotor_msgs::PositionCommand cmd;
      cmd.position.x =
          std::numeric_limits<float>::quiet_NaN();  // lower-order commands must
                                                    // be disabled by nan
      cmd.position.y =
          std::numeric_limits<float>::quiet_NaN();  // lower-order commands must
                                                    // be disabled by nan
      cmd.position.z =
          std::numeric_limits<float>::quiet_NaN();  // lower-order commands must
                                                    // be disabled by nan
      cmd.velocity.x = -1.0;
      cmd.velocity.y = 0.0;
      cmd.velocity.z = 0.0;
      cmd_pub.publish(cmd);

      ros::Duration(0.01).sleep();
// ROS1:       ros::spinOnce();
      rclcpp::spin_some(nh);
    }

    /*** example 3: accelleration control ***/
    std::cout << "\033[42m"
              << "Accelleration Control to (1,0,0) meters/second^2"
              << "\033[0m" << std::endl;
    for (int i = 0; i < 500; i++) {
      quadrotor_msgs::PositionCommand cmd;
      cmd.position.x =
          std::numeric_limits<float>::quiet_NaN();  // lower-order commands must
                                                    // be disabled by nan
      cmd.position.y =
          std::numeric_limits<float>::quiet_NaN();  // lower-order commands must
                                                    // be disabled by nan
      cmd.position.z =
          std::numeric_limits<float>::quiet_NaN();  // lower-order commands must
                                                    // be disabled by nan
      cmd.velocity.x =
          std::numeric_limits<float>::quiet_NaN();  // lower-order commands must
                                                    // be disabled by nan
      cmd.velocity.y =
          std::numeric_limits<float>::quiet_NaN();  // lower-order commands must
                                                    // be disabled by nan
      cmd.velocity.z =
          std::numeric_limits<float>::quiet_NaN();  // lower-order commands must
                                                    // be disabled by nan
      cmd.acceleration.x = 1.0;
      cmd.acceleration.y = 0.0;
      cmd.acceleration.z = 0.0;
      cmd_pub.publish(cmd);

      ros::Duration(0.01).sleep();
// ROS1:       ros::spinOnce();
      rclcpp::spin_some(nh);
    }
  }

  return 0;
}