#include <nav_msgs/Odometry.h>
// ROS1: #include <nodelet/nodelet.h>
// ROS2: No nodelet support - convert to component node
#include <quadrotor_msgs/Corrections.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>
// ROS1: #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/Imu.h>
#include <so3_control/SO3Control.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Eigen>

class SO3ControlNodelet : public nodelet::Nodelet {
 public:
  SO3ControlNodelet()
      : position_cmd_updated_(false),
        position_cmd_init_(false),
        des_yaw_(0),
        des_yaw_dot_(0),
        current_yaw_(0),
        enable_motors_(true),
        use_external_yaw_(false) {}

  void onInit();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  void publishSO3Command();
  void position_cmd_callback(
      const quadrotor_msgs::PositionCommand::ConstPtr& cmd);
  void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
  void enable_motors_callback(const std_msgs::Bool::ConstPtr& msg);
  void corrections_callback(const quadrotor_msgs::Corrections::ConstPtr& msg);
  void imu_callback(const sensor_msgs::Imu& imu);

  SO3Control controller_;
// ROS1:   ros::Publisher so3_command_pub_;
  rclcpp::Publisher so3_command_pub_;
// ROS1:   ros::Subscriber odom_sub_;
  rclcpp::Subscriber odom_sub_;
// ROS1:   ros::Subscriber position_cmd_sub_;
  rclcpp::Subscriber position_cmd_sub_;
// ROS1:   ros::Subscriber enable_motors_sub_;
  rclcpp::Subscriber enable_motors_sub_;
// ROS1:   ros::Subscriber corrections_sub_;
  rclcpp::Subscriber corrections_sub_;
// ROS1:   ros::Subscriber imu_sub_;
  rclcpp::Subscriber imu_sub_;

  bool position_cmd_updated_, position_cmd_init_;
  std::string frame_id_;

  Eigen::Vector3d des_pos_, des_vel_, des_acc_, kx_, kv_;
  double des_yaw_, des_yaw_dot_;
  double current_yaw_;
  bool enable_motors_;
  bool use_external_yaw_;
  double kR_[3], kOm_[3], corrections_[3];
  double init_x_, init_y_, init_z_;
};

void SO3ControlNodelet::publishSO3Command() {
  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_yaw_,
                               des_yaw_dot_, kx_, kv_);

  const Eigen::Vector3d& force = controller_.getComputedForce();
  const Eigen::Quaterniond& orientation = controller_.getComputedOrientation();

  quadrotor_msgs::SO3Command::Ptr so3_command(
      new quadrotor_msgs::SO3Command);  //! @note memory leak?
// ROS1:   so3_command->header.stamp = ros::Time::now();
  so3_command->header.stamp = rclcpp::Clock().now();
  so3_command->header.frame_id = frame_id_;
  so3_command->force.x = force(0);
  so3_command->force.y = force(1);
  so3_command->force.z = force(2);
  so3_command->orientation.x = orientation.x();
  so3_command->orientation.y = orientation.y();
  so3_command->orientation.z = orientation.z();
  so3_command->orientation.w = orientation.w();
  for (int i = 0; i < 3; i++) {
    so3_command->kR[i] = kR_[i];
    so3_command->kOm[i] = kOm_[i];
  }
  so3_command->aux.current_yaw = current_yaw_;
  so3_command->aux.kf_correction = corrections_[0];
  so3_command->aux.angle_corrections[0] = corrections_[1];
  so3_command->aux.angle_corrections[1] = corrections_[2];
  so3_command->aux.enable_motors = enable_motors_;
  so3_command->aux.use_external_yaw = use_external_yaw_;
  so3_command_pub_.publish(so3_command);
}

void SO3ControlNodelet::position_cmd_callback(
    const quadrotor_msgs::PositionCommand::ConstPtr& cmd) {
  des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y,
                             cmd->acceleration.z);

  if (cmd->kx[0] > 1e-5 || cmd->kx[1] > 1e-5 || cmd->kx[2] > 1e-5) {
    kx_ = Eigen::Vector3d(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
  }
  if (cmd->kv[0] > 1e-5 || cmd->kv[1] > 1e-5 || cmd->kv[2] > 1e-5) {
    kv_ = Eigen::Vector3d(cmd->kv[0], cmd->kv[1], cmd->kv[2]);
  }

  des_yaw_ = cmd->yaw;
  des_yaw_dot_ = cmd->yaw_dot;
  position_cmd_updated_ = true;
  position_cmd_init_ = true;

  publishSO3Command();
}

void SO3ControlNodelet::odom_callback(
    const nav_msgs::Odometry::ConstPtr& odom) {
  const Eigen::Vector3d position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);

// ROS1:   current_yaw_ = tf::getYaw(odom->pose.pose.orientation);
  current_yaw_ = tf::getYaw(odom->pose.pose.orientation);

  controller_.setPosition(position);
  controller_.setVelocity(velocity);

  if (position_cmd_init_) {
    // We set position_cmd_updated_ = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the so3 command ourselves
    // TODO: Fallback to hover if position_cmd hasn't been received for some
    // time
    if (!position_cmd_updated_) publishSO3Command();
    position_cmd_updated_ = false;
  } else if (init_z_ > -9999.0) {
    des_pos_ = Eigen::Vector3d(init_x_, init_y_, init_z_);
    des_vel_ = Eigen::Vector3d(0, 0, 0);
    des_acc_ = Eigen::Vector3d(0, 0, 0);
    publishSO3Command();
  }
}

void SO3ControlNodelet::enable_motors_callback(
    const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data)
    ROS_INFO("Enabling motors");
  else
    ROS_INFO("Disabling motors");

  enable_motors_ = msg->data;
}

void SO3ControlNodelet::corrections_callback(
    const quadrotor_msgs::Corrections::ConstPtr& msg) {
  corrections_[0] = msg->kf_correction;
  corrections_[1] = msg->angle_corrections[0];
  corrections_[2] = msg->angle_corrections[1];
}

void SO3ControlNodelet::imu_callback(const sensor_msgs::Imu& imu) {
  const Eigen::Vector3d acc(imu.linear_acceleration.x,
                            imu.linear_acceleration.y,
                            imu.linear_acceleration.z);
  controller_.setAcc(acc);
}

void SO3ControlNodelet::onInit() {
// ROS1:   ros::NodeHandle nh(getPrivateNodeHandle());
  rclcpp::Node::SharedPtr nh(getPrivateNodeHandle());

  std::string quadrotor_name;
// ROS1:   nh.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
nh->declare_parameter<quadrotor_name>("quadrotor_name", std::string("quadrotor");
nh->get_parameter("quadrotor_name", quadrotor_name);
  frame_id_ = "/" + quadrotor_name;

  double mass;
// ROS1:   nh.param("mass", mass, 0.5);
nh->declare_parameter<mass>("mass", 0.5);
nh->get_parameter("mass", mass);
  controller_.setMass(mass);

// ROS1:   nh.param("use_external_yaw", use_external_yaw_, true);
nh->declare_parameter<use_external_yaw_>("use_external_yaw", true);
nh->get_parameter("use_external_yaw", use_external_yaw_);

// ROS1:   nh.param("gains/rot/x", kR_[0], 1.5);
nh->declare_parameter<kR_>("gains/rot/x", [0], 1.5);
nh->get_parameter("gains/rot/x", kR_);
// ROS1:   nh.param("gains/rot/y", kR_[1], 1.5);
nh->declare_parameter<kR_>("gains/rot/y", [1], 1.5);
nh->get_parameter("gains/rot/y", kR_);
// ROS1:   nh.param("gains/rot/z", kR_[2], 1.0);
nh->declare_parameter<kR_>("gains/rot/z", [2], 1.0);
nh->get_parameter("gains/rot/z", kR_);
// ROS1:   nh.param("gains/ang/x", kOm_[0], 0.13);
nh->declare_parameter<kOm_>("gains/ang/x", [0], 0.13);
nh->get_parameter("gains/ang/x", kOm_);
// ROS1:   nh.param("gains/ang/y", kOm_[1], 0.13);
nh->declare_parameter<kOm_>("gains/ang/y", [1], 0.13);
nh->get_parameter("gains/ang/y", kOm_);
// ROS1:   nh.param("gains/ang/z", kOm_[2], 0.1);
nh->declare_parameter<kOm_>("gains/ang/z", [2], 0.1);
nh->get_parameter("gains/ang/z", kOm_);
// ROS1:   nh.param("gains/kx/x", kx_[0], 5.7);
nh->declare_parameter<kx_>("gains/kx/x", [0], 5.7);
nh->get_parameter("gains/kx/x", kx_);
// ROS1:   nh.param("gains/kx/y", kx_[1], 5.7);
nh->declare_parameter<kx_>("gains/kx/y", [1], 5.7);
nh->get_parameter("gains/kx/y", kx_);
// ROS1:   nh.param("gains/kx/z", kx_[2], 6.2);
nh->declare_parameter<kx_>("gains/kx/z", [2], 6.2);
nh->get_parameter("gains/kx/z", kx_);
// ROS1:   nh.param("gains/kv/x", kv_[0], 3.4);
nh->declare_parameter<kv_>("gains/kv/x", [0], 3.4);
nh->get_parameter("gains/kv/x", kv_);
// ROS1:   nh.param("gains/kv/y", kv_[1], 3.4);
nh->declare_parameter<kv_>("gains/kv/y", [1], 3.4);
nh->get_parameter("gains/kv/y", kv_);
// ROS1:   nh.param("gains/kv/z", kv_[2], 4.0);
nh->declare_parameter<kv_>("gains/kv/z", [2], 4.0);
nh->get_parameter("gains/kv/z", kv_);

// ROS1:   nh.param("corrections/z", corrections_[0], 0.0);
nh->declare_parameter<corrections_>("corrections/z", [0], 0.0);
nh->get_parameter("corrections/z", corrections_);
// ROS1:   nh.param("corrections/r", corrections_[1], 0.0);
nh->declare_parameter<corrections_>("corrections/r", [1], 0.0);
nh->get_parameter("corrections/r", corrections_);
// ROS1:   nh.param("corrections/p", corrections_[2], 0.0);
nh->declare_parameter<corrections_>("corrections/p", [2], 0.0);
nh->get_parameter("corrections/p", corrections_);

// ROS1:   nh.param("so3_control/init_state_x", init_x_, 0.0);
nh->declare_parameter<init_x_>("so3_control/init_state_x", 0.0);
nh->get_parameter("so3_control/init_state_x", init_x_);
// ROS1:   nh.param("so3_control/init_state_y", init_y_, 0.0);
nh->declare_parameter<init_y_>("so3_control/init_state_y", 0.0);
nh->get_parameter("so3_control/init_state_y", init_y_);
// ROS1:   nh.param("so3_control/init_state_z", init_z_, -10000.0);
nh->declare_parameter<init_z_>("so3_control/init_state_z", -10000.0);
nh->get_parameter("so3_control/init_state_z", init_z_);

  so3_command_pub_ = nh.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);

  odom_sub_ = nh.subscribe("odom", 10, &SO3ControlNodelet::odom_callback, this,
// ROS1:                            ros::TransportHints().tcpNoDelay());
// ROS2: TransportHints not needed
  position_cmd_sub_ = nh.subscribe("position_cmd", 10,
                                   &SO3ControlNodelet::position_cmd_callback,
// ROS1:                                    this, ros::TransportHints().tcpNoDelay());
// ROS2: TransportHints not needed
  enable_motors_sub_ =
      nh.subscribe("motors", 2, &SO3ControlNodelet::enable_motors_callback,
// ROS1:                    this, ros::TransportHints().tcpNoDelay());
// ROS2: TransportHints not needed
  corrections_sub_ =
      nh.subscribe("corrections", 10, &SO3ControlNodelet::corrections_callback,
// ROS1:                    this, ros::TransportHints().tcpNoDelay());
// ROS2: TransportHints not needed
  imu_sub_ = nh.subscribe("imu", 10, &SO3ControlNodelet::imu_callback, this,
// ROS1:                           ros::TransportHints().tcpNoDelay());
// ROS2: TransportHints not needed
}

#include <pluginlib/class_list_macros.h>
// PLUGINLIB_DECLARE_CLASS(so3_control, SO3ControlNodelet, SO3ControlNodelet,
// nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(SO3ControlNodelet, nodelet::Nodelet);