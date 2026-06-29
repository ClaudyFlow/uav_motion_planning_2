#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>

#include "maps.hpp"

void optimizeMap(mocka::Maps::BasicInfo& in) {
  std::vector<int>* temp = new std::vector<int>;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  cloud->width = in.cloud->width;
  cloud->height = in.cloud->height;
  cloud->points.resize(cloud->width * cloud->height);

  for (uint32_t i = 0; i < cloud->width; i++) {
    cloud->points[i].x = in.cloud->points[i].x;
    cloud->points[i].y = in.cloud->points[i].y;
    cloud->points[i].z = in.cloud->points[i].z;
  }

  kdtree.setInputCloud(cloud);
  double radius = 1.75 / in.scale;  // 1.75 is the rounded up value of sqrt(3)

  for (uint32_t i = 0; i < cloud->width; i++) {
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (kdtree.radiusSearch(cloud->points[i], radius, pointIdxRadiusSearch,
                            pointRadiusSquaredDistance) >= 27)
      temp->push_back(i);
  }

  for (int i = temp->size() - 1; i >= 0; i--)
    in.cloud->points.erase(in.cloud->points.begin() +
                           temp->at(i));  // erasing the enclosed points
  in.cloud->width -= temp->size();

  pcl::toROSMsg(*in.cloud, *in.output);
  in.output->header.frame_id = "world";
  ROS_INFO("finish: number of points after optimization %d", in.cloud->width);
  delete temp;
  return;
}

int main(int argc, char** argv) {
// ROS1:   ros::init(argc, argv, "mockamap");
  rclcpp::init(argc, argv, "mockamap");
// ROS1:   ros::NodeHandle nh;
  rclcpp::Node::SharedPtr nh;
// ROS1:   ros::NodeHandle nh_private("~");
  rclcpp::Node::SharedPtr nh_private("~");

// ROS1:   ros::Publisher pcl_pub =
  rclcpp::Publisher pcl_pub =
      nh.advertise<sensor_msgs::PointCloud2>("mock_map", 1);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;

  int seed;

  int sizeX;
  int sizeY;
  int sizeZ;

  double scale;
  double update_freq;

  int type;

// ROS1:   nh_private.param("seed", seed, 4546);
nh_private->declare_parameter<seed>("seed", 4546);
nh_private->get_parameter("seed", seed);
// ROS1:   nh_private.param("update_freq", update_freq, 1.0);
nh_private->declare_parameter<update_freq>("update_freq", 1.0);
nh_private->get_parameter("update_freq", update_freq);
// ROS1:   nh_private.param("resolution", scale, 0.38);
nh_private->declare_parameter<scale>("resolution", 0.38);
nh_private->get_parameter("resolution", scale);
// ROS1:   nh_private.param("x_length", sizeX, 100);
nh_private->declare_parameter<sizeX>("x_length", 100);
nh_private->get_parameter("x_length", sizeX);
// ROS1:   nh_private.param("y_length", sizeY, 100);
nh_private->declare_parameter<sizeY>("y_length", 100);
nh_private->get_parameter("y_length", sizeY);
// ROS1:   nh_private.param("z_length", sizeZ, 10);
nh_private->declare_parameter<sizeZ>("z_length", 10);
nh_private->get_parameter("z_length", sizeZ);

// ROS1:   nh_private.param("type", type, 1);
nh_private->declare_parameter<type>("type", 1);
nh_private->get_parameter("type", type);

  scale = 1 / scale;
  sizeX = sizeX * scale;
  sizeY = sizeY * scale;
  sizeZ = sizeZ * scale;

  mocka::Maps::BasicInfo info;
  info.nh_private = &nh_private;
  info.sizeX = sizeX;
  info.sizeY = sizeY;
  info.sizeZ = sizeZ;
  info.seed = seed;
  info.scale = scale;
  info.output = &output;
  info.cloud = &cloud;

  mocka::Maps map;
  map.setInfo(info);
  map.generate(type);

  //  optimizeMap(info);

  //! @note publish loop
// ROS1:   ros::Rate loop_rate(update_freq);
  rclcpp::Rate loop_rate(update_freq);
// ROS1:   while (ros::ok()) {
  while (rclcpp::ok()) {
    pcl_pub.publish(output);
// ROS1:     ros::spinOnce();
    rclcpp::spin_some(nh);
    loop_rate.sleep();
  }

  return 0;
}