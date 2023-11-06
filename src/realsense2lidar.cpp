// Copyright (c) 2022 Jonas Mahler

// This file is part of realsense2lidar.

// realsense2lidar is free software: you can redistribute it and/or modify it under the terms
// of the GNU General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.

// realsense2lidar is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along
// with Foobar. If not, see <https://www.gnu.org/licenses/>.

#define BOOST_BIND_NO_PLACEHOLDERS

#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>

#include "realsense2lidar/realsense2lidar_node.hpp"

#include "std_msgs/msg/header.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using std::placeholders::_1;

realsense2lidar::realsense2lidar(const rclcpp::NodeOptions& options) : Node("realsense2lidar", options)
{
  declare_parameter<std::string>("topic_pointcloud_in", "/camera/depth/color/points");
  declare_parameter<std::string>("topic_pointcloud_out", "realsense2lidar/point_cloud_realsense2lidar");

  param_topic_pointcloud_in = get_parameter("topic_pointcloud_in").as_string();
  param_topic_pointcloud_out = get_parameter("topic_pointcloud_out").as_string();

  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_out, 2);
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      param_topic_pointcloud_in, 10, std::bind(&realsense2lidar::topic_callback, this, _1));

  RCLCPP_INFO(this->get_logger(),
              "\n"
              "Node:       realsense2lidar\n"
              "Subscribes: Pointcloud2 message: %s\n"
              "Publishes:  Pointcloud2 message: %s \n"
              "Details:    No filter applied in this example.\n"
              "Running...",
              param_topic_pointcloud_in.c_str(), param_topic_pointcloud_out.c_str());
}

void realsense2lidar::topic_callback(sensor_msgs::msg::PointCloud2 points_msg)
{
  unsigned int num_points = points_msg.width;
  RCLCPP_INFO(this->get_logger(), "The number of points in the input pointcloud is %i", num_points);
  sensor_msgs::msg::PointCloud2 pcl_msg;

  // Modifier to describe what the fields are.
  sensor_msgs::PointCloud2Modifier modifier(pcl_msg);

  modifier.setPointCloud2Fields(4,
  "x", 1, sensor_msgs::msg::PointField::FLOAT32,
  "y", 1, sensor_msgs::msg::PointField::FLOAT32,
  "z", 1, sensor_msgs::msg::PointField::FLOAT32,
  "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);

  modifier.resize(points_msg.height * points_msg.width);
  pcl_msg.header.stamp = points_msg.header.stamp;
  pcl_msg.header.frame_id = points_msg.header.frame_id;
  pcl_msg.height = 1;

  //Iterators for PointCloud msg
  sensor_msgs::PointCloud2Iterator<float> iterX(pcl_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iterY(pcl_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iterZ(pcl_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iterIntensity(pcl_msg, "intensity");

  // Now create iterators for fields
  sensor_msgs::PointCloud2Iterator<float> iter_x(points_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(points_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(points_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(points_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(points_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(points_msg, "b");

  for (size_t i = 0; i < pcl_msg.height * pcl_msg.width-1; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b, ++iterX, ++iterY, ++iterZ, ++iterIntensity)
  {
    *iterX = *iter_x;
    *iterY = *iter_y;
    *iterZ = *iter_z;
    *iterIntensity = (*iter_r + *iter_g + *iter_b)/3;
  }

  // Publish to ROS2 network
  publisher_->publish(pcl_msg);
}
