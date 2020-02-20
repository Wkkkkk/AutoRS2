//
// Created by zhihui on 1/16/20.
//

#ifndef DRIVER_ROS_DRIVER_H
#define DRIVER_ROS_DRIVER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "laser/common/config.h"
#include "laser/common/basic_types.h"
#include "laser/mapper/laser_mapp.h"

class SLAMMappingNode : public rclcpp::Node {
public:
    using PointCloudDataType = sensor_msgs::msg::PointCloud2;

    explicit SLAMMappingNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    void PointCloudDataCallback(PointCloudDataType::SharedPtr msg);

    std::string config_file;
    std::string rmap_filename;

    lasermapp::common::Config::Ptr gConfig;
    lasermapp::mapper::LaserMapper::Ptr laserMapp;

    rclcpp::Subscription<PointCloudDataType>::SharedPtr subscription_;
};


#endif //DRIVER_ROS_DRIVER_H
