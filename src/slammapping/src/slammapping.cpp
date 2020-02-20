//
// Created by zhihui on 1/16/20.
//

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <matcher.hpp>

#include "slammapping/slammapping.h"

SLAMMappingNode::SLAMMappingNode(const rclcpp::NodeOptions &options)
        : Node("slammapping_node", options) {

    rmap_filename = "/home/zhihui/workspace/data/global.rmap";
    config_file = "/home/zhihui/AutoRS/src/process/slammapping/config/mapping.yaml";

    gConfig = std::make_shared<lasermapp::common::Config>();
    gConfig->setParameterFile(config_file);
    gConfig->rawmap_export_ = rmap_filename;

    laserMapp.reset(new lasermapp::mapper::LaserMapper(gConfig));

    subscription_ = this->create_subscription<PointCloudDataType>(
            "/transformed_points", 10,
            std::bind(&SLAMMappingNode::PointCloudDataCallback, this, std::placeholders::_1));
}

void SLAMMappingNode::PointCloudDataCallback(PointCloudDataType::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Ros Subscriber received point cloud data with width %d, height %d",
                msg->width, msg->height);

    //TODO do something with msg
    lasermapp::PointRawCloudT raw_data;
    pcl::fromROSMsg(*msg.get(), raw_data);

    auto time_stamp = msg->header.stamp.nanosec;

    laserMapp->processScan(time_stamp, raw_data);
}