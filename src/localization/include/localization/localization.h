//
// Created by zhihui on 1/17/20.
//

#ifndef LOCALIZATION_LOCALIZATION_H
#define LOCALIZATION_LOCALIZATION_H

#include <sophus/se3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "message/msg/scan.hpp"
#include "message/msg/measurement.hpp"
#include "private/gps_filter.h"

class Localization : public rclcpp::Node {
public:
    using PointCloudDataType = sensor_msgs::msg::PointCloud2;
    using Measurement = message::msg::Measurement;
    using ScanROS = message::msg::Scan;

    explicit Localization(Route::Ptr activeRoute, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    void ScanCallback(ScanROS::SharedPtr msg);

    bool has_init = false;
    Scan::Ptr prevLocalizationScan;
    Route::Ptr activeRoute_;
    std::unique_ptr <lins::lego_loam::LOAMMappingMatcherLego> loam_mapp_matcher;

    rclcpp::Publisher<PointCloudDataType>::SharedPtr pointcloud_publisher_;
    rclcpp::Subscription<ScanROS>::SharedPtr scan_subscription_;
};


#endif //LOCALIZATION_LOCALIZATION_H
