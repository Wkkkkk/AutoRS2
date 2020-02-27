//
// Created by zhihui on 1/17/20.
//

#ifndef LOCALIZATION_LOCALIZATION_H
#define LOCALIZATION_LOCALIZATION_H

#include <sophus/se3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "message/msg/measurement.hpp"
#include "private/gps_filter.h"

class Localization : public rclcpp::Node {
public:
    using PointCloudDataType = sensor_msgs::msg::PointCloud2;
    using Measurement = message::msg::Measurement;

    explicit Localization(Route::Ptr activeRoute, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    void MeasurementCallback(Measurement::SharedPtr msg);

    bool has_init = false;
    Scan::Ptr prevLocalizationScan;
    Route::Ptr activeRoute_;
    std::unique_ptr<lins::lego_loam::LOAMMappingMatcherLego> loam_mapp_matcher;

    rclcpp::Publisher<PointCloudDataType>::SharedPtr measurement_publisher;
    rclcpp::Subscription<Measurement>::SharedPtr subscription_;
};


#endif //LOCALIZATION_LOCALIZATION_H
