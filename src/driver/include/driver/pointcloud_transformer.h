//
// Created by zhihui on 1/16/20.
//

#ifndef DRIVER_POINTCLOUD_TRANSFORMER_H
#define DRIVER_POINTCLOUD_TRANSFORMER_H

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudTransformer : public rclcpp::Node {
public:
    using PointCloudDataType = sensor_msgs::msg::PointCloud2;

    explicit PointCloudTransformer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~PointCloudTransformer();

private:
    class Impl;

    void PointCloudDataCallback(PointCloudDataType::SharedPtr msg);

    rclcpp::Subscription<PointCloudDataType>::SharedPtr subscription_;
    rclcpp::Publisher<PointCloudDataType>::SharedPtr publisher_;
    std::unique_ptr <Impl> pimpl_;
};


#endif //DRIVER_POINTCLOUD_TRANSFORMER_H
