//
// Created by zhihui on 1/17/20.
//

#ifndef LOCALIZATION_DATA_SYNCHRONIZER_H
#define LOCALIZATION_DATA_SYNCHRONIZER_H

#include <vector>
#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "common.h"
#include "localization/blocking_queue.h"
#include "message/msg/measurement.hpp"

using namespace lins::common;

class DataSynchronizer : public rclcpp::Node {
public:
    using PointCloudDataType = sensor_msgs::msg::PointCloud2;
    using IMUDataType = sensor_msgs::msg::Imu;
    using Measurement = message::msg::Measurement;

    explicit DataSynchronizer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    void IMUDataCallback(IMUDataType::SharedPtr msg);

    void PointCloudDataCallback(PointCloudDataType::SharedPtr msg);

    core::BlockingQueue <IMUDataType::SharedPtr> imu_buffer_;

    rclcpp::Subscription<IMUDataType>::SharedPtr imu_subscription_;
    rclcpp::Subscription<PointCloudDataType>::SharedPtr pointcloud_subscription_;

    rclcpp::Publisher<Measurement>::SharedPtr measurement_publisher;
};

#endif //LOCALIZATION_DATA_SYNCHRONIZER_H
