//
// Created by zhihui on 1/17/20.
//

#include "localization/private/data_synchronizer.h"

DataSynchronizer::DataSynchronizer(const rclcpp::NodeOptions &options)
        : Node("data_synchronizer", options) {
    imu_subscription_ = this->create_subscription<IMUDataType>(
            "/imu", 10, std::bind(&DataSynchronizer::IMUDataCallback, this, std::placeholders::_1));
    pointcloud_subscription_ = this->create_subscription<PointCloudDataType>(
            "/transformed_points", 10,
            std::bind(&DataSynchronizer::PointCloudDataCallback, this, std::placeholders::_1));

    measurement_publisher_ = this->create_publisher<Measurement>("/measurements", 10);
}

void DataSynchronizer::IMUDataCallback(IMUDataType::SharedPtr msg) {
    imu_buffer_.Push(msg);
}

void DataSynchronizer::PointCloudDataCallback(PointCloudDataType::SharedPtr msg) {
    auto measurement = std::make_unique<Measurement>();

    // pack some imu data from last frame
    const size_t max_imu_num = 10;
    const size_t previous_imu_num = imu_buffer_.Size();
    for (size_t i = 0; i < previous_imu_num && i < max_imu_num; ++i) {
        auto imu = imu_buffer_.Pop();
        measurement->imu.push_back(*imu);
    }
    // drop the left
    imu_buffer_.Clear();

    // TODO avoid this copy
    measurement->pointcloud = std::move(*msg);

    measurement_publisher_->publish(std::move(measurement));
}