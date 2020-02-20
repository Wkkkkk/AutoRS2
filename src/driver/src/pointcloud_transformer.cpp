//
// Created by zhihui on 1/16/20.
//

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "driver/point_type.h"
#include "driver/pointcloud_transformer.h"

class PointCloudTransformer::Impl {
public:
    PointCloudDataType::UniquePtr transform(PointCloudDataType::SharedPtr msg) {
        PointRawCloudT raw_data;
        pcl::fromROSMsg(*msg.get(), raw_data);

        Eigen::Quaterniond quat_b_l = Eigen::Quaterniond(
                Eigen::AngleAxisd(B_L_rot_z * M_PI / 180.0, Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(B_L_rot_y * M_PI / 180.0, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(B_L_rot_x * M_PI / 180.0, Eigen::Vector3d::UnitX()));
        Eigen::Vector3d vect_b_l = Eigen::Vector3d(device_B_L_offset_x, device_B_L_offset_y, device_B_L_offset_z);

        PointRawCloudT transformed_data;
        pcl::transformPointCloud(raw_data, transformed_data, vect_b_l, quat_b_l);

        auto result_msg = std::make_unique<PointCloudDataType>();
        pcl::toROSMsg(transformed_data, *result_msg);

        return result_msg;
    }

private:
    double B_L_rot_x = .0;
    double B_L_rot_y = .0;
    double B_L_rot_z = .0;
    double device_B_L_offset_x = .0;
    double device_B_L_offset_y = .0;
    double device_B_L_offset_z = .0;
};

PointCloudTransformer::PointCloudTransformer(const rclcpp::NodeOptions &options)
        : Node("pointcloud_transformer", options), pimpl_{std::make_unique<Impl>()} {
    subscription_ = this->create_subscription<PointCloudDataType>(
            "/velodyne_points", 10,
            std::bind(&PointCloudTransformer::PointCloudDataCallback, this, std::placeholders::_1));

    publisher_ = create_publisher<PointCloudDataType>("/transformed_points", 10);
}

PointCloudTransformer::~PointCloudTransformer() {}

void PointCloudTransformer::PointCloudDataCallback(PointCloudDataType::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Ros Subscriber received point cloud data with width %d, height %d",
                msg->width, msg->height);

    //do something with msg
    auto transfromed_pointcloud = pimpl_->transform(msg);

    publisher_->publish(std::move(transfromed_pointcloud));
}