//
// Created by zhihui on 1/16/20.
//

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "driver/pointcloud_transformer.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudTransformer>());

    return 0;
}