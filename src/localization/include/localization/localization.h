//
// Created by zhihui on 1/17/20.
//

#ifndef LOCALIZATION_LOCALIZATION_H
#define LOCALIZATION_LOCALIZATION_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class Localization : public rclcpp::Node {
public:
    explicit Localization(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

};


#endif //LOCALIZATION_LOCALIZATION_H
