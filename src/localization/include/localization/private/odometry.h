//
// Created by zhihui on 1/18/20.
//

#ifndef LOCALIZATION_ODOMETRY_H
#define LOCALIZATION_ODOMETRY_H

#include <memory>

#include <sophus/se3.hpp>
#include <rclcpp/rclcpp.hpp>

#include "message/msg/measurement.hpp"
#include "message/msg/scan.hpp"
#include "localization/private/gps_filter.h"

class Odometry : public rclcpp::Node {
public:
    using Measurement = message::msg::Measurement;
    using ScanROS = message::msg::Scan;

    explicit Odometry(Route::Ptr activeRoute, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    void init(Measurement::SharedPtr msg);

    void process(Measurement::SharedPtr msg);

    void MeasurementCallback(Measurement::SharedPtr msg);

    void poseMsg(Scan::Ptr scan);

    enum class Status {
        UNINITIAL, INITIAL, WAITING, SUCESS, LOST
    };
    Status status_ = Status::UNINITIAL;
    Route::Ptr activeRoute_;
    Sophus::SE3d init_pose;
    Scan::Ptr prevScan;
    int scanIDCounter = 1;
    std::unique_ptr <GPSFilter> gpsfilter;
    std::unique_ptr <lins::lego_loam::LOAMOdometryMatcherLego> loam_odom_matcher;

    rclcpp::Publisher<ScanROS>::SharedPtr scan_publisher;
    rclcpp::Subscription<Measurement>::SharedPtr measurement_subscription_;
};


#endif //LOCALIZATION_ODOMETRY_H
