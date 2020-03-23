//
// Created by zhihui on 1/18/20.
//

#include <pcl_conversions/pcl_conversions.h>

#include "localization/private/scan_transform.h"
#include "localization/private/odometry.h"
//#include "driver/point_type.h"

Odometry::Odometry(Route::Ptr activeRoute, const rclcpp::NodeOptions &options)
        : Node("Odometry", options), activeRoute_(activeRoute), prevScan(nullptr), gpsfilter(new GPSFilter),
          loam_odom_matcher(new lins::lego_loam::LOAMOdometryMatcherLego()) {

    scan_publisher = this->create_publisher<ScanROS>("/scan", 10);

    measurement_subscription_ = this->create_subscription<Measurement>(
            "/measurements", 10, std::bind(&Odometry::MeasurementCallback, this, std::placeholders::_1));
}

void Odometry::MeasurementCallback(Measurement::SharedPtr msg) {
    switch (status_) {
        case Status::UNINITIAL:
            init(msg);
            break;
        case Status::INITIAL:
        case Status::SUCESS:
            process(msg);
            break;
        case Status::WAITING:
            break;
        case Status::LOST:
        default:
            return;
    }
}

void Odometry::init(Measurement::SharedPtr msg) {
    Eigen::Vector3d initG;
    Eigen::Quaterniond initQ;
    initG.setZero();
    initG = Eigen::Vector3d(0.0, 0.0, 9.8);
    initQ = init_pose.unit_quaternion();
    Eigen::Vector3d initP = init_pose.translation();
    auto initHeading = Config::get<double>("localization.init_yaw");
    auto td = Config::get<double>("data_sync_td") * 1e9;

    const auto imu_data_list = msg->imu;
    const auto gps_data_list = msg->gps;
    const auto pointcloud_data = msg->pointcloud;

    Eigen::Matrix3d rotMat = Utility::g2R(initG);
    initQ = rotMat;
    initQ = Eigen::AngleAxisd(-initHeading * M_PI / 180.0, Eigen::Vector3d::UnitZ()) * initQ;

    Sophus::SE3d init_T_W_I0;
    init_T_W_I0.translation() = initP;
    init_T_W_I0.so3().setQuaternion(initQ.normalized());
    activeRoute_->tf->T_W_B0 = init_T_W_I0 * activeRoute_->tf->T_B_I.inverse();

    Sophus::SE3d init_T_W_I;
    init_T_W_I = init_T_W_I0 * activeRoute_->tf->T_I_G.inverse();

    Scan::Ptr initScan(new Scan);
    initScan->timestamp = pointcloud_data.header.stamp.nanosec + td;
    pcl::fromROSMsg(pointcloud_data, initScan->rawScan);

    //loam::extractFeature(*initScan);
    lins::lego_loam::extractFeatureLego(*initScan);
    pcl::copyPointCloud(initScan->lessSharpCloud, initScan->lessSharpCloudOdom);
    pcl::copyPointCloud(initScan->lessFlatCloud, initScan->lessFlatCloudOdom);
    pcl::copyPointCloud(initScan->lessSharpCloud, initScan->lessSharpCloudRect);
    pcl::copyPointCloud(initScan->lessFlatCloud, initScan->lessFlatCloudRect);
    pcl::copyPointCloud(initScan->outlierCloud, initScan->outlierCloudOdom);
    pcl::copyPointCloud(initScan->outlierCloud, initScan->outlierCloudRect);

    initScan->poseIMU = init_T_W_I;
    initScan->poseOdom = init_T_W_I;
    initScan->poseMapp = init_T_W_I;
    initScan->scanID = 0;
    initScan->state = Scan::KEYSCAN;
    initScan->keyScanID = 0;
    Trajectory::Ptr initTraj(new Trajectory);
    initTraj->data.emplace_back(initScan->timestamp, initScan->poseMapp);

    prevScan = initScan;
    poseMsg(initScan);

    status_ = Status::INITIAL;
}

void Odometry::process(Measurement::SharedPtr msg) {
    std::vector <IMUReading::Ptr> imus;

    const auto imu_data_list = msg->imu;
    const auto gps_data_list = msg->gps;
    const auto pointcloud_data = msg->pointcloud;

    Scan::Ptr scan(new Scan);
    scan->scanID = scanIDCounter++;
    pcl::fromROSMsg(pointcloud_data, scan->rawScan);

    lins::lego_loam::extractFeatureLego(*scan);
    scan->state = Scan::IMU;

    /// Odom matching
    Sophus::SE3d imuDiff;
    imuDiff = Sophus::SE3d();

    static const bool USE_ODOM = Config::get<bool>("localization.use_odom");
    if (USE_ODOM) {
        loam_odom_matcher->odometryMatcher(*prevScan, *scan, prevScan->poseOdom * imuDiff);
    } else {
        scan->poseOdom = scan->poseIMU;
        scan->state = Scan::ODOM;
    }
    if (scan->state != Scan::ODOM) {
        status_ = Status::LOST;
        return;
    }

    prevScan = scan;
    poseMsg(scan);
}

void Odometry::poseMsg(Scan::Ptr scan) {
    auto ros_msg = exportToRosMsg(scan);
    scan_publisher->publish(ros_msg);
}
