//
// Created by zhihui on 1/17/20.
//

#include "localization/localization.h"
#include "localization/private/scan_transform.h"

Localization::Localization(Route::Ptr activeRoute, const rclcpp::NodeOptions &options)
        : Node("localization", options), activeRoute_(activeRoute) {

    loam_mapp_matcher = std::make_unique<lins::lego_loam::LOAMMappingMatcherLego>(activeRoute->globalCorner,
                                                                                  activeRoute->globalSurf,
                                                                                  activeRoute->mapCornerKDTree,
                                                                                  activeRoute->mapSurfKDTree);

    scan_subscription_ = this->create_subscription<ScanROS>(
            "/scan", 10, std::bind(&Localization::ScanCallback, this, std::placeholders::_1));
}

void Localization::ScanCallback(ScanROS::SharedPtr msg) {
    if (!has_init) {
        prevLocalizationScan = importFromRosMsg(*msg);

        has_init = true;
        return;
    }

    Scan::Ptr scan;

    if (has_init) {
        scan = importFromRosMsg(*msg);
    }

    /// Mapp matching
    const Sophus::SE3d &prevOdomPose = prevLocalizationScan->poseOdom;
    const Sophus::SE3d &endOdomPose = scan->poseOdom;
    Sophus::SE3d odomDiff = prevOdomPose.inverse() * endOdomPose;

    loam_mapp_matcher->mappingMatcher(*prevLocalizationScan,
                                      *scan,
                                      prevLocalizationScan->poseMapp * activeRoute_->tf->T_I_L * odomDiff,
                                      activeRoute_->tf->T_I_L);

    /// Maintain Map
    auto checkLost = [](Scan::Ptr prevScan, Scan::Ptr curScan) -> bool {
        return (curScan->poseMapp.translation() - prevScan->poseMapp.translation()).norm()
               > Config::get<double>("slam.lost_translation");
    };

    auto checkLost2 = [](Scan::Ptr prevScan, Scan::Ptr curScan) -> bool {
        double dist = (curScan->poseMapp.translation() - prevScan->poseMapp.translation()).norm();
        double time = curScan->timestamp - prevScan->timestamp / 1000000000.;
        double velocity = dist / time;
        return velocity > Config::get<double>("slam.lost_translation") * 2.0;
    };

    //check lost using speed？ 20m/s  (need redefinition)
    if (checkLost2(prevLocalizationScan, scan)) {
        return;
    }

    Sophus::SE3f odom_T_W_L = (scan->poseOdom * activeRoute_->tf->T_I_L).cast<float>();
    Sophus::SE3f mapp_T_W_L = (scan->poseMapp * activeRoute_->tf->T_I_L).cast<float>();
    Eigen::Vector3f odom_offset = odom_T_W_L.translation().cast<float>();
    Eigen::Quaternionf odom_rotation = odom_T_W_L.unit_quaternion().cast<float>();
    Eigen::Vector3f offset = mapp_T_W_L.translation().cast<float>();
    Eigen::Quaternionf rotation = mapp_T_W_L.unit_quaternion().cast<float>();

    //Sensor2Base
    Eigen::Vector3d base_x(1, 0, 0);
    Eigen::Vector3d transform_x = rotation.toRotationMatrix().cast<double>() * base_x;
    float length_x = sqrt(
            transform_x[0] * transform_x[0] + transform_x[1] * transform_x[1] + transform_x[2] * transform_x[2]);

    pcl::PointCloud<lins::PointRawT>::Ptr mapRectCloud(new pcl::PointCloud <lins::PointRawT>);
    pcl::transformPointCloud(scan->rawScan, //raw
                             *mapRectCloud,
                             offset,
                             rotation);
    //TODO
    // publish pointcloud & odometry data

    /// Forward scan
    prevLocalizationScan = scan;
}
