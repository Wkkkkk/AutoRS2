//
// Created by zhihui on 3/23/20.
//

#include <pcl_conversions/pcl_conversions.h>
#include <sophus/se3.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include "localization/private/scan_transform.h"

void vector3MsgToSophus(const geometry_msgs::msg::Vector3 &v, Sophus::SE3d::Point &translation) {
    translation << v.x, v.y, v.z;
}

void quaternionMsgToEigen(const geometry_msgs::msg::Quaternion &m, Eigen::Quaterniond &e) {
    e = Eigen::Quaterniond(m.w, m.x, m.y, m.z);
}

void pointMsgToEigen(const geometry_msgs::msg::Point &m, Eigen::Vector3d &e) {
    e(0) = m.x;
    e(1) = m.y;
    e(2) = m.z;
}

geometry_msgs::msg::Point eigenToPointMsg(Eigen::Vector3d &e) {
    geometry_msgs::msg::Point p;
    p.x = e.x();
    p.y = e.y();
    p.z = e.z();
    return p;
}

geometry_msgs::msg::Quaternion eigenToQuaternionMsg(Eigen::Quaterniond &e) {
    geometry_msgs::msg::Quaternion q;
    q.w = e.w();
    q.x = e.x();
    q.y = e.y();
    q.z = e.z();
    return q;
}

void transformMsgToSophus(const geometry_msgs::msg::Transform &transform, Sophus::SE3d &se3) {
    Sophus::SE3d::Point translation;
    Eigen::Quaternion <Sophus::SE3d::Scalar> orientation;
    vector3MsgToSophus(transform.translation, translation);
    quaternionMsgToEigen(transform.rotation, orientation);
    se3 = Sophus::SE3d(orientation, translation);  // TODO faster way to set this than reconstructing
}

Sophus::SE3d transformMsgToSophus(const geometry_msgs::msg::Transform &transform) {
    Sophus::SE3d T;
    transformMsgToSophus(transform, T);
    return T;
}

geometry_msgs::msg::Transform sophusToTransformMsg(const Sophus::SE3d &se3) {
    geometry_msgs::msg::Transform msg;
    msg.translation.x = se3.translation().x();
    msg.translation.y = se3.translation().y();
    msg.translation.z = se3.translation().z();
    msg.rotation.x = se3.unit_quaternion().x();
    msg.rotation.y = se3.unit_quaternion().y();
    msg.rotation.z = se3.unit_quaternion().z();
    msg.rotation.w = se3.unit_quaternion().w();
    return msg;
}

void poseMsgToSophus(const geometry_msgs::msg::Pose &pose, Sophus::SE3d &se3) {
    Eigen::Quaternion <Sophus::SE3d::Scalar> orientation;
    Sophus::SE3d::Point translation;
    pointMsgToEigen(pose.position, translation);
    quaternionMsgToEigen(pose.orientation, orientation);
    se3 = Sophus::SE3d(orientation, translation);  // TODO faster way to set this than reconstructing
}

geometry_msgs::msg::Pose sophusToPoseMsg(const Sophus::SE3d &s) {
    geometry_msgs::msg::Pose pose;
    Eigen::Vector3d translation = s.translation();
    pose.position = eigenToPointMsg(translation);
    Eigen::Quaterniond quaternion = s.unit_quaternion();
    pose.orientation = eigenToQuaternionMsg(quaternion);
    return pose;
}

ScanROS exportToRosMsg(ScanPCL::Ptr scan) {
    ScanROS msg;
    msg.timestamp = scan->timestamp;
    pcl::toROSMsg(scan->rawScan, msg.raw_scan);
    pcl::toROSMsg(scan->lessSharpCloud, msg.less_sharp_cloud);
    pcl::toROSMsg(scan->lessSharpCloudOdom, msg.less_sharp_cloud_odom);
    pcl::toROSMsg(scan->lessSharpCloudRect, msg.less_sharp_cloud_rect);
    pcl::toROSMsg(scan->lessFlatCloud, msg.less_flat_cloud);
    pcl::toROSMsg(scan->lessFlatCloudOdom, msg.less_flat_cloud_odom);;
    pcl::toROSMsg(scan->lessFlatCloudRect, msg.less_flat_cloud_rect);

    pcl::toROSMsg(scan->outlierCloud, msg.outlier_cloud);;
    pcl::toROSMsg(scan->outlierCloudOdom, msg.outlier_cloud_odom);
    pcl::toROSMsg(scan->outlierCloudRect, msg.outlier_cloud_rect);;

    msg.pose_imu = sophusToPoseMsg(scan->poseIMU);
    msg.pose_odom = sophusToPoseMsg(scan->poseOdom);
    msg.pose_mapp = sophusToPoseMsg(scan->poseMapp);
    msg.scan_id = 0;
    msg.key_scan_id = 0;
    msg.state = ScanROS::STATE_KEYSCAN;

    return msg;
}

ScanPCL::Ptr importFromRosMsg(ScanROS msg) {
    ScanPCL::Ptr scan(new ScanPCL);
    scan->timestamp = msg.timestamp;
    pcl::fromROSMsg(msg.raw_scan, scan->rawScan);
    pcl::fromROSMsg(msg.less_sharp_cloud, scan->lessSharpCloud);
    pcl::fromROSMsg(msg.less_sharp_cloud_odom, scan->lessSharpCloudOdom);
    pcl::fromROSMsg(msg.less_sharp_cloud_rect, scan->lessSharpCloudRect);
    pcl::fromROSMsg(msg.less_flat_cloud, scan->lessFlatCloud);
    pcl::fromROSMsg(msg.less_flat_cloud_odom, scan->lessFlatCloudOdom);
    pcl::fromROSMsg(msg.less_flat_cloud_rect, scan->lessFlatCloudRect);
    pcl::fromROSMsg(msg.outlier_cloud, scan->outlierCloud);
    pcl::fromROSMsg(msg.outlier_cloud_odom, scan->outlierCloudOdom);
    pcl::fromROSMsg(msg.outlier_cloud_rect, scan->outlierCloudRect);

    poseMsgToSophus(msg.pose_imu, scan->poseIMU);
    poseMsgToSophus(msg.pose_odom, scan->poseOdom);
    poseMsgToSophus(msg.pose_mapp, scan->poseMapp);
    scan->scanID = 0;
    scan->keyScanID = 0;
    scan->state = ScanPCL::KEYSCAN;

    return scan;
}
