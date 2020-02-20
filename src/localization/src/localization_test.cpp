//
// Created by zhihui on 1/17/20.
//

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "localization/private/data_synchronizer.h"
#include "localization/localization.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    // magic!
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    auto publisher_node = std::make_shared<DataSynchronizer>(options);
    auto subscriber_node = std::make_shared<Localization>(options);
    exec.add_node(publisher_node);
    exec.add_node(subscriber_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}