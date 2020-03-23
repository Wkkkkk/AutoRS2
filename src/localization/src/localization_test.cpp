//
// Created by zhihui on 1/17/20.
//

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "localization/private/data_synchronizer.h"
#include "localization/private/odometry.h"
#include "localization/localization.h"
#include "localization/init.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    // magic!
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(false);

    auto route = init();

    auto data_synchronizer = std::make_shared<DataSynchronizer>(options);
    auto odometry = std::make_shared<Odometry>(route, options);
    auto localization = std::make_shared<Localization>(route, options);
    exec.add_node(data_synchronizer);
    exec.add_node(odometry);
    exec.add_node(localization);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
