cmake_minimum_required(VERSION 3.5)
project(localization)

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(third_party_dir /home/zhihui/AutoRS/LIBS)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
#ros
find_package(roscpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message REQUIRED)
find_package(driver REQUIRED)

#others
find_package(yaml-cpp CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(PCL CONFIG REQUIRED)
find_package(Ceres CONFIG REQUIRED)
find_package(glog CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(PROJ4 CONFIG REQUIRED)
find_package(pugixml CONFIG REQUIRED)

include_directories(
        include
        ${PCL_INCLUDE_DIRS}
        ${third_party_dir}/3rdparty/include
        ${third_party_dir}/LINSLocalization/include
)

link_directories(
        ${third_party_dir}/3rdparty/lib
        ${third_party_dir}/LINSLocalization/lib
        /home/zhihui/workspace/library/boost_1_58_0/lib
)

add_library(localization_nodes SHARED
        src/localization.cpp
        src/gps_filter.cpp
        src/data_synchronizer.cpp
        src/odometry.cpp
        src/scan_transform.cpp
        src/init.cpp
        )
target_compile_definitions(localization_nodes
        PRIVATE "MINIMAL_COMPOSITION_DLL")
ament_target_dependencies(localization_nodes rclcpp rclcpp_components sensor_msgs geometry_msgs message driver)
target_link_libraries(localization_nodes
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        Eigen3::Eigen
        yaml-cpp
        pugixml
        proj
        common
        ${BOOST_LIBRARIES}
        matcher
        msf_module
        glog::glog
        ceres
        )
message(STATUS ">>>>>>>>>>>>>FOUND PCL: ${PCL_LIBRARIES}")


add_executable(localization src/localization_test.cpp)
ament_target_dependencies(localization rclcpp)
target_link_libraries(localization
        localization_nodes
        )

install(TARGETS
        localization_nodes
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS
        localization
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
        )

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
