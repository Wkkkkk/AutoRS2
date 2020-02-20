cmake_minimum_required(VERSION 3.5)
project(driver)

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(PCL CONFIG REQUIRED)
find_package(pugixml CONFIG REQUIRED)

include_directories(
        include
        ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
        src/pointcloud_transformer.cpp)
target_compile_definitions(${PROJECT_NAME}
        PRIVATE "MINIMAL_COMPOSITION_DLL")
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} pugixml)
ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_components sensor_msgs)

add_executable(driver_test src/driver_test.cpp)
target_link_libraries(driver_test driver)
ament_target_dependencies(driver_test rclcpp)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".git" EXCLUDE)

install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(TARGETS
        driver_test
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
        )

#ament_export_interfaces(${PROJECT_NAME})
#ament_export_dependencies(Eigen3 PCL)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

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
