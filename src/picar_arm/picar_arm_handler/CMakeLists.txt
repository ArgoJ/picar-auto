cmake_minimum_required(VERSION 3.20)
project(picar_arm_handler)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(picar_arm_safety_interface REQUIRED)

add_executable(arm_handler_node
    src/arm_handler_node.cpp
)

target_include_directories(arm_handler_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(arm_handler_node
    rclcpp::rclcpp
    std_msgs::std_msgs
    geometry_msgs::geometry_msgs
    picar_arm_safety_interface::picar_arm_safety_interface
)

install(TARGETS
    arm_handler_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    include/
    DESTINATION include
)

ament_package()
