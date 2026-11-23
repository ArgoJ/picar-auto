cmake_minimum_required(VERSION 3.16)
project(picar_arm_safety)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.27)
    cmake_policy(SET CMP0148 OLD)
endif()

set(ACADOS_WARN_FLAGS)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(ACADOS_WARN_FLAGS -Wall -Wextra -Wpedantic)
endif()

# --- ROS DEPENDENCIES ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(picar_arm_safety_interface REQUIRED)

# --- ACADOS PATHS ---
set(ACADOS_INCLUDE_PATH /home/pirob/acados/include)
set(ACADOS_LIB_DIR /home/pirob/acados/lib)
set(ACADOS_GENERATED_CODE_DIR /home/pirob/ros2_ws/src/picar_arm/picar_arm_safety/c_generated_code)

# --- NODE LIBRARY ---
add_library(picar_arm_safety_node_lib STATIC
    src/node.cpp
)

# --- TARGET CONFIGURATION ---
target_compile_options(picar_arm_safety_node_lib PRIVATE ${ACADOS_WARN_FLAGS})
target_include_directories(picar_arm_safety_node_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${ACADOS_GENERATED_CODE_DIR}
    ${ACADOS_INCLUDE_PATH}
    ${ACADOS_INCLUDE_PATH}/acados
    ${ACADOS_INCLUDE_PATH}/blasfeo/include
    ${ACADOS_INCLUDE_PATH}/hpipm/include
)
target_link_libraries(picar_arm_safety_node_lib
    ${ACADOS_GENERATED_CODE_DIR}/libacados_ocp_solver_ocp_picar_arm_safety_filter_5c21b6cb.so
    ${ACADOS_LIB_DIR}/libacados.so
    ${ACADOS_LIB_DIR}/libblasfeo.so
    ${ACADOS_LIB_DIR}/libhpipm.so
    m
)

# --- DEPENDENCIES ---
if(COMMAND ament_target_dependencies)
    ament_target_dependencies(picar_arm_safety_node_lib
        rclcpp
        std_msgs
        picar_arm_safety_interface
    )
else()
    target_link_libraries(picar_arm_safety_node_lib
        rclcpp::rclcpp
        std_msgs::std_msgs
        picar_arm_safety_interface::picar_arm_safety_interface
    )
endif()

add_executable(picar_arm_safety_node 
    src/main.cpp
)
target_compile_options(picar_arm_safety_node PRIVATE
    ${ACADOS_WARN_FLAGS}
)
target_link_libraries(picar_arm_safety_node
    picar_arm_safety_node_lib
)

# --- INSTALLATIONS ---
install(TARGETS 
    picar_arm_safety_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
    include/
    DESTINATION include
)

install(FILES 
    ${ACADOS_GENERATED_CODE_DIR}/libacados_ocp_solver_ocp_picar_arm_safety_filter_5c21b6cb.so
    DESTINATION lib
)

# --- EXPORT HOOKS ---
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/export_acados_path.sh.in")

# --- TESTS ---
if(BUILD_TESTING)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(
        test/test_picar_arm_safety.launch.py
        TARGET test_picar_arm_safety
        TIMEOUT 180
    )

    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(gtest_picar_arm_safety
        test/test_picar_arm_safety.cpp
    )
    if(TARGET gtest_picar_arm_safety)
        target_compile_options(gtest_picar_arm_safety PRIVATE
            ${ACADOS_WARN_FLAGS}
        )
        target_link_libraries(gtest_picar_arm_safety
            picar_arm_safety_node_lib
        )
    endif()
endif()

ament_package()