cmake_minimum_required(VERSION 3.20)
project(picar_arm_mpc)
project(picar_arm_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
