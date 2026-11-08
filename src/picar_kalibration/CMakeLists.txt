cmake_minimum_required(VERSION 3.20)
project(picar_kalibration)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/SetServoCalibration.srv"
    "srv/GetServoCalibration.srv"
    "srv/SaveCalibration.srv"
    DEPENDENCIES std_msgs
)

install(PROGRAMS
    scripts/kalibration_service_node.py
    scripts/calibrate_cli.py
    DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/kalibration_service_node.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME kalibration_service_node
)

install(PROGRAMS
    scripts/calibrate_cli.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME calibrate_cli
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_kalibration_service test/test_kalibration_service.py)
endif()

ament_package()
