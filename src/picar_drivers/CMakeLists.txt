cmake_minimum_required(VERSION 3.20)
project(picar_drivers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(COMMON_DEPENDENCIES
    rclcpp::rclcpp
    std_msgs::std_msgs
    sensor_msgs::sensor_msgs
    geometry_msgs::geometry_msgs
)

# 1. LED Node
add_executable(led_node src/led_node.cpp)
target_include_directories(led_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(led_node
    ${COMMON_DEPENDENCIES}
    std_srvs::std_srvs
    gpiodcxx
    gpiod
)

# 2. Buzzer Node
add_executable(buzzer_node src/buzzer_node.cpp)
target_include_directories(buzzer_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(buzzer_node
    ${COMMON_DEPENDENCIES}
    lgpio
)

# 3. Ultrasonic Node
add_executable(ultrasonic_node src/ultrasonic_node.cpp)
target_include_directories(ultrasonic_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(ultrasonic_node
    ${COMMON_DEPENDENCIES}
    gpiodcxx
    gpiod
)

# 4. OLED Node
add_executable(oled_node
    src/oled_node.cpp
    src/ssd1306_display.cpp
)
target_include_directories(oled_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(oled_node
    ${COMMON_DEPENDENCIES}
    z
)

# 5. Motor Node (DC-Antriebsmotoren)
add_executable(motor_node
    src/motor_node.cpp
    src/pca9685.cpp
)
target_include_directories(motor_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(motor_node
    ${COMMON_DEPENDENCIES}
    i2c
)

# 6. Servo Node (Lenkung, Arm, Greifer)
add_executable(servo_node
    src/servo_node.cpp
    src/pca9685.cpp
)
target_include_directories(servo_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(servo_node
    ${COMMON_DEPENDENCIES}
    i2c
)

# 7. MPU6050 IMU Node
add_executable(mpu6050_node
    src/mpu6050_node.cpp
    src/mpu6050_sensor.cpp
)
target_include_directories(mpu6050_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(mpu6050_node
    ${COMMON_DEPENDENCIES}
    i2c
)

# 8. Battery Node (ADS7830 ADC Akkustatus)
add_executable(battery_node
    src/battery_node.cpp
    src/ads7830_sensor.cpp
)
target_include_directories(battery_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(battery_node
    ${COMMON_DEPENDENCIES}
    i2c
)

install(TARGETS
    led_node
    buzzer_node
    ultrasonic_node
    oled_node
    motor_node
    servo_node
    mpu6050_node
    battery_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_led_node test/test_led_node.py)
    ament_add_pytest_test(test_buzzer_node test/test_buzzer_node.py)
    ament_add_pytest_test(test_ultrasonic_node test/test_ultrasonic_node.py)
    ament_add_pytest_test(test_oled_node test/test_oled_node.py)
    ament_add_pytest_test(test_mpu6050_node test/test_mpu6050_node.py)
    ament_add_pytest_test(test_motor_node test/test_motor_node.py)
    ament_add_pytest_test(test_servo_node test/test_servo_node.py)
    ament_add_pytest_test(test_battery_node test/test_battery_node.py)
endif()

ament_package()
