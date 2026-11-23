#include <rclcpp/version.h>
#include <rcutils/logging.h>

#include "picar_arm_safety/node.h"

// --- Main ---
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Suppress ROS timer logging
#if RCLCPP_VERSION_MAJOR >= 32
    const char *logger_name = "rclcpp";
#else
    const char *logger_name = "rcl";
#endif
    if (rcutils_logging_set_logger_level(logger_name, RCUTILS_LOG_SEVERITY_WARN) != RCUTILS_RET_OK) {
        RCLCPP_WARN(rclcpp::get_logger(logger_name), "Failed to set logger level for '%s'", logger_name);
    }

    auto node = std::make_shared<picar_arm_safety::PicarArmSafetyNode>();
    RCLCPP_INFO(node->get_logger(), "Using SingleThreadedExecutor.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
