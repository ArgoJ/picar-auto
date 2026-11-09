#pragma once

#include <array>
#include <cmath>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <picar_arm_safety_interface/msg/control.hpp>
#include <picar_arm_safety_interface/msg/parameters.hpp>
#include <picar_arm_safety_interface/msg/references.hpp>
#include <picar_arm_safety_interface/msg/state.hpp>

namespace picar_arm_handler {

class ArmHandlerNode : public rclcpp::Node {
public:
    ArmHandlerNode();
    ~ArmHandlerNode() override = default;

private:
    // Setup & Parameter-Handling
    void init_parameters();
    void create_subscribers();
    void create_publishers();
    rcl_interfaces::msg::SetParametersResult on_parameter_update(
        const std::vector<rclcpp::Parameter> &params);

    // Callbacks
    void cmdPointCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void cmdPoseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void cmdGripperCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void armControlCallback(const picar_arm_safety_interface::msg::Control::SharedPtr msg);

    // Regelung & Kinematik
    void controlLoop();
    bool solveIK(double x, double y, double z, std::array<double, 3> &q_out);

    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_cmd_point_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_cmd_pose_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_cmd_gripper_;
    rclcpp::Subscription<picar_arm_safety_interface::msg::Control>::SharedPtr sub_arm_control_;

    // Publisher an Acados MPC Node
    rclcpp::Publisher<picar_arm_safety_interface::msg::State>::SharedPtr pub_arm_state_;
    rclcpp::Publisher<picar_arm_safety_interface::msg::References>::SharedPtr pub_arm_ref_;
    rclcpp::Publisher<picar_arm_safety_interface::msg::Parameters>::SharedPtr pub_arm_params_;

    // Publisher an Servo-Treiber (motor_node)
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_servo_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_gripper_hw_;

    // Timer & Parameter-Callback
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Arm-Zustand & Referenzen (in rad bzw. rad/s)
    std::array<double, 3> q_{0.0, 0.0, 0.0};
    std::array<double, 3> v_{0.0, 0.0, 0.0};
    std::array<double, 3> u_{0.0, 0.0, 0.0};
    std::array<double, 3> q_ref_{0.0, 0.0, 0.0};

    // Greifer-Zustand
    double gripper_deg_{67.0};
    double q4_rad_{0.0};

    // Zuletzt gesendete Hardware-Winkel
    std::array<double, 3> last_sent_angles_{-999.0, -999.0, -999.0};
    double last_sent_gripper_{-999.0};

    // Feste Kinematik- & Modellkonstanten (identisch zu arm_mpc.py)
    static constexpr double kTauV = 0.08;        // PT1-Zeitkonstante [s]
    static constexpr double kShoulderX = 0.08;   // Schulter x [m]
    static constexpr double kShoulderY = 0.0;    // Schulter y [m]
    static constexpr double kShoulderZ = 0.175;  // Schulter z [m]
    static constexpr double kL2 = 0.106;         // Oberarm [m]
    static constexpr double kL3 = 0.109;         // Unterarm [m]
    static constexpr double kL4 = 0.075;         // Greifer [m]

    // Dynamisch konfigurierbare ROS 2 Parameter
    double dt_{0.05};             // Zykluszeit [s] (20 Hz)
    double center_deg_{90.0};     // Servo-Mittelstellung [Grad]
    double gripper_min_{67.0};    // Greifer geschlossen [Grad]
    double gripper_max_{130.0};   // Greifer offen [Grad]

    // Chassis-Box Parameter [m] (werden zur Laufzeit im Vektor p an den MPC gesendet)
    double box_xc_{0.0};
    double box_yc_{0.0};
    double box_zc_{0.06};
    double box_dx_{0.12};
    double box_dy_{0.05};
    double box_dz_{0.07};
};

}  // namespace picar_arm_handler
