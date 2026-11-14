#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

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
    struct ScalarParam {
        const char *name;
        double *target;
        bool strictly_positive;
        double min_val;
    };

    std::array<ScalarParam, 18> scalarParams();
    void initParameters();
    void setupInterfaces();
    rcl_interfaces::msg::SetParametersResult onParameterUpdate(
        const std::vector<rclcpp::Parameter> &params);

    void cmdPointCallback(const geometry_msgs::msg::Point::ConstSharedPtr &msg);
    void cmdPoseCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr &msg);
    void cmdTwistCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg);

    void controlLoop();
    bool handleShutdownLoop(const rclcpp::Time &now);
    void publishServoCommands();
    void commandDefaultPose();
    void applyTwistStep(const rclcpp::Time &now);
    void setGripperAngle(double target_deg);

    bool solveBoundedCartesian(
        const std::array<double, 3> &candidate,
        std::array<double, 3> &adjusted_out,
        std::array<double, 3> &q_out) const;
    bool tryApplyCartesianTarget(const std::array<double, 3> &candidate);
    bool tryArcSlideTarget(
        double phi_new, double r_target, double z_target, double v_r, double v_z);

    std::array<double, 3> computeFK(const std::array<double, 3> &q) const;
    bool solveIK(double x, double y, double z, std::array<double, 3> &q_out) const;
    double degToJointRad(std::size_t idx, double servo_deg) const;
    double jointRadToDeg(std::size_t idx, double q_rad) const;
    double defaultGripperDeg() const;
    double forearmLength() const;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_cmd_point_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_cmd_pose_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_twist_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_cmd_gripper_;
    rclcpp::Subscription<picar_arm_safety_interface::msg::Control>::SharedPtr sub_arm_control_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_gripper_toggle_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_home_;

    rclcpp::Publisher<picar_arm_safety_interface::msg::State>::SharedPtr pub_arm_state_;
    rclcpp::Publisher<picar_arm_safety_interface::msg::References>::SharedPtr pub_arm_ref_;
    rclcpp::Publisher<picar_arm_safety_interface::msg::Parameters>::SharedPtr pub_arm_params_;
    std::array<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr, 3> pub_arm_joints_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_gripper_cmd_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    std::array<double, 3> q_{0.0, 0.0, 0.0};
    std::array<double, 3> v_{0.0, 0.0, 0.0};
    std::array<double, 3> u_{0.0, 0.0, 0.0};
    std::array<double, 3> q_ref_{0.0, 0.0, 0.0};
    std::array<double, 3> target_pos_{0.0, 0.0, 0.0};

    geometry_msgs::msg::Twist active_twist_{};
    rclcpp::Time last_twist_time_{0, 0, RCL_ROS_TIME};
    bool has_twist_{false};
    bool twist_moving_{false};

    double gripper_deg_{67.0};
    double q4_rad_{0.0};
    bool gripper_open_{false};

    std::array<double, 3> last_sent_angles_{-999.0, -999.0, -999.0};
    double last_sent_gripper_{-999.0};

    // Feste Kinematik- & Modellkonstanten (abgestimmt auf arm_mpc.py)
    static constexpr double kTauV = 0.08;
    static constexpr double kShoulderX = 0.08;
    static constexpr double kShoulderY = 0.0;
    static constexpr double kShoulderZ = 0.175;
    static constexpr double kL2 = 0.106;
    static constexpr double kL3 = 0.109;
    static constexpr double kL4 = 0.075;

    double dt_{0.05};
    double gripper_min_{67.0};
    double gripper_max_{130.0};

    // Default-Pose [Grad] und Kalibrierung zwischen Servo-Winkeln und geometrischen Radiant
    std::vector<double> default_pose_deg_{90.0, 60.0, 30.0, 67.0};
    std::vector<double> joint_zero_deg_{90.0, 180.0, 90.0};
    std::vector<double> joint_sign_{1.0, -1.0, 1.0};
    std::vector<double> q_min_rad_{-1.39, 0.07, -1.48};
    std::vector<double> q_max_rad_{1.39, 2.35, 1.39};

    bool move_to_default_on_shutdown_{true};
    double shutdown_max_vel_{1.0};
    double shutdown_pid_kp_{4.0};
    double shutdown_pid_ki_{0.1};
    double shutdown_pid_kd_{0.15};
    double shutdown_timeout_s_{4.0};
    bool shutting_down_{false};
    bool shutdown_finalized_{false};
    rclcpp::Time shutdown_start_time_{0, 0, RCL_ROS_TIME};
    std::array<double, 3> shutdown_err_int_{0.0, 0.0, 0.0};

    double twist_timeout_s_{0.25};
    double twist_linear_scale_{1.0};
    double twist_angular_scale_{1.0};
    double max_ik_lead_rad_{0.45};

    double box_xc_{0.0};
    double box_yc_{0.0};
    double box_zc_{0.06};
    double box_dx_{0.12};
    double box_dy_{0.05};
    double box_dz_{0.07};
};

}  // namespace picar_arm_handler
