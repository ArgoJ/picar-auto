#include "picar_arm_handler/arm_handler_node.hpp"

#include <algorithm>
#include <chrono>

namespace picar_arm_handler {

namespace {
constexpr double degToRad(double deg) {
    return deg * M_PI / 180.0;
}

constexpr double radToDeg(double rad) {
    return rad * 180.0 / M_PI;
}
}  // namespace

ArmHandlerNode::ArmHandlerNode()
    : Node("arm_handler_node")
{
    this->init_parameters();
    this->create_subscribers();
    this->create_publishers();

    // --- Control Timer ---
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(dt_));
    timer_ = this->create_wall_timer(
        period_ns,
        std::bind(&ArmHandlerNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "C++ ArmHandlerNode gestartet (dt = %.3f s).", dt_);
}

void ArmHandlerNode::init_parameters() {
    this->declare_parameter("dt", dt_);
    this->declare_parameter("center_deg", center_deg_);
    this->declare_parameter("gripper_min_deg", gripper_min_);
    this->declare_parameter("gripper_max_deg", gripper_max_);

    this->declare_parameter("box_xc", box_xc_);
    this->declare_parameter("box_yc", box_yc_);
    this->declare_parameter("box_zc", box_zc_);
    this->declare_parameter("box_dx", box_dx_);
    this->declare_parameter("box_dy", box_dy_);
    this->declare_parameter("box_dz", box_dz_);

    dt_          = this->get_parameter("dt").as_double();
    center_deg_  = this->get_parameter("center_deg").as_double();
    gripper_min_ = this->get_parameter("gripper_min_deg").as_double();
    gripper_max_ = this->get_parameter("gripper_max_deg").as_double();

    box_xc_      = this->get_parameter("box_xc").as_double();
    box_yc_      = this->get_parameter("box_yc").as_double();
    box_zc_      = this->get_parameter("box_zc").as_double();
    box_dx_      = this->get_parameter("box_dx").as_double();
    box_dy_      = this->get_parameter("box_dy").as_double();
    box_dz_      = this->get_parameter("box_dz").as_double();

    gripper_deg_ = gripper_min_;
    q4_rad_      = 0.0;

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ArmHandlerNode::on_parameter_update, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult ArmHandlerNode::on_parameter_update(
    const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "OK";

    for (const auto &param : params) {
        const auto &name = param.get_name();
        if (name == "dt") {
            const double new_dt = param.as_double();
            if (new_dt <= 0.0) {
                result.successful = false;
                result.reason = "dt muss groesser als 0 sein.";
                return result;
            }
            dt_ = new_dt;
            if (timer_) {
                timer_->cancel();
                const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(dt_));
                timer_ = this->create_wall_timer(
                    period_ns,
                    std::bind(&ArmHandlerNode::controlLoop, this));
            }
        } else if (name == "center_deg") {
            center_deg_ = param.as_double();
        } else if (name == "gripper_min_deg") {
            gripper_min_ = param.as_double();
        } else if (name == "gripper_max_deg") {
            gripper_max_ = param.as_double();
        } else if (name == "box_xc") {
            box_xc_ = param.as_double();
        } else if (name == "box_yc") {
            box_yc_ = param.as_double();
        } else if (name == "box_zc") {
            box_zc_ = param.as_double();
        } else if (name == "box_dx") {
            box_dx_ = param.as_double();
        } else if (name == "box_dy") {
            box_dy_ = param.as_double();
        } else if (name == "box_dz") {
            box_dz_ = param.as_double();
        }
    }

    return result;
}

void ArmHandlerNode::create_subscribers() {
    sub_cmd_point_ = this->create_subscription<geometry_msgs::msg::Point>(
        "picar/arm/cmd_point", 10,
        std::bind(&ArmHandlerNode::cmdPointCallback, this, std::placeholders::_1));

    sub_cmd_pose_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "picar/arm/cmd_pose", 10,
        std::bind(&ArmHandlerNode::cmdPoseCallback, this, std::placeholders::_1));

    sub_cmd_gripper_ = this->create_subscription<std_msgs::msg::Float32>(
        "picar/arm/gripper_cmd", 10,
        std::bind(&ArmHandlerNode::cmdGripperCallback, this, std::placeholders::_1));

    sub_arm_control_ = this->create_subscription<picar_arm_safety_interface::msg::Control>(
        "arm_control", 10,
        std::bind(&ArmHandlerNode::armControlCallback, this, std::placeholders::_1));
}

void ArmHandlerNode::create_publishers() {
    pub_arm_state_ = this->create_publisher<picar_arm_safety_interface::msg::State>("arm_state", 10);
    pub_arm_ref_ = this->create_publisher<picar_arm_safety_interface::msg::References>("arm_reference", 10);
    pub_arm_params_ = this->create_publisher<picar_arm_safety_interface::msg::Parameters>("arm_params", 10);

    pub_servo_cmd_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("picar/servo/cmd", 10);
    pub_gripper_hw_ = this->create_publisher<std_msgs::msg::Float32>("picar/gripper/cmd", 10);
}

bool ArmHandlerNode::solveIK(double x, double y, double z, std::array<double, 3> &q_out) {
    const double dx = x - kShoulderX;
    const double dy = y - kShoulderY;
    const double dz = z - kShoulderZ;

    // 1. Basis-Azimut q1
    const double q1 = std::atan2(dy, dx);
    if (q1 < -1.4 || q1 > 1.4) {
        return false;
    }

    // 2. Abstand im vertikalen Schnitt
    const double r = std::hypot(dx, dy);
    const double d_sq = r * r + dz * dz;
    const double dist = std::sqrt(d_sq);

    const double l1 = kL2;
    const double l2 = kL3 + kL4 * std::cos(q4_rad_);

    if (dist > (l1 + l2) || dist < std::abs(l1 - l2)) {
        return false;
    }

    // 3. Kosinussatz fuer q3 (Elbow-Up: sin_q3 <= 0)
    double cos_q3 = (d_sq - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
    cos_q3 = std::clamp(cos_q3, -1.0, 1.0);
    const double sin_q3_mag = std::sqrt(std::max(0.0, 1.0 - cos_q3 * cos_q3));

    const double alpha = std::atan2(dz, r);
    for (double sign : {-1.0, 1.0}) {
        const double sin_q3 = sign * sin_q3_mag;
        const double q3 = std::atan2(sin_q3, cos_q3);
        const double beta = std::atan2(l2 * sin_q3, l1 + l2 * cos_q3);
        const double q2 = alpha - beta;

        if (q2 >= -0.8 && q2 <= 1.6 && q3 >= -1.6 && q3 <= 1.6) {
            q_out = {q1, q2, q3};
            return true;
        }
    }

    return false;
}

void ArmHandlerNode::cmdPointCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    std::array<double, 3> q_sol{};
    if (solveIK(msg->x, msg->y, msg->z, q_sol)) {
        q_ref_ = q_sol;
        RCLCPP_INFO(
            this->get_logger(),
            "Ziel (%.2f, %.2f, %.2f)m -> Servos [Pan=%.1f, Tilt=%.1f, Lift=%.1f] Grad",
            msg->x, msg->y, msg->z,
            center_deg_ + radToDeg(q_ref_[0]),
            center_deg_ + radToDeg(q_ref_[1]),
            center_deg_ + radToDeg(q_ref_[2]));
    } else {
        RCLCPP_WARN(
            this->get_logger(),
            "Ziel (%.2f, %.2f, %.2f)m ausserhalb der Reichweite oder Gelenkgrenzen.",
            msg->x, msg->y, msg->z);
    }
}

void ArmHandlerNode::cmdPoseCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 3) {
        RCLCPP_WARN(this->get_logger(), "cmd_pose erwartet mind. 3 Werte: [pan, tilt, lift] in Grad.");
        return;
    }

    q_ref_[0] = degToRad(static_cast<double>(msg->data[0]) - center_deg_);
    q_ref_[1] = degToRad(static_cast<double>(msg->data[1]) - center_deg_);
    q_ref_[2] = degToRad(static_cast<double>(msg->data[2]) - center_deg_);

    if (msg->data.size() >= 4) {
        gripper_deg_ = std::clamp(static_cast<double>(msg->data[3]), gripper_min_, gripper_max_);
        q4_rad_ = degToRad(gripper_deg_ - gripper_min_);
    }
}

void ArmHandlerNode::cmdGripperCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    gripper_deg_ = std::clamp(static_cast<double>(msg->data), gripper_min_, gripper_max_);
    q4_rad_ = degToRad(gripper_deg_ - gripper_min_);
}

void ArmHandlerNode::armControlCallback(const picar_arm_safety_interface::msg::Control::SharedPtr msg) {
    u_[0] = msg->u[0];
    u_[1] = msg->u[1];
    u_[2] = msg->u[2];
}

void ArmHandlerNode::controlLoop() {
    const auto now = this->get_clock()->now();

    // 1. Parameter (Greifer + Chassis-Box) an MPC senden
    picar_arm_safety_interface::msg::Parameters param_msg;
    param_msg.header.stamp = now;
    param_msg.p = {q4_rad_, box_xc_, box_yc_, box_zc_, box_dx_, box_dy_, box_dz_};
    pub_arm_params_->publish(param_msg);

    // 2. Referenzwinkel an MPC senden
    picar_arm_safety_interface::msg::References ref_msg;
    ref_msg.header.stamp = now;
    ref_msg.yref_0 = {q_ref_[0], q_ref_[1], q_ref_[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ref_msg.yref = ref_msg.yref_0;
    ref_msg.yref_e = {q_ref_[0], q_ref_[1], q_ref_[2]};
    pub_arm_ref_->publish(ref_msg);

    // 3. Aktuellen Zustand an MPC senden
    picar_arm_safety_interface::msg::State state_msg;
    state_msg.header.stamp = now;
    state_msg.x = {q_[0], q_[1], q_[2], v_[0], v_[1], v_[2]};
    pub_arm_state_->publish(state_msg);

    // 4. Zustand mit sicherer Stellgroesse u vorwaertsintegrieren
    for (size_t i = 0; i < 3; ++i) {
        v_[i] += ((u_[i] - v_[i]) / kTauV) * dt_;
        q_[i] += v_[i] * dt_;
    }

    // 5. Sichere Winkel an motor_node senden (CH1=Pan, CH2=Tilt, CH3=Lift)
    for (size_t i = 0; i < 3; ++i) {
        const double angle_deg = center_deg_ + radToDeg(q_[i]);
        if (std::abs(angle_deg - last_sent_angles_[i]) >= 0.1) {
            std_msgs::msg::Float32MultiArray cmd;
            cmd.data = {static_cast<float>(i + 1), static_cast<float>(angle_deg)};
            pub_servo_cmd_->publish(cmd);
            last_sent_angles_[i] = angle_deg;
        }
    }

    // 6. Greifer senden (CH4)
    if (std::abs(gripper_deg_ - last_sent_gripper_) >= 0.5) {
        std_msgs::msg::Float32 g_cmd;
        g_cmd.data = static_cast<float>(gripper_deg_);
        pub_gripper_hw_->publish(g_cmd);
        last_sent_gripper_ = gripper_deg_;
    }
}

}  // namespace picar_arm_handler

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<picar_arm_handler::ArmHandlerNode>());
    rclcpp::shutdown();
    return 0;
}
