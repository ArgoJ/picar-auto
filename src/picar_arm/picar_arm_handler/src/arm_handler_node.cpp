#include "picar_arm_handler/arm_handler_node.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <limits>
#include <string>
#include <utility>

namespace picar_arm_handler {

namespace {
std::atomic<int> g_arm_signal_count{0};

constexpr double degToRad(double deg) { return deg * M_PI / 180.0; }
constexpr double radToDeg(double rad) { return rad * 180.0 / M_PI; }

void armHandlerSignalHandler(int sig) {
    (void)sig;
    if (g_arm_signal_count.fetch_add(1) >= 1) {
        rclcpp::shutdown();
    }
}

bool assignVec3(
    const std::vector<double> &src, std::vector<double> &dst,
    const char *name, bool nonzero, std::string &err_out)
{
    if (src.size() != 3) {
        err_out = std::string(name) + " braucht genau 3 Elemente.";
        return false;
    }
    if (nonzero && (std::abs(src[0]) < 1e-6 || std::abs(src[1]) < 1e-6 || std::abs(src[2]) < 1e-6)) {
        err_out = std::string(name) + " braucht 3 Elemente ungleich 0.";
        return false;
    }
    dst = src;
    return true;
}

constexpr double kNoFloor = -std::numeric_limits<double>::infinity();
}  // namespace

ArmHandlerNode::ArmHandlerNode() : Node("arm_handler_node") {
    last_twist_time_ = this->get_clock()->now();
    shutdown_start_time_ = last_twist_time_;
    initParameters();
    setupInterfaces();

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(dt_));
    timer_ = this->create_wall_timer(period, [this]() { controlLoop(); });

    RCLCPP_INFO(this->get_logger(), "ArmHandlerNode gestartet (dt = %.3f s).", dt_);
}

std::array<ArmHandlerNode::ScalarParam, 18> ArmHandlerNode::scalarParams() {
    return {{
        {"dt", &dt_, true, 1e-3},
        {"gripper_min_deg", &gripper_min_, false, kNoFloor},
        {"gripper_max_deg", &gripper_max_, false, kNoFloor},
        {"shutdown_max_vel_rad_s", &shutdown_max_vel_, true, 0.05},
        {"shutdown_pid_kp", &shutdown_pid_kp_, false, 0.0},
        {"shutdown_pid_ki", &shutdown_pid_ki_, false, 0.0},
        {"shutdown_pid_kd", &shutdown_pid_kd_, false, 0.0},
        {"shutdown_timeout_s", &shutdown_timeout_s_, true, 0.1},
        {"twist_timeout_s", &twist_timeout_s_, true, 1e-3},
        {"twist_linear_scale", &twist_linear_scale_, false, kNoFloor},
        {"twist_angular_scale", &twist_angular_scale_, false, kNoFloor},
        {"max_ik_lead_rad", &max_ik_lead_rad_, true, 1e-3},
        {"box_xc", &box_xc_, false, kNoFloor},
        {"box_yc", &box_yc_, false, kNoFloor},
        {"box_zc", &box_zc_, false, kNoFloor},
        {"box_dx", &box_dx_, false, kNoFloor},
        {"box_dy", &box_dy_, false, kNoFloor},
        {"box_dz", &box_dz_, false, kNoFloor},
    }};
}

void ArmHandlerNode::initParameters() {
    for (const auto &spec : scalarParams()) {
        double val = this->declare_parameter(spec.name, *spec.target);
        *spec.target = std::isfinite(spec.min_val) ? std::max(spec.min_val, val) : val;
    }

    move_to_default_on_shutdown_ =
        this->declare_parameter("move_to_default_on_shutdown", move_to_default_on_shutdown_);

    default_pose_deg_ = this->declare_parameter("default_pose_deg", default_pose_deg_);
    if (default_pose_deg_.size() < 3) {
        RCLCPP_WARN(
            this->get_logger(),
            "Parameter 'default_pose_deg' hat %zu Elemente - Fallback [90, 60, 30, %.1f].",
            default_pose_deg_.size(), gripper_min_);
        default_pose_deg_ = {90.0, 60.0, 30.0, gripper_min_};
    }

    auto loadVec3 = [this](const char *name, std::vector<double> &dst, bool nonzero) {
        std::string err;
        auto val = this->declare_parameter(name, dst);
        if (!assignVec3(val, dst, name, nonzero, err)) {
            RCLCPP_WARN(this->get_logger(), "%s Verwende Defaults.", err.c_str());
        }
    };
    loadVec3("joint_zero_deg", joint_zero_deg_, false);
    loadVec3("joint_sign", joint_sign_, true);
    loadVec3("q_min_rad", q_min_rad_, false);
    loadVec3("q_max_rad", q_max_rad_, false);
    for (size_t i = 0; i < 3; ++i) {
        if (q_min_rad_[i] > q_max_rad_[i]) {
            std::swap(q_min_rad_[i], q_max_rad_[i]);
        }
    }

    for (size_t i = 0; i < 3; ++i) {
        q_[i] = degToJointRad(i, default_pose_deg_[i]);
    }
    q_ref_ = q_;
    setGripperAngle(defaultGripperDeg());
    target_pos_ = computeFK(q_ref_);

    param_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) { return onParameterUpdate(params); });
}

rcl_interfaces::msg::SetParametersResult ArmHandlerNode::onParameterUpdate(
    const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "OK";
    const auto specs = scalarParams();

    for (const auto &param : params) {
        const auto &name = param.get_name();
        if (name == "move_to_default_on_shutdown") {
            move_to_default_on_shutdown_ = param.as_bool();
            continue;
        }

        if (name == "joint_zero_deg" || name == "joint_sign" ||
            name == "q_min_rad" || name == "q_max_rad")
        {
            std::vector<double> *dst = (name == "joint_zero_deg") ? &joint_zero_deg_
                                     : (name == "joint_sign")     ? &joint_sign_
                                     : (name == "q_min_rad")      ? &q_min_rad_
                                                                  : &q_max_rad_;
            if (!assignVec3(param.as_double_array(), *dst, name.c_str(),
                            name == "joint_sign", result.reason))
            {
                result.successful = false;
                return result;
            }
            for (size_t i = 0; i < 3; ++i) {
                if (q_min_rad_[i] > q_max_rad_[i]) {
                    std::swap(q_min_rad_[i], q_max_rad_[i]);
                }
            }
            continue;
        }

        for (const auto &spec : specs) {
            if (name != spec.name) {
                continue;
            }
            double val = param.as_double();
            if (spec.strictly_positive && val <= 0.0) {
                result.successful = false;
                result.reason = name + " muss groesser als 0 sein.";
                return result;
            }
            if (!spec.strictly_positive && std::isfinite(spec.min_val)) {
                val = std::max(spec.min_val, val);
            }
            *spec.target = val;
            if (name == "dt" && timer_) {
                timer_->cancel();
                const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(dt_));
                timer_ = this->create_wall_timer(period, [this]() { controlLoop(); });
            }
            break;
        }
    }

    return result;
}

void ArmHandlerNode::setupInterfaces() {
    sub_cmd_point_ = this->create_subscription<geometry_msgs::msg::Point>(
        "picar/arm/cmd_point", 10,
        [this](geometry_msgs::msg::Point::ConstSharedPtr msg) { cmdPointCallback(msg); });
    sub_cmd_pose_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "picar/arm/cmd_pose", 10,
        [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) { cmdPoseCallback(msg); });
    sub_cmd_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "picar/arm/cmd_twist", 10,
        [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) { cmdTwistCallback(msg); });
    sub_cmd_gripper_ = this->create_subscription<std_msgs::msg::Float32>(
        "picar/arm/gripper_cmd", 10,
        [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
            setGripperAngle(static_cast<double>(msg->data));
            target_pos_ = computeFK(q_ref_);
        });
    sub_arm_control_ = this->create_subscription<picar_arm_safety_interface::msg::Control>(
        "arm_control", 10,
        [this](picar_arm_safety_interface::msg::Control::ConstSharedPtr msg) {
            u_ = {msg->u[0], msg->u[1], msg->u[2]};
        });

    srv_gripper_toggle_ = this->create_service<std_srvs::srv::Trigger>(
        "/gripper/toggle",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            setGripperAngle(gripper_open_ ? gripper_min_ : gripper_max_);
            target_pos_ = computeFK(q_ref_);
            res->success = true;
            res->message = gripper_open_ ? "Greifer geoeffnet." : "Greifer geschlossen.";
        });
    srv_home_ = this->create_service<std_srvs::srv::Trigger>(
        "/picar/arm/home",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            commandDefaultPose();
            res->success = true;
            res->message = "Fahre ueber Acados-MPC in Ruheposition.";
        });

    pub_arm_state_  = this->create_publisher<picar_arm_safety_interface::msg::State>("arm_state", 10);
    pub_arm_ref_    = this->create_publisher<picar_arm_safety_interface::msg::References>("arm_reference", 10);
    pub_arm_params_ = this->create_publisher<picar_arm_safety_interface::msg::Parameters>("arm_params", 10);

    pub_arm_joints_[0] = this->create_publisher<std_msgs::msg::Float32>("picar/servo/rotation", 10);
    pub_arm_joints_[1] = this->create_publisher<std_msgs::msg::Float32>("picar/servo/shoulder", 10);
    pub_arm_joints_[2] = this->create_publisher<std_msgs::msg::Float32>("picar/servo/elbow", 10);
    pub_gripper_cmd_   = this->create_publisher<std_msgs::msg::Float32>("picar/servo/gripper", 10);
}

double ArmHandlerNode::degToJointRad(std::size_t idx, double servo_deg) const {
    return degToRad((servo_deg - joint_zero_deg_[idx]) * joint_sign_[idx]);
}

double ArmHandlerNode::jointRadToDeg(std::size_t idx, double q_rad) const {
    return joint_zero_deg_[idx] + joint_sign_[idx] * radToDeg(q_rad);
}

double ArmHandlerNode::defaultGripperDeg() const {
    return (default_pose_deg_.size() >= 4) ? default_pose_deg_[3] : gripper_min_;
}

double ArmHandlerNode::forearmLength() const {
    return kL3 + kL4 * std::cos(q4_rad_);
}

std::array<double, 3> ArmHandlerNode::computeFK(const std::array<double, 3> &q) const {
    const double l2 = forearmLength();
    const double pitch = q[1] + q[2];
    const double r = kL2 * std::cos(q[1]) + l2 * std::cos(pitch);
    return {
        kShoulderX + r * std::cos(q[0]),
        kShoulderY + r * std::sin(q[0]),
        kShoulderZ + kL2 * std::sin(q[1]) + l2 * std::sin(pitch)
    };
}

bool ArmHandlerNode::solveIK(
    double x, double y, double z, std::array<double, 3> &q_out) const
{
    const double dx = x - kShoulderX;
    const double dy = y - kShoulderY;
    const double dz = z - kShoulderZ;

    const double r = std::hypot(dx, dy);
    const double q1 = (r > 1e-6) ? std::atan2(dy, dx) : q_ref_[0];
    if (q1 < q_min_rad_[0] || q1 > q_max_rad_[0]) {
        return false;
    }

    const double l1 = kL2;
    const double l2 = forearmLength();
    const double d_sq = r * r + dz * dz;
    const double dist = std::sqrt(d_sq);
    if (dist > (l1 + l2) || dist < std::abs(l1 - l2)) {
        return false;
    }

    const double cos_q3 = std::clamp((d_sq - l1 * l1 - l2 * l2) / (2.0 * l1 * l2), -1.0, 1.0);
    const double sin_q3_mag = std::sqrt(std::max(0.0, 1.0 - cos_q3 * cos_q3));
    const double alpha = std::atan2(dz, r);

    bool found = false;
    double best_cost = std::numeric_limits<double>::infinity();
    for (double sign : {-1.0, 1.0}) {
        const double sin_q3 = sign * sin_q3_mag;
        const double q3 = std::atan2(sin_q3, cos_q3);
        const double q2 = alpha - std::atan2(l2 * sin_q3, l1 + l2 * cos_q3);

        if (q2 < q_min_rad_[1] || q2 > q_max_rad_[1] ||
            q3 < q_min_rad_[2] || q3 > q_max_rad_[2])
        {
            continue;
        }
        // Leichter Bias gegen Elbow-Down, damit senkrechtes Absenken aus [90,90,90] nach vorne faltet
        const double cost = std::abs(q2 - q_[1]) + std::abs(q3 - q_[2]) + ((sign > 0.0) ? 0.15 : 0.0);
        if (cost < best_cost) {
            best_cost = cost;
            q_out = {q1, q2, q3};
            found = true;
        }
    }
    return found;
}

void ArmHandlerNode::cmdPointCallback(const geometry_msgs::msg::Point::ConstSharedPtr &msg) {
    std::array<double, 3> q_sol{};
    if (!solveIK(msg->x, msg->y, msg->z, q_sol)) {
        RCLCPP_WARN(
            this->get_logger(),
            "Ziel (%.2f, %.2f, %.2f)m ausserhalb der Reichweite oder Gelenkgrenzen.",
            msg->x, msg->y, msg->z);
        return;
    }
    q_ref_ = q_sol;
    target_pos_ = {msg->x, msg->y, msg->z};
    has_twist_ = false;
    twist_moving_ = false;
    RCLCPP_INFO(
        this->get_logger(),
        "Ziel (%.2f, %.2f, %.2f)m -> Servos [Rot=%.1f, Shoulder=%.1f, Elbow=%.1f] Grad",
        msg->x, msg->y, msg->z,
        jointRadToDeg(0, q_ref_[0]),
        jointRadToDeg(1, q_ref_[1]),
        jointRadToDeg(2, q_ref_[2]));
}

void ArmHandlerNode::setGripperAngle(double target_deg) {
    gripper_deg_ = std::clamp(target_deg, gripper_min_, gripper_max_);
    q4_rad_ = degToRad(gripper_deg_ - gripper_min_);
    gripper_open_ = (gripper_deg_ >= 0.5 * (gripper_min_ + gripper_max_));
}

void ArmHandlerNode::cmdPoseCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr &msg) {
    if (msg->data.size() < 3) {
        RCLCPP_WARN(this->get_logger(), "cmd_pose erwartet mind. 3 Werte: [rotation, shoulder, elbow] in Grad.");
        return;
    }
    for (size_t i = 0; i < 3; ++i) {
        q_ref_[i] = degToJointRad(i, static_cast<double>(msg->data[i]));
    }
    if (msg->data.size() >= 4) {
        setGripperAngle(static_cast<double>(msg->data[3]));
    }
    target_pos_ = computeFK(q_ref_);
    has_twist_ = false;
    twist_moving_ = false;
}

void ArmHandlerNode::cmdTwistCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg) {
    const auto &lin = msg->twist.linear;
    const auto &ang = msg->twist.angular;
    if (!std::isfinite(lin.x) || !std::isfinite(lin.y) ||
        !std::isfinite(lin.z) || !std::isfinite(ang.z))
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "cmd_twist enthaelt NaN/Inf. Nachricht ignoriert.");
        return;
    }
    active_twist_ = msg->twist;
    last_twist_time_ = this->get_clock()->now();
    has_twist_ = true;
}

bool ArmHandlerNode::solveBoundedCartesian(
    const std::array<double, 3> &candidate,
    std::array<double, 3> &adjusted_out,
    std::array<double, 3> &q_out) const
{
    const double l1 = kL2;
    const double l2 = forearmLength();
    const double r_min = std::abs(l1 - l2) + 1e-4;
    const double r_max = (l1 + l2) - 1e-4;

    const double dx = candidate[0] - kShoulderX;
    const double dy = candidate[1] - kShoulderY;
    const double dz = candidate[2] - kShoulderZ;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    adjusted_out = candidate;
    if (dist > 1e-9 && (dist > r_max || dist < r_min)) {
        const double scale = ((dist > r_max) ? r_max : r_min) / dist;
        adjusted_out = {kShoulderX + dx * scale, kShoulderY + dy * scale, kShoulderZ + dz * scale};
    }

    if (!solveIK(adjusted_out[0], adjusted_out[1], adjusted_out[2], q_out)) {
        return false;
    }

    // Anti-Windup: verhindere Weglaufen der Referenz, falls MPC/Servo-Limits blockieren
    for (size_t i = 0; i < 3; ++i) {
        const double new_lead = std::abs(q_out[i] - q_[i]);
        const double old_lead = std::abs(q_ref_[i] - q_[i]);
        if (new_lead > max_ik_lead_rad_ && new_lead > old_lead) {
            return false;
        }
    }
    return true;
}

bool ArmHandlerNode::tryApplyCartesianTarget(const std::array<double, 3> &candidate) {
    std::array<double, 3> adjusted{};
    std::array<double, 3> q_sol{};
    if (!solveBoundedCartesian(candidate, adjusted, q_sol)) {
        return false;
    }
    target_pos_ = adjusted;
    q_ref_ = q_sol;
    return true;
}

bool ArmHandlerNode::tryArcSlideTarget(
    double phi_new, double r_target, double z_target, double v_r, double v_z)
{
    const double l1 = kL2;
    const double l2 = forearmLength();
    const double r_min_sq = l1 * l1 + l2 * l2 + 2.0 * l1 * l2 * std::cos(q_min_rad_[2]);
    const double cos_p = std::cos(phi_new);
    const double sin_p = std::sin(phi_new);

    bool found = false;
    double best_dist = std::numeric_limits<double>::infinity();
    std::array<double, 3> best_adj{};
    std::array<double, 3> best_q{};

    auto testArcPoint = [&](double r_c, double z_c, double err) {
        if (r_c < 0.0 || z_c < -0.015 || err >= best_dist) {
            return;
        }
        std::array<double, 3> adj{};
        std::array<double, 3> q_sol{};
        if (solveBoundedCartesian({kShoulderX + r_c * cos_p, kShoulderY + r_c * sin_p, z_c}, adj, q_sol)) {
            best_dist = err;
            best_adj = adj;
            best_q = q_sol;
            found = true;
        }
    };

    if (std::abs(v_z) > 1e-7) {
        // Entlang der Arbeitsraum-Boegen (q3_min, R_max, q2_min, q2_max) gleiten
        const double dz = z_target - kShoulderZ;
        if (r_min_sq - dz * dz >= 0.0) {
            const double r_c = std::sqrt(r_min_sq - dz * dz) + 1e-4;
            testArcPoint(r_c, z_target, std::abs(r_c - r_target));
        }
        const double r_max_sq = (l1 + l2) * (l1 + l2);
        if (r_max_sq - dz * dz >= 0.0) {
            const double r_c = std::max(0.0, std::sqrt(r_max_sq - dz * dz) - 1e-4);
            testArcPoint(r_c, z_target, std::abs(r_c - r_target));
        }
        for (int b : {0, 1}) {
            const double q2_b = (b == 0) ? q_min_rad_[1] : q_max_rad_[1];
            const double dz_e = z_target - (kShoulderZ + l1 * std::sin(q2_b));
            if (l2 * l2 - dz_e * dz_e >= 0.0) {
                const double r_c = l1 * std::cos(q2_b) + std::sqrt(l2 * l2 - dz_e * dz_e) + ((b == 0) ? -1e-4 : 1e-4);
                testArcPoint(r_c, z_target, std::abs(r_c - r_target));
            }
        }
    } else if (std::abs(v_r) > 1e-7) {
        // Bei rein radialer Vorgabe am Boden/Innenkreis entlang q2_min bzw. q3_min nachfuehren
        const double dx_min = r_target - l1 * std::cos(q_min_rad_[1]);
        if (l2 * l2 - dx_min * dx_min >= 0.0) {
            const double z_e = kShoulderZ + l1 * std::sin(q_min_rad_[1]);
            const double root = std::sqrt(l2 * l2 - dx_min * dx_min);
            testArcPoint(r_target, z_e - root + 1e-4, std::abs((z_e - root) - z_target));
            testArcPoint(r_target, z_e + root - 1e-4, std::abs((z_e + root) - z_target));
        }
        if (r_min_sq - r_target * r_target >= 0.0) {
            const double root = std::sqrt(r_min_sq - r_target * r_target);
            testArcPoint(r_target, kShoulderZ - root - 1e-4, std::abs((kShoulderZ - root) - z_target));
            testArcPoint(r_target, kShoulderZ + root + 1e-4, std::abs((kShoulderZ + root) - z_target));
        }
    }

    if (found) {
        target_pos_ = best_adj;
        q_ref_ = best_q;
    }
    return found;
}

void ArmHandlerNode::applyTwistStep(const rclcpp::Time &now) {
    if (!has_twist_) {
        return;
    }

    auto syncStop = [this]() {
        if (!twist_moving_) {
            return;
        }
        twist_moving_ = false;
        for (size_t i = 0; i < 3; ++i) {
            q_ref_[i] = std::clamp(q_[i] + v_[i] * kTauV, q_min_rad_[i], q_max_rad_[i]);
        }
        target_pos_ = computeFK(q_ref_);
    };

    if ((now - last_twist_time_).seconds() > twist_timeout_s_) {
        has_twist_ = false;
        active_twist_ = geometry_msgs::msg::Twist{};
        syncStop();
        return;
    }

    const double dr_step   = active_twist_.linear.x * twist_linear_scale_ * dt_;
    const double dy_step   = active_twist_.linear.y * twist_linear_scale_ * dt_;
    const double dz_step   = active_twist_.linear.z * twist_linear_scale_ * dt_;
    const double dphi_step = active_twist_.angular.z * twist_angular_scale_ * dt_;

    if (std::abs(dr_step) < 1e-7 && std::abs(dy_step) < 1e-7 &&
        std::abs(dz_step) < 1e-7 && std::abs(dphi_step) < 1e-7)
    {
        syncStop();
        return;
    }

    twist_moving_ = true;

    const auto p_cur = computeFK(q_);
    const double r_real = std::hypot(p_cur[0] - kShoulderX, p_cur[1] - kShoulderY);
    const double rel_x = target_pos_[0] - kShoulderX;
    const double rel_y = target_pos_[1] - kShoulderY;
    double r_cur = std::hypot(rel_x, rel_y);
    double z_cur = target_pos_[2];
    if (std::abs(z_cur - p_cur[2]) > 0.04) z_cur = p_cur[2];
    if (std::abs(r_cur - r_real) > 0.04)   r_cur = r_real;

    double phi_cur = (r_cur > 1e-6) ? std::atan2(rel_y, rel_x) : q_ref_[0];
    if (std::abs(phi_cur - q_[0]) > 0.25) {
        phi_cur = q_[0] + std::clamp(phi_cur - q_[0], -0.25, 0.25);
    }

    const double r_max_cyl = (kL2 + forearmLength()) - 1e-4;
    const double max_phi_lead = 0.9 * max_ik_lead_rad_;
    double phi_new = std::clamp(phi_cur + dphi_step, q_min_rad_[0], q_max_rad_[0]);
    phi_new = std::clamp(phi_new, q_[0] - max_phi_lead, q_[0] + max_phi_lead);

    const double cos_p = std::cos(phi_new);
    const double sin_p = std::sin(phi_new);
    for (double scale : {1.0, 0.5, 0.25}) {
        const double r_new = std::clamp(r_cur + dr_step * scale, 0.0, r_max_cyl);
        const double z_new = std::max(-0.02, z_cur + dz_step * scale);
        if (tryApplyCartesianTarget({kShoulderX + r_new * cos_p, kShoulderY + r_new * sin_p + dy_step * scale, z_new})) {
            return;
        }
        if ((std::abs(dz_step) > 1e-7 || std::abs(dr_step) > 1e-7) &&
            tryArcSlideTarget(phi_new, r_new, z_new, dr_step, dz_step))
        {
            return;
        }
    }

    if (std::abs(dphi_step) >= 1e-7) {
        q_ref_[0] = phi_new;
        target_pos_ = computeFK(q_ref_);
    } else if (std::abs(dy_step) >= 1e-7) {
        tryApplyCartesianTarget({kShoulderX + r_cur * cos_p, kShoulderY + r_cur * sin_p + dy_step, z_cur});
    }
}

void ArmHandlerNode::commandDefaultPose() {
    has_twist_ = false;
    twist_moving_ = false;
    active_twist_ = geometry_msgs::msg::Twist{};
    for (size_t i = 0; i < 3; ++i) {
        q_ref_[i] = degToJointRad(i, default_pose_deg_[i]);
    }
    setGripperAngle(defaultGripperDeg());
    target_pos_ = computeFK(q_ref_);
}

void ArmHandlerNode::publishServoCommands() {
    for (size_t i = 0; i < 3; ++i) {
        const double angle_deg = jointRadToDeg(i, q_[i]);
        if (std::abs(angle_deg - last_sent_angles_[i]) >= 0.1) {
            std_msgs::msg::Float32 cmd;
            cmd.data = static_cast<float>(angle_deg);
            pub_arm_joints_[i]->publish(cmd);
            // Erst bei verbundenem servo_node als gesendet markieren (DDS-Discovery beim Start)
            if (pub_arm_joints_[i]->get_subscription_count() > 0) {
                last_sent_angles_[i] = angle_deg;
            }
        }
    }

    if (std::abs(gripper_deg_ - last_sent_gripper_) >= 0.5) {
        std_msgs::msg::Float32 g_cmd;
        g_cmd.data = static_cast<float>(gripper_deg_);
        pub_gripper_cmd_->publish(g_cmd);
        if (pub_gripper_cmd_->get_subscription_count() > 0) {
            last_sent_gripper_ = gripper_deg_;
        }
    }
}

bool ArmHandlerNode::handleShutdownLoop(const rclcpp::Time &now) {
    if (g_arm_signal_count.load() == 0) {
        return false;
    }

    if (!shutting_down_) {
        shutting_down_ = true;
        shutdown_start_time_ = now;
        has_twist_ = false;
        shutdown_err_int_.fill(0.0);
        sub_cmd_point_.reset();
        sub_cmd_pose_.reset();
        sub_cmd_twist_.reset();
        sub_cmd_gripper_.reset();
        sub_arm_control_.reset();
        srv_gripper_toggle_.reset();
        srv_home_.reset();

        if (move_to_default_on_shutdown_) {
            RCLCPP_INFO(
                this->get_logger(),
                "Shutdown: fahre per PID (v_max=%.2f rad/s) ohne Acados in Ruheposition...",
                shutdown_max_vel_);
        }
    }

    const double elapsed = (now - shutdown_start_time_).seconds();
    const bool force_abort = (g_arm_signal_count.load() >= 2);
    if (shutdown_finalized_ || !move_to_default_on_shutdown_ ||
        force_abort || elapsed >= shutdown_timeout_s_)
    {
        if (move_to_default_on_shutdown_ && !shutdown_finalized_ && !force_abort) {
            RCLCPP_WARN(
                this->get_logger(),
                "Shutdown-Timeout (%.1fs) erreicht - beende ArmHandlerNode.",
                shutdown_timeout_s_);
        }
        if (timer_) {
            timer_->cancel();
        }
        rclcpp::shutdown();
        return true;
    }

    bool reached = true;
    std::array<double, 3> q_target{};
    for (size_t i = 0; i < 3; ++i) {
        q_target[i] = degToJointRad(i, default_pose_deg_[i]);
        const double err = q_target[i] - q_[i];
        double u_cmd = shutdown_pid_kp_ * err + shutdown_pid_ki_ * shutdown_err_int_[i] - shutdown_pid_kd_ * v_[i];
        if (std::abs(u_cmd) <= shutdown_max_vel_ || (err * u_cmd < 0.0)) {
            shutdown_err_int_[i] = std::clamp(shutdown_err_int_[i] + err * dt_, -0.5, 0.5);
            u_cmd = shutdown_pid_kp_ * err + shutdown_pid_ki_ * shutdown_err_int_[i] - shutdown_pid_kd_ * v_[i];
        }
        u_[i] = std::clamp(u_cmd, -shutdown_max_vel_, shutdown_max_vel_);
        if (std::abs(err) > 0.015 || std::abs(v_[i]) > 0.05) {
            reached = false;
        }
    }

    const double target_gripper = std::clamp(defaultGripperDeg(), gripper_min_, gripper_max_);
    const double max_step = radToDeg(shutdown_max_vel_) * dt_;
    setGripperAngle(gripper_deg_ + std::clamp(target_gripper - gripper_deg_, -max_step, max_step));
    if (std::abs(gripper_deg_ - target_gripper) > 0.5) {
        reached = false;
    }

    if (reached) {
        q_ = q_target;
        v_.fill(0.0);
        u_.fill(0.0);
        setGripperAngle(target_gripper);
        publishServoCommands();
        shutdown_finalized_ = true;
        RCLCPP_INFO(this->get_logger(), "Ruheposition per PID erreicht.");
        return true;
    }
    return false;
}

void ArmHandlerNode::controlLoop() {
    const auto now = this->get_clock()->now();
    if (handleShutdownLoop(now)) {
        return;
    }

    if (!shutting_down_) {
        applyTwistStep(now);

        picar_arm_safety_interface::msg::Parameters param_msg;
        param_msg.header.stamp = now;
        param_msg.p = {q4_rad_, box_xc_, box_yc_, box_zc_, box_dx_, box_dy_, box_dz_};
        pub_arm_params_->publish(param_msg);

        picar_arm_safety_interface::msg::References ref_msg;
        ref_msg.header.stamp = now;
        ref_msg.yref_0 = {q_ref_[0], q_ref_[1], q_ref_[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        ref_msg.yref = ref_msg.yref_0;
        ref_msg.yref_e = {q_ref_[0], q_ref_[1], q_ref_[2]};
        pub_arm_ref_->publish(ref_msg);

        picar_arm_safety_interface::msg::State state_msg;
        state_msg.header.stamp = now;
        state_msg.x = {q_[0], q_[1], q_[2], v_[0], v_[1], v_[2]};
        pub_arm_state_->publish(state_msg);
    }

    for (size_t i = 0; i < 3; ++i) {
        v_[i] += ((u_[i] - v_[i]) / kTauV) * dt_;
        q_[i] += v_[i] * dt_;
    }
    publishServoCommands();
}

}  // namespace picar_arm_handler

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, picar_arm_handler::armHandlerSignalHandler);
    std::signal(SIGTERM, picar_arm_handler::armHandlerSignalHandler);
    rclcpp::spin(std::make_shared<picar_arm_handler::ArmHandlerNode>());
    rclcpp::shutdown();
    return 0;
}
