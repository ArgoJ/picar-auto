#ifndef PICAR_ARM_SAFETY_NODE_H
#define PICAR_ARM_SAFETY_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <array>
#include <vector>
#include <cmath>
#include <algorithm>
#include <unordered_map>

// ROS2 message includes
#include "picar_arm_safety_interface/msg/state.hpp"
#include "picar_arm_safety_interface/msg/control.hpp"
#include "picar_arm_safety_interface/msg/references.hpp"
#include "picar_arm_safety_interface/msg/parameters.hpp"
#include "std_msgs/msg/header.hpp"

// Acados includes
#include "acados/ocp_nlp/ocp_nlp_sqp_rti.h"
#include "acados/ocp_nlp/ocp_nlp_common.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_ocp_picar_arm_safety_filter_5c21b6cb.h"

// Package includes
#include "picar_arm_safety/utils.hpp"
#include "picar_arm_safety/config.hpp"


namespace picar_arm_safety
{
class PicarArmSafetyNode : public rclcpp::Node {
private:
    // --- ROS Subscriptions ---
    rclcpp::Subscription<picar_arm_safety_interface::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<picar_arm_safety_interface::msg::References>::SharedPtr references_sub_;
    rclcpp::Subscription<picar_arm_safety_interface::msg::Parameters>::SharedPtr parameters_sub_;

    // --- ROS Publishers ---
    rclcpp::Publisher<picar_arm_safety_interface::msg::Control>::SharedPtr control_pub_;

    // --- ROS Params and Timer
    rclcpp::TimerBase::SharedPtr control_timer_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    using ParamHandler = std::function<void(const rclcpp::Parameter&, rcl_interfaces::msg::SetParametersResult&)>;
    std::unordered_map<std::string, ParamHandler> parameter_handlers_;

    // --- Acados Solver ---
    ocp_picar_arm_safety_filter_5c21b6cb_solver_capsule *ocp_capsule_;
    ocp_nlp_in* ocp_nlp_in_;
    ocp_nlp_out* ocp_nlp_out_;
    ocp_nlp_out* ocp_nlp_sens_;
    ocp_nlp_config* ocp_nlp_config_;
    void* ocp_nlp_opts_;
    ocp_nlp_dims* ocp_nlp_dims_;

    // --- Data and States ---
    PicarArmSafetyNodeConfig config_;
    bool has_received_state_{false};
    bool has_new_yref_{true};
    bool has_new_p_{true};
    bool first_solve_{true};
    std::array<double, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NU> u0_{};
    std::array<double, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NX> current_x_{};
    std::array<double, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NY0> current_yref_0_{};
    std::array<double, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NY> current_yref_{};
    std::array<double, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NYN> current_yref_e_{};
    std::array<double, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NP> current_p_{};

public:
    PicarArmSafetyNode();
    ~PicarArmSafetyNode();

private:
    // --- Core Methods ---
    void initialize_solver();
    void control_loop();
    void solver_status_behaviour(int status);

    // --- ROS Callbacks ---
    void state_callback(const picar_arm_safety_interface::msg::State::SharedPtr msg);
    void references_callback(const picar_arm_safety_interface::msg::References::SharedPtr msg);
    void parameters_callback(const picar_arm_safety_interface::msg::Parameters::SharedPtr msg);

    // --- ROS Publisher ---
    void publish_control(
        const std::array<double, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NU>& u0, 
        int status);

    // --- ROS Parameter ---
    void setup_parameter_handlers();
    void declare_parameters();
    void load_parameters();
    void apply_all_parameters_to_solver();
    rcl_interfaces::msg::SetParametersResult on_parameter_update(
        const std::vector<rclcpp::Parameter>& params);

    template<size_t N>
    void update_constraint(
        const rclcpp::Parameter& param,
        rcl_interfaces::msg::SetParametersResult& result,
        const char* field,
        const std::vector<int>& stages);
    template<size_t N>
    void update_cost(
        const rclcpp::Parameter& param,
        rcl_interfaces::msg::SetParametersResult& result,
        const char* field,
        const std::vector<int>& stages);

    // --- Helpers ---
    bool set_period(double period_seconds);
    void start_control_timer(double period_seconds = static_cast<double>(0.05));
    bool is_running() const {
        return control_timer_ && !control_timer_->is_canceled();
    }

    // --- Acados Solver ---
    int prepare_rti_solve();
    int feedback_rti_solve();
    int ocp_solve();

    // --- Acados Getter ---
    void get_control(double* u, int stage);
    void get_state(double* x, int stage);

    // --- Acados Setter ---
    void set_x0(double* x0);
    void set_yref(double* yref, int stage);
    void set_yref0(double* yref0);
    void set_yrefs(double* yref);
    void set_yref_e(double* yref_e);
    void set_ocp_parameter(double* p, size_t np, int stage);
    void set_ocp_parameters(double* p, size_t np);
    void warmstart_solver_states(double *x0);

    // --- Helpers ---
    bool check_acados_status(
        const char* field,
        int stage,
        int status);
};

} // namespace picar_arm_safety

#endif // PICAR_ARM_SAFETY_NODE_H
