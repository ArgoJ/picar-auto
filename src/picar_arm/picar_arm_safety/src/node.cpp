#include "picar_arm_safety/node.h"

namespace picar_arm_safety
{
PicarArmSafetyNode::PicarArmSafetyNode()
    : Node("picar_arm_safety_node"), control_timer_(nullptr)
{
    RCLCPP_INFO(this->get_logger(), "Initializing Picar Arm Safety Node...");

    // --- default values ---
    config_ = PicarArmSafetyNodeConfig();
    current_x_ = {0, 0, 0, 0, 0, 0};
    current_yref_0_ = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    current_yref_ = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    current_yref_e_ = {0, 0, 0};
    current_p_ = {0, 0, 0, 0.06, 0.12, 0.05, 0.07};

    // --- Parameters ---
    this->declare_parameters();
    this->setup_parameter_handlers();
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&PicarArmSafetyNode::on_parameter_update, this, std::placeholders::_1));
    this->load_parameters();

    // --- Subscriber ---
    state_sub_ = this->create_subscription<picar_arm_safety_interface::msg::State>(
        "/arm_state", 10,
        std::bind(&PicarArmSafetyNode::state_callback, this, std::placeholders::_1));
    references_sub_ = this->create_subscription<picar_arm_safety_interface::msg::References>(
        "/arm_reference", 10,
        std::bind(&PicarArmSafetyNode::references_callback, this, std::placeholders::_1));
    parameters_sub_ = this->create_subscription<picar_arm_safety_interface::msg::Parameters>(
        "/arm_params", 10,
        std::bind(&PicarArmSafetyNode::parameters_callback, this, std::placeholders::_1));

    // --- Publisher ---
    control_pub_ = this->create_publisher<picar_arm_safety_interface::msg::Control>(
        "/arm_control", 10);

    // --- Init solver ---
    this->initialize_solver();
    this->apply_all_parameters_to_solver();
    this->start_control_timer(config_.ts);
}

PicarArmSafetyNode::~PicarArmSafetyNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down and freeing Acados solver memory.");
    if (ocp_capsule_) {
        int status = ocp_picar_arm_safety_filter_809ed34e_acados_free(ocp_capsule_);
        if (status) {
            RCLCPP_ERROR(this->get_logger(), "ocp_picar_arm_safety_filter_809ed34e_acados_free() returned status %d.", status);
        }
        status = ocp_picar_arm_safety_filter_809ed34e_acados_free_capsule(ocp_capsule_);
        if (status) {
            RCLCPP_ERROR(this->get_logger(), "ocp_picar_arm_safety_filter_809ed34e_acados_free_capsule() returned status %d.", status);
        }
    }
}


// --- Core Methods ---
void PicarArmSafetyNode::initialize_solver() {
    ocp_capsule_ = ocp_picar_arm_safety_filter_809ed34e_acados_create_capsule();
    int status = ocp_picar_arm_safety_filter_809ed34e_acados_create(ocp_capsule_);
    if (status) {
        RCLCPP_FATAL(this->get_logger(), "ocp_picar_arm_safety_filter_809ed34eacados_create() failed with status %d.", status);
        rclcpp::shutdown();
    }

    ocp_nlp_in_ = ocp_picar_arm_safety_filter_809ed34e_acados_get_nlp_in(ocp_capsule_);
    ocp_nlp_out_ = ocp_picar_arm_safety_filter_809ed34e_acados_get_nlp_out(ocp_capsule_);
    ocp_nlp_sens_ = ocp_picar_arm_safety_filter_809ed34e_acados_get_sens_out(ocp_capsule_);
    ocp_nlp_config_ = ocp_picar_arm_safety_filter_809ed34e_acados_get_nlp_config(ocp_capsule_);
    ocp_nlp_opts_ = ocp_picar_arm_safety_filter_809ed34e_acados_get_nlp_opts(ocp_capsule_);
    ocp_nlp_dims_ = ocp_picar_arm_safety_filter_809ed34e_acados_get_nlp_dims(ocp_capsule_);

    RCLCPP_INFO(this->get_logger(), "acados solver initialized successfully.");
}

void PicarArmSafetyNode::control_loop() {
    std::array<double, OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NX> x0{};
    std::array<double, OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NY0> yref0{};
    std::array<double, OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NY> yref{};
    std::array<double, OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NYN> yrefN{};
    std::array<double, OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NP> p{};

    {
        if (!has_received_state_) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for first state message...");
            return;
        }
        x0 = current_x_;
        yref0 = current_yref_0_;
        yref = current_yref_;
        yrefN = current_yref_e_;
        p = current_p_;
    }

    {
        // Update solver
        this->set_x0(x0.data());
        this->set_yref0(yref0.data());
        this->set_yrefs(yref.data());
        this->set_yref_e(yrefN.data());
        this->set_ocp_parameters(p.data(), p.size());

        // Solve OCP
        int status;
        if (first_solve_) {
            this->warmstart_solver_states(x0.data());
            status = this->ocp_solve();
        }
        else {
            status = this->feedback_rti_solve();
        }
        this->solver_status_behaviour(status);
    }
}

void PicarArmSafetyNode::solver_status_behaviour(int status) {
    // publish u0 also if the solver failed
    this->get_control(u0_.data(), 0);
    this->publish_control(u0_, status);
    // prepare for next iteration
    if (status == ACADOS_SUCCESS) {
        first_solve_ = false;
        this->prepare_rti_solve();
    }
    else {
        first_solve_ = true;
        if (config_.verbose) {
            ocp_picar_arm_safety_filter_809ed34e_acados_print_stats(ocp_capsule_);
        }
        RCLCPP_INFO(this->get_logger(), "Resetting acados solver memory...");
        ocp_picar_arm_safety_filter_809ed34e_acados_reset(ocp_capsule_, 1, 0, 0, 0);
    }
}


// --- ROS Callbacks ---
void PicarArmSafetyNode::state_callback(const picar_arm_safety_interface::msg::State::SharedPtr msg) {
    if (std::any_of(msg->x.begin(), msg->x.end(), [](double val){ return !std::isfinite(val); })) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "State callback received NaN/Inf in 'x'. Ignoring message.");
        return;
    }
    current_x_ = msg->x;
    has_received_state_ = true;
    RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "State callback: x=" << current_x_);
}

void PicarArmSafetyNode::references_callback(const picar_arm_safety_interface::msg::References::SharedPtr msg) {
    if (std::any_of(msg->yref_0.begin(), msg->yref_0.end(), [](double val){ return !std::isfinite(val); })) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Reference callback received NaN/Inf in 'yref_0'. Ignoring message.");
        return;
    }
    if (std::any_of(msg->yref.begin(), msg->yref.end(), [](double val){ return !std::isfinite(val); })) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Reference callback received NaN/Inf in 'yref'. Ignoring message.");
        return;
    }
    if (std::any_of(msg->yref_e.begin(), msg->yref_e.end(), [](double val){ return !std::isfinite(val); })) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Reference callback received NaN/Inf in 'yref_e'. Ignoring message.");
        return;
    } current_yref_0_ = msg->yref_0;   current_yref_  = msg->yref;     current_yref_e_ = msg->yref_e;  RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Refs callback: yref_0=" << current_yref_0_);   RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Refs callback: yref="   << current_yref_);    RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Refs callback: yref_e=" << current_yref_e_);  
}

void PicarArmSafetyNode::parameters_callback(const picar_arm_safety_interface::msg::Parameters::SharedPtr msg) {
    if (std::any_of(msg->p.begin(), msg->p.end(), [](double val){ return !std::isfinite(val); })) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Parameter callback received NaN/Inf in 'p'. Ignoring message.");
        return;
    }
    current_p_ = msg->p;
    RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Params callback: p=" << current_p_);
}


// --- ROS Publisher ---
void PicarArmSafetyNode::publish_control(
        const std::array<double, OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NU>& u0, 
        int status
) {
    auto control = std::make_unique<picar_arm_safety_interface::msg::Control>();
    control->header.stamp = this->get_clock()->now();
    control->status = status;
    control->u = u0;
    control_pub_->publish(std::move(control));
}


// --- ROS Parameter ---
void PicarArmSafetyNode::setup_parameter_handlers() {

    // Stage Constraints
    parameter_handlers_["constraints.lbu"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N);
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NBU>(p, res, "lbu", stages);
        };
    parameter_handlers_["constraints.lbx"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N);
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NBX>(p, res, "lbx", stages);
        };
    parameter_handlers_["constraints.lh"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N);
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NH>(p, res, "lh", stages);
        };
    parameter_handlers_["constraints.lsh"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N);
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NSH>(p, res, "lsh", stages);
        };
    parameter_handlers_["constraints.ubu"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N);
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NBU>(p, res, "ubu", stages);
        };
    parameter_handlers_["constraints.ubx"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N);
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NBX>(p, res, "ubx", stages);
        };
    parameter_handlers_["constraints.uh"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N);
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NH>(p, res, "uh", stages);
        };
    parameter_handlers_["constraints.ush"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N);
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NSH>(p, res, "ush", stages);
        };

    // Terminal Constraints
    parameter_handlers_["constraints.lh_e"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NHN>(p, res, "lh", std::vector<int>{OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N});
        };
    parameter_handlers_["constraints.lsh_e"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NSHN>(p, res, "lsh", std::vector<int>{OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N});
        };
    parameter_handlers_["constraints.uh_e"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NHN>(p, res, "uh", std::vector<int>{OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N});
        };
    parameter_handlers_["constraints.ush_e"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NSHN>(p, res, "ush", std::vector<int>{OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N});
        };

    // Weights
    parameter_handlers_["cost.W_0"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NY0>(p, res, "W", std::vector<int>{0});
        };
    parameter_handlers_["cost.W"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N);
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NY>(p, res, "W", stages);
        };
    parameter_handlers_["cost.W_e"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NYN>(p, res, "W", std::vector<int>{OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N});
        };
    // Stage Slacks
    parameter_handlers_["cost.Zl"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N);
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NS>(p, res, "Zl", stages);
        };
    parameter_handlers_["cost.Zu"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N);
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NS>(p, res, "Zu", stages);
        };
    parameter_handlers_["cost.zl"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N);
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NS>(p, res, "zl", stages);
        };
    parameter_handlers_["cost.zu"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N);
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NS>(p, res, "zu", stages);
        };

    // Terminal Slacks
    parameter_handlers_["cost.Zl_e"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NSN>(p, res, "Zl", std::vector<int>{OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N});
        };
    parameter_handlers_["cost.Zu_e"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NSN>(p, res, "Zu", std::vector<int>{OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N});
        };
    parameter_handlers_["cost.zl_e"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NSN>(p, res, "zl", std::vector<int>{OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N});
        };
    parameter_handlers_["cost.zu_e"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_NSN>(p, res, "zu", std::vector<int>{OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N});
        };

    // Solver Options
    parameter_handlers_["solver_options.print_level"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult&) {
            int print_level = p.as_int();
            ocp_nlp_solver_opts_set(ocp_nlp_config_, ocp_nlp_opts_, "print_level", &print_level);
        };

    // Ros Configs
    parameter_handlers_["ts"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->set_period(p.as_double());
            // Restart timer with the new period
            if (!this->is_running()) {
                // Assumes that the node is not yet ready and that the timer will be started later
                return;
            }
            try {
                this->start_control_timer(this->config_.ts);
            } catch (const std::exception& e) {
                res.reason = "Failed to start control timer, while setting parameter '" + p.get_name() + "': " + e.what();
                res.successful = false;
            }
        };
    parameter_handlers_["verbose"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult&) {
            this->config_.verbose = p.as_bool();
        };
}

void PicarArmSafetyNode::declare_parameters() {
    // Constraints
    this->declare_parameter("constraints.lbu", std::vector<double>{-3, -3, -3});
    this->declare_parameter("constraints.lbx", std::vector<double>{-1.4, -0.8, -1.6, -3, -3, -3});
    this->declare_parameter("constraints.lh", std::vector<double>{0, 0, 0, 0, 0, 0, 0, 0, 0});
    this->declare_parameter("constraints.lh_e", std::vector<double>{0, 0, 0, 0, 0, 0, 0, 0, 0});
    this->declare_parameter("constraints.lsh", std::vector<double>{0, 0, 0, 0, 0, 0, 0, 0, 0});
    this->declare_parameter("constraints.lsh_e", std::vector<double>{0, 0, 0, 0, 0, 0, 0, 0, 0});
    this->declare_parameter("constraints.ubu", std::vector<double>{3, 3, 3});
    this->declare_parameter("constraints.ubx", std::vector<double>{1.4, 1.6, 1.6, 3, 3, 3});
    this->declare_parameter("constraints.uh", std::vector<double>{1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000});
    this->declare_parameter("constraints.uh_e", std::vector<double>{1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000});
    this->declare_parameter("constraints.ush", std::vector<double>{0, 0, 0, 0, 0, 0, 0, 0, 0});
    this->declare_parameter("constraints.ush_e", std::vector<double>{0, 0, 0, 0, 0, 0, 0, 0, 0});

    // Weights
    this->declare_parameter("cost.W", std::vector<double>{5, 5, 5, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2});
    this->declare_parameter("cost.W_0", std::vector<double>{5, 5, 5, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2});
    this->declare_parameter("cost.W_e", std::vector<double>{5, 5, 5});

    // Slacks
    this->declare_parameter("cost.Zl", std::vector<double>{1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000});
    this->declare_parameter("cost.Zl_e", std::vector<double>{1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000});
    this->declare_parameter("cost.Zu", std::vector<double>{0, 0, 0, 0, 0, 0, 0, 0, 0});
    this->declare_parameter("cost.Zu_e", std::vector<double>{0, 0, 0, 0, 0, 0, 0, 0, 0});
    this->declare_parameter("cost.zl", std::vector<double>{100, 100, 100, 100, 100, 100, 100, 100, 100});
    this->declare_parameter("cost.zl_e", std::vector<double>{100, 100, 100, 100, 100, 100, 100, 100, 100});
    this->declare_parameter("cost.zu", std::vector<double>{0, 0, 0, 0, 0, 0, 0, 0, 0});
    this->declare_parameter("cost.zu_e", std::vector<double>{0, 0, 0, 0, 0, 0, 0, 0, 0});

    // Solver Options
    this->declare_parameter("solver_options.print_level", 0);

    // Ros Configs
    this->declare_parameter("ts", 0.05);
    this->declare_parameter("verbose", false);
}

void PicarArmSafetyNode::load_parameters() {
    this->get_parameter("ts", config_.ts);
    this->get_parameter("verbose", config_.verbose);
}

void PicarArmSafetyNode::apply_all_parameters_to_solver() {
    if (!ocp_capsule_) {
        RCLCPP_WARN(this->get_logger(), "apply_all_parameters_to_solver() called before solver init.");
        return;
    }
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;

    for (auto & kv : parameter_handlers_) {
        const auto & name = kv.first;
        if (!this->has_parameter(name)) continue;
        auto param = this->get_parameter(name);
        kv.second(param, res);
        if (!res.successful) {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to apply initial parameter '%s': %s",
                name.c_str(), res.reason.c_str());
            // reset flag for next parameter
            res.successful = true;
            res.reason.clear();
        }
    }
}

rcl_interfaces::msg::SetParametersResult PicarArmSafetyNode::on_parameter_update(
    const std::vector<rclcpp::Parameter>& params
) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : params) {
        auto& param_name = param.get_name();

        if (parameter_handlers_.count(param_name)) {
            parameter_handlers_.at(param_name)(param, result);
            if (!result.successful) break;
        } else {
            result.reason = "Update for unknown parameter '" + param_name + "' received.";
            result.successful = false;
        }
    }
    return result;
}

template <size_t N>
void PicarArmSafetyNode::update_param_array(
    const rclcpp::Parameter& param,
    std::array<double, N>& destination_array,
    rcl_interfaces::msg::SetParametersResult& result
) {
    auto values = param.as_double_array();

    if (values.size() != N) {
        result.successful = false;
        result.reason = "Parameter '" + param.get_name() + "' has size " +
                        std::to_string(values.size()) + ", but expected is " + std::to_string(N) + ".";
        return;
    }

    std::copy_n(values.begin(), N, destination_array.begin());
}

template<size_t N>
void PicarArmSafetyNode::update_constraint(
    const rclcpp::Parameter& param,
    rcl_interfaces::msg::SetParametersResult& result,
    const char* field,
    const std::vector<int>& stages
) {
    auto values = param.as_double_array();

    if (values.size() != N) {
        result.successful = false;
        result.reason = "Constraint '" + std::string(param.get_name()) + "' has size " +
                      std::to_string(values.size()) + ", but expected is " + std::to_string(N) + ".";
        return;
    }

    std::array<double, N> vec{};
    std::copy_n(values.begin(), N, vec.begin());

    {
        for (int stage : stages) {
            int status = ocp_nlp_constraints_model_set(ocp_nlp_config_, ocp_nlp_dims_, ocp_nlp_in_, ocp_nlp_out_, stage, field, vec.data());

            if (status != ACADOS_SUCCESS) {
                result.successful = false;
                result.reason = "Acados solver failed to set cost field '" + std::string(field) +
                            "' for stage " + std::to_string(stage) + " (error code: " + std::to_string(status) + ")";
                return;
            }
        }
    }
}

template<size_t N>
void PicarArmSafetyNode::update_cost(
    const rclcpp::Parameter& param,
    rcl_interfaces::msg::SetParametersResult& result,
    const char* field,
    const std::vector<int>& stages
) {
    const auto values = param.as_double_array();

    if (values.size() != N) {
        result.successful = false;
        result.reason = "Cost '" + std::string(param.get_name()) + "' has size " +
                      std::to_string(values.size()) + ", but expected is " + std::to_string(N) + ".";
        return;
    }

    std::array<double, N> vec;
    std::array<double, N * N> mat{};
    std::copy_n(values.begin(), N, vec.begin());
    double* data_ptr = nullptr;

    const bool is_weight = std::strcmp(field, "W") == 0;
    if (is_weight) {
        mat = diag_from_vec(vec);
        data_ptr = mat.data();
        RCLCPP_INFO_STREAM(this->get_logger(), "update cost field '" << field << "' mat(flat) = " << mat);
    } else {
        data_ptr = vec.data();
        RCLCPP_INFO_STREAM(this->get_logger(), "update cost field '" << field << "' values = " << vec);
    }

    {
        for (int stage : stages) {
            int status = ocp_nlp_cost_model_set(ocp_nlp_config_, ocp_nlp_dims_, ocp_nlp_in_, stage, field, data_ptr);
            if (status != ACADOS_SUCCESS) {
                result.successful = false;
                result.reason = "Acados solver failed to set cost field '" + std::string(field) +
                            "' for stage " + std::to_string(stage) + " (error code: " + std::to_string(status) + ")";
                return;
            }
        }
    }
}

// --- Helpers ---
void PicarArmSafetyNode::set_period(double period_seconds) {
    if (config_.ts == period_seconds) {
        // Nothing to do
        return;
    }
    RCLCPP_INFO_STREAM(
        this->get_logger(), "update period 'Ts' = " << period_seconds << "s");
    this->config_.ts = period_seconds;
    // Check period validity
    if (this->config_.ts <= 0.0) {
        this->config_.ts = 0.05;
        RCLCPP_WARN(this->get_logger(),
            "Control period must be positive, defaulting to 0.05s, the first time step of the OCP.");
    }
}

void PicarArmSafetyNode::start_control_timer(double period_seconds) {
    this->set_period(period_seconds);
    // if timer already exists, restart with new period
    if (this->is_running()) {
        RCLCPP_WARN(this->get_logger(), "Control timer already running, restarting...");
        control_timer_->cancel();
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Starting control loop with period " << period_seconds << "s.");
    // create timer
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period_seconds));
    control_timer_ = this->create_wall_timer(
        period,
        std::bind(&PicarArmSafetyNode::control_loop, this));
}


// --- Acados Solver ---
int PicarArmSafetyNode::prepare_rti_solve() {
    int phase = PREPARATION;
    ocp_nlp_sqp_rti_opts_set(ocp_nlp_config_, ocp_nlp_opts_, "rti_phase", &phase);
    int status = ocp_picar_arm_safety_filter_809ed34e_acados_solve(ocp_capsule_);
    if (status != ACADOS_SUCCESS && status != ACADOS_READY) {
        first_solve_ = true;
        RCLCPP_ERROR(this->get_logger(), "Solver failed at preparation phase: %d", status);
    }
    return status;
}

int PicarArmSafetyNode::feedback_rti_solve() {
    int phase = FEEDBACK;
    ocp_nlp_sqp_rti_opts_set(ocp_nlp_config_, ocp_nlp_opts_, "rti_phase", &phase);
    int status = ocp_picar_arm_safety_filter_809ed34e_acados_solve(ocp_capsule_);
    if (status != ACADOS_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Solver failed at feedback phase: %d", status);
    }
    return status;
}
int PicarArmSafetyNode::ocp_solve() {
    int phase = PREPARATION_AND_FEEDBACK;
    ocp_nlp_sqp_rti_opts_set(ocp_nlp_config_, ocp_nlp_opts_, "rti_phase", &phase);
    int status = ocp_picar_arm_safety_filter_809ed34e_acados_solve(ocp_capsule_);
    if (status != ACADOS_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Solver failed with status: %d", status);
    }
    return status;
}


// --- Acados Getter ---
void PicarArmSafetyNode::get_control(double* u, int stage) {
    ocp_nlp_out_get(ocp_nlp_config_, ocp_nlp_dims_, ocp_nlp_out_, stage, "u", u);
}

void PicarArmSafetyNode::get_state(double* x, int stage) {
    ocp_nlp_out_get(ocp_nlp_config_, ocp_nlp_dims_, ocp_nlp_out_, stage, "x", x);
}


// --- Acados Setter ---
void PicarArmSafetyNode::set_x0(double* x0) {
    int lbx_status = ocp_nlp_constraints_model_set(ocp_nlp_config_, ocp_nlp_dims_, ocp_nlp_in_, ocp_nlp_out_, 0, "lbx", x0);
    int ubx_status = ocp_nlp_constraints_model_set(ocp_nlp_config_, ocp_nlp_dims_, ocp_nlp_in_, ocp_nlp_out_, 0, "ubx", x0);
    check_acados_status("set_x0 (lbx)", 0, lbx_status);
    check_acados_status("set_x0 (ubx)", 0, ubx_status);
}

void PicarArmSafetyNode::set_yref(double* yref, int stage) {
    int status = ocp_nlp_cost_model_set(ocp_nlp_config_, ocp_nlp_dims_, ocp_nlp_in_, stage, "yref", yref);
    check_acados_status("set_yref (yref)", stage, status);
}
void PicarArmSafetyNode::set_yref0(double* yref0) {
    this->set_yref(yref0, 0);
}

void PicarArmSafetyNode::set_yrefs(double* yref) {
    for (int i = 1; i < OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N; i++) {
        this->set_yref(yref, i);
    }
}

void PicarArmSafetyNode::set_yref_e(double* yrefN) {
    this->set_yref(yrefN, OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N);
}

void PicarArmSafetyNode::set_ocp_parameter(double* p, size_t np, int stage) {
    int status = ocp_picar_arm_safety_filter_809ed34e_acados_update_params(ocp_capsule_, stage, p, np);
    check_acados_status("set_ocp_parameter (p)", stage, status);
}

void PicarArmSafetyNode::set_ocp_parameters(double* p, size_t np) {
    for (int i = 0; i <= OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N; i++) {
        this->set_ocp_parameter(p, np, i);
    }
}

void PicarArmSafetyNode::warmstart_solver_states(double *x0) {
    for (int i = 1; i <= OCP_PICAR_ARM_SAFETY_FILTER_809ED34E_N; ++i) {
        ocp_nlp_out_set(ocp_nlp_config_, ocp_nlp_dims_, ocp_nlp_out_, ocp_nlp_in_, i, "x", x0);
    }
}


// --- Acados Helper ---
bool PicarArmSafetyNode::check_acados_status(const char* field, int stage, int status) {
    if (status != ACADOS_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "%s failed at stage %d: %d", field, stage, status);
        return false;
    }
    return true;
}

} // namespace picar_arm_safety


// --- Main ---
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Suppress ROS timer logging
    rcutils_logging_set_logger_level("rcl", RCUTILS_LOG_SEVERITY_WARN);
    rcutils_logging_set_logger_level("rclcpp", RCUTILS_LOG_SEVERITY_WARN);

    auto node = std::make_shared<picar_arm_safety::PicarArmSafetyNode>();
    RCLCPP_INFO(node->get_logger(), "Using SingleThreadedExecutor.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
