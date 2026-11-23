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
        "arm_state", 10,
        std::bind(&PicarArmSafetyNode::state_callback, this, std::placeholders::_1));
    references_sub_ = this->create_subscription<picar_arm_safety_interface::msg::References>(
        "arm_reference", 10,
        std::bind(&PicarArmSafetyNode::references_callback, this, std::placeholders::_1));
    parameters_sub_ = this->create_subscription<picar_arm_safety_interface::msg::Parameters>(
        "arm_params", 10,
        std::bind(&PicarArmSafetyNode::parameters_callback, this, std::placeholders::_1));

    // --- Publisher ---
    control_pub_ = this->create_publisher<picar_arm_safety_interface::msg::Control>(
        "arm_control", 10);

    // --- Init solver ---
    this->initialize_solver();
    if (!ocp_capsule_) return;
    this->apply_all_parameters_to_solver();
    this->start_control_timer(config_.ts);
}

PicarArmSafetyNode::~PicarArmSafetyNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down and freeing Acados solver memory.");
    if (ocp_capsule_) {
        int status = ocp_picar_arm_safety_filter_5c21b6cb_acados_free(ocp_capsule_);
        if (status) {
            RCLCPP_ERROR(this->get_logger(), "ocp_picar_arm_safety_filter_5c21b6cb_acados_free() returned status %d.", status);
        }
        status = ocp_picar_arm_safety_filter_5c21b6cb_acados_free_capsule(ocp_capsule_);
        if (status) {
            RCLCPP_ERROR(this->get_logger(), "ocp_picar_arm_safety_filter_5c21b6cb_acados_free_capsule() returned status %d.", status);
        }
    }
}


// --- Core Methods ---
void PicarArmSafetyNode::initialize_solver() {
    ocp_capsule_ = ocp_picar_arm_safety_filter_5c21b6cb_acados_create_capsule();
    int status = ocp_picar_arm_safety_filter_5c21b6cb_acados_create(ocp_capsule_);
    if (status) {
        RCLCPP_FATAL(this->get_logger(), "ocp_picar_arm_safety_filter_5c21b6cb_acados_create() failed with status %d.", status);
        ocp_picar_arm_safety_filter_5c21b6cb_acados_free_capsule(ocp_capsule_);
        ocp_capsule_ = nullptr;
        rclcpp::shutdown();
        return;
    }

    ocp_nlp_in_ = ocp_picar_arm_safety_filter_5c21b6cb_acados_get_nlp_in(ocp_capsule_);
    ocp_nlp_out_ = ocp_picar_arm_safety_filter_5c21b6cb_acados_get_nlp_out(ocp_capsule_);
    ocp_nlp_sens_ = ocp_picar_arm_safety_filter_5c21b6cb_acados_get_sens_out(ocp_capsule_);
    ocp_nlp_config_ = ocp_picar_arm_safety_filter_5c21b6cb_acados_get_nlp_config(ocp_capsule_);
    ocp_nlp_opts_ = ocp_picar_arm_safety_filter_5c21b6cb_acados_get_nlp_opts(ocp_capsule_);
    ocp_nlp_dims_ = ocp_picar_arm_safety_filter_5c21b6cb_acados_get_nlp_dims(ocp_capsule_);

    RCLCPP_INFO(this->get_logger(), "acados solver initialized successfully.");
}

void PicarArmSafetyNode::control_loop() {
    std::array<double, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NX> x0{};
    std::array<double, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NY0> yref0{};
    std::array<double, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NY> yref{};
    std::array<double, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NYN> yrefN{};
    std::array<double, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NP> p{};
    bool wait_for_state = false;
    bool update_yref = false;
    bool update_p = false;
    {
        if (!has_received_state_) {
            wait_for_state = true;
        } else {
            x0 = current_x_;
            update_yref = has_new_yref_;
            if (update_yref) {
                yref0 = current_yref_0_;
                yref = current_yref_;
                yrefN = current_yref_e_;
                has_new_yref_ = false;
            }
            update_p = has_new_p_;
            if (update_p) {
                p = current_p_;
                has_new_p_ = false;
            }
        }
    }
    if (wait_for_state) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for first state message...");
        return;
    }

    {
        // Update solver
        this->set_x0(x0.data());
        if (update_yref) {
            this->set_yref0(yref0.data());
            this->set_yrefs(yref.data());
            this->set_yref_e(yrefN.data());
        }
        if (update_p) {
            this->set_ocp_parameters(p.data(), p.size());
        }

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
        bool verbose = config_.verbose;
        if (verbose) {
            ocp_picar_arm_safety_filter_5c21b6cb_acados_print_stats(ocp_capsule_);
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Resetting acados solver memory...");
        ocp_picar_arm_safety_filter_5c21b6cb_acados_reset(ocp_capsule_, 1, 0, 0, 0);
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

    {
        current_x_ = msg->x;
        has_received_state_ = true;
    }
    RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "State callback: x=" << msg->x);
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
    }

    { current_yref_0_ = msg->yref_0;   current_yref_  = msg->yref;     current_yref_e_ = msg->yref_e; 
        has_new_yref_ = true;
    } RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Refs callback: yref_0=" << msg->yref_0);   RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Refs callback: yref="   << msg->yref);    RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Refs callback: yref_e=" << msg->yref_e);  
}

void PicarArmSafetyNode::parameters_callback(const picar_arm_safety_interface::msg::Parameters::SharedPtr msg) {
    if (std::any_of(msg->p.begin(), msg->p.end(), [](double val){ return !std::isfinite(val); })) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Parameter callback received NaN/Inf in 'p'. Ignoring message.");
        return;
    }

    {
        current_p_ = msg->p;
        has_new_p_ = true;
    }
    RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Params callback: p=" << msg->p);
}


// --- ROS Publisher ---
void PicarArmSafetyNode::publish_control(
        const std::array<double, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NU>& u0, 
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
            auto stages = range(0, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NBU>(p, res, "lbu", stages);
        };
    parameter_handlers_["constraints.lbx"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NBX>(p, res, "lbx", stages);
        };
    parameter_handlers_["constraints.lh"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NH>(p, res, "lh", stages);
        };
    parameter_handlers_["constraints.lsh"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NSH>(p, res, "lsh", stages);
        };
    parameter_handlers_["constraints.ubu"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(0, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NBU>(p, res, "ubu", stages);
        };
    parameter_handlers_["constraints.ubx"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NBX>(p, res, "ubx", stages);
        };
    parameter_handlers_["constraints.uh"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NH>(p, res, "uh", stages);
        };
    parameter_handlers_["constraints.ush"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NSH>(p, res, "ush", stages);
        };

    // Terminal Constraints
    parameter_handlers_["constraints.lh_e"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NHN>(p, res, "lh", std::vector<int>{OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N});
        };
    parameter_handlers_["constraints.lsh_e"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NSHN>(p, res, "lsh", std::vector<int>{OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N});
        };
    parameter_handlers_["constraints.uh_e"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NHN>(p, res, "uh", std::vector<int>{OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N});
        };
    parameter_handlers_["constraints.ush_e"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->update_constraint<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NSHN>(p, res, "ush", std::vector<int>{OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N});
        };

    // Weights
    parameter_handlers_["cost.W_0"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NY0>(p, res, "W", std::vector<int>{0});
        };
    parameter_handlers_["cost.W"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NY>(p, res, "W", stages);
        };
    parameter_handlers_["cost.W_e"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NYN>(p, res, "W", std::vector<int>{OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N});
        };
    // Stage Slacks
    parameter_handlers_["cost.Zl"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NS>(p, res, "Zl", stages);
        };
    parameter_handlers_["cost.Zu"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NS>(p, res, "Zu", stages);
        };
    parameter_handlers_["cost.zl"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NS>(p, res, "zl", stages);
        };
    parameter_handlers_["cost.zu"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            auto stages = range(1, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NS>(p, res, "zu", stages);
        };

    // Terminal Slacks
    parameter_handlers_["cost.Zl_e"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NSN>(p, res, "Zl", std::vector<int>{OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N});
        };
    parameter_handlers_["cost.Zu_e"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NSN>(p, res, "Zu", std::vector<int>{OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N});
        };
    parameter_handlers_["cost.zl_e"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NSN>(p, res, "zl", std::vector<int>{OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N});
        };
    parameter_handlers_["cost.zu_e"] =
        [this](const rclcpp::Parameter& p, rcl_interfaces::msg::SetParametersResult& res) {
            this->update_cost<OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NSN>(p, res, "zu", std::vector<int>{OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N});
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
            const double new_ts = p.as_double();
            const bool unchanged = (new_ts == this->config_.ts);
            if (!this->set_period(new_ts)) {
                res.reason = "Control period 'ts' must be positive and finite.";
                res.successful = false;
                return;
            }
            // Restart timer with the new period
            if (unchanged || !this->is_running()) {
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
    this->declare_parameter("constraints.lbu", std::vector<double>{-1, -1, -1});
    this->declare_parameter("constraints.lbx", std::vector<double>{-1.4, -0.8, -1.6, -1, -1, -1});
    this->declare_parameter("constraints.lh", std::vector<double>{0, 0, 0, 0, 0, 0, 0, 0, 0});
    this->declare_parameter("constraints.lh_e", std::vector<double>{0, 0, 0, 0, 0, 0, 0, 0, 0});
    this->declare_parameter("constraints.lsh", std::vector<double>{0, 0, 0, 0, 0, 0, 0, 0, 0});
    this->declare_parameter("constraints.lsh_e", std::vector<double>{0, 0, 0, 0, 0, 0, 0, 0, 0});
    this->declare_parameter("constraints.ubu", std::vector<double>{1, 1, 1});
    this->declare_parameter("constraints.ubx", std::vector<double>{1.4, 1.6, 1.6, 1, 1, 1});
    this->declare_parameter("constraints.uh", std::vector<double>{1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000});
    this->declare_parameter("constraints.uh_e", std::vector<double>{1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000});
    this->declare_parameter("constraints.ush", std::vector<double>{0, 0, 0, 0, 0, 0, 0, 0, 0});
    this->declare_parameter("constraints.ush_e", std::vector<double>{0, 0, 0, 0, 0, 0, 0, 0, 0});

    // Weights
    this->declare_parameter("cost.W", std::vector<double>{25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.05});
    this->declare_parameter("cost.W_0", std::vector<double>{25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.05});
    this->declare_parameter("cost.W_e", std::vector<double>{25, 0, 0, 0, 25, 0, 0, 0, 25});

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
    this->declare_parameter("ts", static_cast<double>(0.05));
    this->declare_parameter("verbose", false);
}

void PicarArmSafetyNode::load_parameters() {
    double ts = this->config_.ts;
    this->get_parameter("ts", ts);
    this->set_period(ts);
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
        const auto& param_name = param.get_name();
        const auto it = parameter_handlers_.find(param_name);

        if (it != parameter_handlers_.end()) {
            it->second(param, result);
            if (!result.successful) break;
        } else {
            result.reason = "Update for unknown parameter '" + param_name + "' received.";
            result.successful = false;
            break;
        }
    }
    return result;
}

template<size_t N>
void PicarArmSafetyNode::update_constraint(
    const rclcpp::Parameter& param,
    rcl_interfaces::msg::SetParametersResult& result,
    const char* field,
    const std::vector<int>& stages
) {
    const auto& values = param.as_double_array();

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
                result.reason = "Acados solver failed to set constraint field '" + std::string(field) +
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
    const auto& values = param.as_double_array();
    const bool is_weight = std::strcmp(field, "W") == 0;

    std::array<double, N> vec{};
    std::array<double, N * N> mat{};
    double* data_ptr = nullptr;

    if (is_weight) {
        if (values.size() == N) {
            std::copy_n(values.begin(), N, vec.begin());
            mat = diag_from_vec(vec);
        } else if (values.size() == N * N) {
            std::array<double, N * N> row_major{};
            std::copy_n(values.begin(), N * N, row_major.begin());
            mat = row_major_to_col_major<double, N>(row_major);
        } else {
            result.successful = false;
            result.reason = "Cost '" + std::string(param.get_name()) + "' has size " +
                          std::to_string(values.size()) + ", but expected is " + std::to_string(N) +
                          " (diagonal) or " + std::to_string(N * N) + " (full matrix).";
            return;
        }
        data_ptr = mat.data();
        RCLCPP_INFO_STREAM(this->get_logger(), "update cost field '" << field << "' mat(flat) = " << mat);
    } else {
        if (values.size() != N) {
            result.successful = false;
            result.reason = "Cost '" + std::string(param.get_name()) + "' has size " +
                          std::to_string(values.size()) + ", but expected is " + std::to_string(N) + ".";
            return;
        }
        std::copy_n(values.begin(), N, vec.begin());
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
bool PicarArmSafetyNode::set_period(double period_seconds) {
    if (!std::isfinite(period_seconds) || period_seconds <= 0.0) {
        RCLCPP_WARN(this->get_logger(),
            "Control period must be positive and finite.");
        return false;
    }
    if (config_.ts == period_seconds) {
        // Nothing to do
        return true;
    }
    RCLCPP_INFO_STREAM(
        this->get_logger(), "update period 'Ts' = " << period_seconds << "s");
    this->config_.ts = period_seconds;
    return true;
}

void PicarArmSafetyNode::start_control_timer(double period_seconds) {
    if (!this->set_period(period_seconds)) {
        RCLCPP_ERROR(this->get_logger(),
            "Cannot start control timer with invalid period.");
        return;
    }
    // if timer already exists, restart with new period
    if (this->is_running()) {
        RCLCPP_WARN(this->get_logger(), "Control timer already running, restarting...");
        control_timer_->cancel();
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Starting control loop with period " << this->config_.ts << "s.");
    // create timer
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(this->config_.ts));
    control_timer_ = rclcpp::create_timer(
        this->get_node_base_interface(),
        this->get_node_timers_interface(),
        this->get_clock(),
        rclcpp::Duration(period),
        std::bind(&PicarArmSafetyNode::control_loop, this));
}


// --- Acados Solver ---
int PicarArmSafetyNode::prepare_rti_solve() {
    int phase = PREPARATION;
    ocp_nlp_sqp_rti_opts_set(ocp_nlp_config_, ocp_nlp_opts_, "rti_phase", &phase);
    int status = ocp_picar_arm_safety_filter_5c21b6cb_acados_solve(ocp_capsule_);
    if (status != ACADOS_SUCCESS && status != ACADOS_READY) {
        first_solve_ = true;
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Solver failed at preparation phase: %d", status);
    }
    return status;
}

int PicarArmSafetyNode::feedback_rti_solve() {
    int phase = FEEDBACK;
    ocp_nlp_sqp_rti_opts_set(ocp_nlp_config_, ocp_nlp_opts_, "rti_phase", &phase);
    int status = ocp_picar_arm_safety_filter_5c21b6cb_acados_solve(ocp_capsule_);
    if (status != ACADOS_SUCCESS) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Solver failed at feedback phase: %d", status);
    }
    return status;
}
int PicarArmSafetyNode::ocp_solve() {
    int phase = PREPARATION_AND_FEEDBACK;
    ocp_nlp_sqp_rti_opts_set(ocp_nlp_config_, ocp_nlp_opts_, "rti_phase", &phase);
    int status = ocp_picar_arm_safety_filter_5c21b6cb_acados_solve(ocp_capsule_);
    if (status != ACADOS_SUCCESS) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Solver failed with status: %d", status);
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
    for (int i = 1; i < OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N; i++) {
        this->set_yref(yref, i);
    }
}

void PicarArmSafetyNode::set_yref_e(double* yrefN) {
    this->set_yref(yrefN, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
}

void PicarArmSafetyNode::set_ocp_parameter(double* p, size_t np, int stage) {
    int status = ocp_picar_arm_safety_filter_5c21b6cb_acados_update_params(ocp_capsule_, stage, p, np);
    check_acados_status("set_ocp_parameter (p)", stage, status);
}

void PicarArmSafetyNode::set_ocp_parameters(double* p, size_t np) {
    for (int i = 0; i <= OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N; i++) {
        this->set_ocp_parameter(p, np, i);
    }
}

void PicarArmSafetyNode::warmstart_solver_states(double *x0) {
    for (int i = 1; i <= OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N; ++i) {
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
