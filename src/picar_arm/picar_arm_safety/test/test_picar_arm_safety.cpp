#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "picar_arm_safety/node.h"

namespace
{

template <typename Tag>
struct MemberAccessor {
    static inline typename Tag::type ptr = nullptr;
};

template <typename Tag, typename Tag::type M>
struct AccessGrant {
    struct Setter {
        Setter() { MemberAccessor<Tag>::ptr = M; }
    };
    static Setter setter;
};

template <typename Tag, typename Tag::type M>
typename AccessGrant<Tag, M>::Setter AccessGrant<Tag, M>::setter;

struct NlpConfigTag { using type = ocp_nlp_config* picar_arm_safety::PicarArmSafetyNode::*; };
struct NlpDimsTag   { using type = ocp_nlp_dims* picar_arm_safety::PicarArmSafetyNode::*; };
struct NlpInTag     { using type = ocp_nlp_in* picar_arm_safety::PicarArmSafetyNode::*; };
struct ConfigTag    { using type = picar_arm_safety::PicarArmSafetyNodeConfig picar_arm_safety::PicarArmSafetyNode::*; };

template struct AccessGrant<NlpConfigTag, &picar_arm_safety::PicarArmSafetyNode::ocp_nlp_config_>;
template struct AccessGrant<NlpDimsTag,   &picar_arm_safety::PicarArmSafetyNode::ocp_nlp_dims_>;
template struct AccessGrant<NlpInTag,     &picar_arm_safety::PicarArmSafetyNode::ocp_nlp_in_>;
template struct AccessGrant<ConfigTag,    &picar_arm_safety::PicarArmSafetyNode::config_>;

[[maybe_unused]] std::vector<double> perturb_bound(const std::vector<double>& orig, bool is_lower) {
    std::vector<double> out = orig;
    const double sign = is_lower ? -1.0 : 1.0;
    for (size_t i = 0; i < out.size(); ++i) {
        const double delta = std::abs(out[i]) * 0.1 + 1.0 + 0.1 * static_cast<double>(i);
        out[i] += sign * delta;
    }
    return out;
}

[[maybe_unused]] void expect_constraint_matches(
    ocp_nlp_config* cfg,
    ocp_nlp_dims* dims,
    ocp_nlp_in* in,
    const std::vector<int>& stages,
    const char* acados_field,
    const std::vector<double>& expected
) {
    std::vector<double> actual(expected.size(), 0.0);
    for (int stage : stages) {
        ocp_nlp_constraints_model_get(cfg, dims, in, stage, acados_field, actual.data());
        for (size_t i = 0; i < expected.size(); ++i) {
            EXPECT_DOUBLE_EQ(actual[i], expected[i])
                << "Constraint mismatch for '" << acados_field << "' at stage " << stage << ", index " << i;
        }
    }
}

[[maybe_unused]] void expect_weight_col_major_matches(
    ocp_nlp_config* cfg,
    ocp_nlp_dims* dims,
    ocp_nlp_in* in,
    const std::vector<int>& stages,
    size_t dim,
    const std::vector<double>& expected_row_major
) {
    ASSERT_EQ(expected_row_major.size(), dim * dim);
    std::vector<double> actual_col_major(dim * dim, 0.0);
    for (int stage : stages) {
        ASSERT_EQ(ocp_nlp_cost_model_get(cfg, dims, in, stage, "W", actual_col_major.data()), ACADOS_SUCCESS);
        for (size_t r = 0; r < dim; ++r) {
            for (size_t c = 0; c < dim; ++c) {
                EXPECT_DOUBLE_EQ(actual_col_major[c * dim + r], expected_row_major[r * dim + c])
                    << "Weight W mismatch at stage " << stage << " (" << r << ", " << c << ")";
            }
        }
    }
}

[[maybe_unused]] void expect_cost_vec_matches(
    ocp_nlp_config* cfg,
    ocp_nlp_dims* dims,
    ocp_nlp_in* in,
    const std::vector<int>& stages,
    const char* acados_field,
    const std::vector<double>& expected
) {
    std::vector<double> actual(expected.size(), 0.0);
    for (int stage : stages) {
        ASSERT_EQ(ocp_nlp_cost_model_get(cfg, dims, in, stage, acados_field, actual.data()), ACADOS_SUCCESS);
        for (size_t i = 0; i < expected.size(); ++i) {
            EXPECT_DOUBLE_EQ(actual[i], expected[i])
                << "Cost mismatch for '" << acados_field << "' at stage " << stage << ", index " << i;
        }
    }
}

[[maybe_unused]] std::vector<int> make_stage_range(int start_stage, int end_exclusive) {
    std::vector<int> stages;
    for (int s = start_stage; s < end_exclusive; ++s) {
        stages.push_back(s);
    }
    return stages;
}

class OcpNodeSolverInspectionTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    static void TearDownTestSuite() {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    void SetUp() override {
        // Construct without spinning an executor so control_timer_ never fires concurrently
        // and set_parameter() invokes on_parameter_update() synchronously in this thread.
        node_ = std::make_shared<picar_arm_safety::PicarArmSafetyNode>();
    }

    void TearDown() override {
        node_.reset();
    }

    ocp_nlp_config* nlp_config() const { return node_.get()->*MemberAccessor<NlpConfigTag>::ptr; }
    ocp_nlp_dims* nlp_dims() const { return node_.get()->*MemberAccessor<NlpDimsTag>::ptr; }
    ocp_nlp_in* nlp_in() const { return node_.get()->*MemberAccessor<NlpInTag>::ptr; }
    const picar_arm_safety::PicarArmSafetyNodeConfig& node_config() const { return node_.get()->*MemberAccessor<ConfigTag>::ptr; }

    std::shared_ptr<picar_arm_safety::PicarArmSafetyNode> node_;
};

TEST_F(OcpNodeSolverInspectionTest, SolverPointersAndHelperUtilsValid) {
    ASSERT_NE(nlp_config(), nullptr);
    ASSERT_NE(nlp_dims(), nullptr);
    ASSERT_NE(nlp_in(), nullptr);

    const std::array<double, 4> row_major{1.0, 2.0, 3.0, 4.0};
    const auto col_major = ::row_major_to_col_major<double, 2>(row_major);
    const std::array<double, 4> expected_col_major{1.0, 3.0, 2.0, 4.0};
    EXPECT_EQ(col_major, expected_col_major);
}

TEST_F(OcpNodeSolverInspectionTest, CostWeightsAndSlacksStoredAccuratelyAcrossStages) {
    {
        const size_t dim = OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NY0;
        const std::vector<int> stages{0};
        const std::vector<double> init_w{25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.05};
        expect_weight_col_major_matches(nlp_config(), nlp_dims(), nlp_in(), stages, dim, init_w);

        std::vector<double> full_mod = init_w;
        for (size_t r = 0; r < dim; ++r) {
            for (size_t c = 0; c < dim; ++c) {
                full_mod[r * dim + c] += (r == c) ? (1.5 + 0.1 * static_cast<double>(r)) : 0.25;
            }
        }
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("cost.W_0", full_mod)).successful);
        expect_weight_col_major_matches(nlp_config(), nlp_dims(), nlp_in(), stages, dim, full_mod);

        std::vector<double> diag_vals(dim);
        std::vector<double> diag_expanded(dim * dim, 0.0);
        for (size_t i = 0; i < dim; ++i) {
            diag_vals[i] = init_w[i * dim + i] + 2.5 + static_cast<double>(i);
            diag_expanded[i * dim + i] = diag_vals[i];
        }
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("cost.W_0", diag_vals)).successful);
        expect_weight_col_major_matches(nlp_config(), nlp_dims(), nlp_in(), stages, dim, diag_expanded);

        std::vector<double> bad_len(dim * dim + 1, 1.0);
        EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("cost.W_0", bad_len)).successful);
        expect_weight_col_major_matches(nlp_config(), nlp_dims(), nlp_in(), stages, dim, diag_expanded);
    }
    {
        const size_t dim = OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NY;
        const std::vector<int> stages = make_stage_range(1, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
        const std::vector<double> init_w{25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.05};
        expect_weight_col_major_matches(nlp_config(), nlp_dims(), nlp_in(), stages, dim, init_w);

        std::vector<double> full_mod = init_w;
        for (size_t r = 0; r < dim; ++r) {
            for (size_t c = 0; c < dim; ++c) {
                full_mod[r * dim + c] += (r == c) ? (1.5 + 0.1 * static_cast<double>(r)) : 0.25;
            }
        }
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("cost.W", full_mod)).successful);
        expect_weight_col_major_matches(nlp_config(), nlp_dims(), nlp_in(), stages, dim, full_mod);

        std::vector<double> diag_vals(dim);
        std::vector<double> diag_expanded(dim * dim, 0.0);
        for (size_t i = 0; i < dim; ++i) {
            diag_vals[i] = init_w[i * dim + i] + 2.5 + static_cast<double>(i);
            diag_expanded[i * dim + i] = diag_vals[i];
        }
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("cost.W", diag_vals)).successful);
        expect_weight_col_major_matches(nlp_config(), nlp_dims(), nlp_in(), stages, dim, diag_expanded);

        std::vector<double> bad_len(dim * dim + 1, 1.0);
        EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("cost.W", bad_len)).successful);
        expect_weight_col_major_matches(nlp_config(), nlp_dims(), nlp_in(), stages, dim, diag_expanded);
    }
    {
        const size_t dim = OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_NYN;
        const std::vector<int> stages{ OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N };
        const std::vector<double> init_w{25, 0, 0, 0, 25, 0, 0, 0, 25};
        expect_weight_col_major_matches(nlp_config(), nlp_dims(), nlp_in(), stages, dim, init_w);

        std::vector<double> full_mod = init_w;
        for (size_t r = 0; r < dim; ++r) {
            for (size_t c = 0; c < dim; ++c) {
                full_mod[r * dim + c] += (r == c) ? (1.5 + 0.1 * static_cast<double>(r)) : 0.25;
            }
        }
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("cost.W_e", full_mod)).successful);
        expect_weight_col_major_matches(nlp_config(), nlp_dims(), nlp_in(), stages, dim, full_mod);

        std::vector<double> diag_vals(dim);
        std::vector<double> diag_expanded(dim * dim, 0.0);
        for (size_t i = 0; i < dim; ++i) {
            diag_vals[i] = init_w[i * dim + i] + 2.5 + static_cast<double>(i);
            diag_expanded[i * dim + i] = diag_vals[i];
        }
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("cost.W_e", diag_vals)).successful);
        expect_weight_col_major_matches(nlp_config(), nlp_dims(), nlp_in(), stages, dim, diag_expanded);

        std::vector<double> bad_len(dim * dim + 1, 1.0);
        EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("cost.W_e", bad_len)).successful);
        expect_weight_col_major_matches(nlp_config(), nlp_dims(), nlp_in(), stages, dim, diag_expanded);
    }
    {
        const std::vector<int> stages = make_stage_range(1, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
        const std::vector<double> init_z{1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "Zl", init_z);

        std::vector<double> mod_z = init_z;
        for (size_t i = 0; i < mod_z.size(); ++i) {
            mod_z[i] += 3.5 + 0.5 * static_cast<double>(i);
        }
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("cost.Zl", mod_z)).successful);
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "Zl", mod_z);

        std::vector<double> bad_z(mod_z.size() + 1, 0.0);
        EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("cost.Zl", bad_z)).successful);
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "Zl", mod_z);
    }
    {
        const std::vector<int> stages = make_stage_range(1, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
        const std::vector<double> init_z{0, 0, 0, 0, 0, 0, 0, 0, 0};
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "Zu", init_z);

        std::vector<double> mod_z = init_z;
        for (size_t i = 0; i < mod_z.size(); ++i) {
            mod_z[i] += 3.5 + 0.5 * static_cast<double>(i);
        }
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("cost.Zu", mod_z)).successful);
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "Zu", mod_z);

        std::vector<double> bad_z(mod_z.size() + 1, 0.0);
        EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("cost.Zu", bad_z)).successful);
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "Zu", mod_z);
    }
    {
        const std::vector<int> stages = make_stage_range(1, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
        const std::vector<double> init_z{100, 100, 100, 100, 100, 100, 100, 100, 100};
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "zl", init_z);

        std::vector<double> mod_z = init_z;
        for (size_t i = 0; i < mod_z.size(); ++i) {
            mod_z[i] += 3.5 + 0.5 * static_cast<double>(i);
        }
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("cost.zl", mod_z)).successful);
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "zl", mod_z);

        std::vector<double> bad_z(mod_z.size() + 1, 0.0);
        EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("cost.zl", bad_z)).successful);
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "zl", mod_z);
    }
    {
        const std::vector<int> stages = make_stage_range(1, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
        const std::vector<double> init_z{0, 0, 0, 0, 0, 0, 0, 0, 0};
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "zu", init_z);

        std::vector<double> mod_z = init_z;
        for (size_t i = 0; i < mod_z.size(); ++i) {
            mod_z[i] += 3.5 + 0.5 * static_cast<double>(i);
        }
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("cost.zu", mod_z)).successful);
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "zu", mod_z);

        std::vector<double> bad_z(mod_z.size() + 1, 0.0);
        EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("cost.zu", bad_z)).successful);
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "zu", mod_z);
    }
    {
        const std::vector<int> stages{ OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N };
        const std::vector<double> init_z{1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "Zl", init_z);

        std::vector<double> mod_z = init_z;
        for (size_t i = 0; i < mod_z.size(); ++i) {
            mod_z[i] += 3.5 + 0.5 * static_cast<double>(i);
        }
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("cost.Zl_e", mod_z)).successful);
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "Zl", mod_z);

        std::vector<double> bad_z(mod_z.size() + 1, 0.0);
        EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("cost.Zl_e", bad_z)).successful);
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "Zl", mod_z);
    }
    {
        const std::vector<int> stages{ OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N };
        const std::vector<double> init_z{0, 0, 0, 0, 0, 0, 0, 0, 0};
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "Zu", init_z);

        std::vector<double> mod_z = init_z;
        for (size_t i = 0; i < mod_z.size(); ++i) {
            mod_z[i] += 3.5 + 0.5 * static_cast<double>(i);
        }
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("cost.Zu_e", mod_z)).successful);
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "Zu", mod_z);

        std::vector<double> bad_z(mod_z.size() + 1, 0.0);
        EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("cost.Zu_e", bad_z)).successful);
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "Zu", mod_z);
    }
    {
        const std::vector<int> stages{ OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N };
        const std::vector<double> init_z{100, 100, 100, 100, 100, 100, 100, 100, 100};
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "zl", init_z);

        std::vector<double> mod_z = init_z;
        for (size_t i = 0; i < mod_z.size(); ++i) {
            mod_z[i] += 3.5 + 0.5 * static_cast<double>(i);
        }
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("cost.zl_e", mod_z)).successful);
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "zl", mod_z);

        std::vector<double> bad_z(mod_z.size() + 1, 0.0);
        EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("cost.zl_e", bad_z)).successful);
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "zl", mod_z);
    }
    {
        const std::vector<int> stages{ OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N };
        const std::vector<double> init_z{0, 0, 0, 0, 0, 0, 0, 0, 0};
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "zu", init_z);

        std::vector<double> mod_z = init_z;
        for (size_t i = 0; i < mod_z.size(); ++i) {
            mod_z[i] += 3.5 + 0.5 * static_cast<double>(i);
        }
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("cost.zu_e", mod_z)).successful);
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "zu", mod_z);

        std::vector<double> bad_z(mod_z.size() + 1, 0.0);
        EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("cost.zu_e", bad_z)).successful);
        expect_cost_vec_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "zu", mod_z);
    }
}

TEST_F(OcpNodeSolverInspectionTest, ConstraintsUpdatedAndRejectedAtomicallyAcrossStages) {
    // Initial non-slack constraints (stage 0)

    // Stage non-slack constraints (bu & g cover stages 0..N-1; bx, h, phi cover stages 1..N-1)
    {
        const std::vector<int> stages = make_stage_range(0, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
        const std::vector<double> init_val{-1, -1, -1};
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "lbu", init_val);

        const auto mod_val = perturb_bound(init_val, true);
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("constraints.lbu", mod_val)).successful);
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "lbu", mod_val);

        std::vector<double> bad_val(mod_val.size() + 1, 0.0);
        EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("constraints.lbu", bad_val)).successful);
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "lbu", mod_val);
    }
    {
        const std::vector<int> stages = make_stage_range(1, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
        const std::vector<double> init_val{-1.4, -0.8, -1.6, -1, -1, -1};
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "lbx", init_val);

        const auto mod_val = perturb_bound(init_val, true);
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("constraints.lbx", mod_val)).successful);
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "lbx", mod_val);

        std::vector<double> bad_val(mod_val.size() + 1, 0.0);
        EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("constraints.lbx", bad_val)).successful);
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "lbx", mod_val);
    }
    {
        const std::vector<int> stages = make_stage_range(1, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
        const std::vector<double> init_val{0, 0, 0, 0, 0, 0, 0, 0, 0};
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "lh", init_val);

        const auto mod_val = perturb_bound(init_val, true);
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("constraints.lh", mod_val)).successful);
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "lh", mod_val);

        std::vector<double> bad_val(mod_val.size() + 1, 0.0);
        EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("constraints.lh", bad_val)).successful);
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "lh", mod_val);
    }
    {
        const std::vector<int> stages = make_stage_range(0, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
        const std::vector<double> init_val{1, 1, 1};
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "ubu", init_val);

        const auto mod_val = perturb_bound(init_val, false);
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("constraints.ubu", mod_val)).successful);
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "ubu", mod_val);

        std::vector<double> bad_val(mod_val.size() + 1, 0.0);
        EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("constraints.ubu", bad_val)).successful);
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "ubu", mod_val);
    }
    {
        const std::vector<int> stages = make_stage_range(1, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
        const std::vector<double> init_val{1.4, 1.6, 1.6, 1, 1, 1};
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "ubx", init_val);

        const auto mod_val = perturb_bound(init_val, false);
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("constraints.ubx", mod_val)).successful);
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "ubx", mod_val);

        std::vector<double> bad_val(mod_val.size() + 1, 0.0);
        EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("constraints.ubx", bad_val)).successful);
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "ubx", mod_val);
    }
    {
        const std::vector<int> stages = make_stage_range(1, OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N);
        const std::vector<double> init_val{1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000};
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "uh", init_val);

        const auto mod_val = perturb_bound(init_val, false);
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("constraints.uh", mod_val)).successful);
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "uh", mod_val);

        std::vector<double> bad_val(mod_val.size() + 1, 0.0);
        EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("constraints.uh", bad_val)).successful);
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, "uh", mod_val);
    }

    // Terminal non-slack constraints (stage N)
    {
        const std::vector<int> stages{ OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N };
        const std::vector<double> init_val{0, 0, 0, 0, 0, 0, 0, 0, 0};
        const char* acados_field = "lh";
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, acados_field, init_val);

        const auto mod_val = perturb_bound(init_val, true);
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("constraints.lh_e", mod_val)).successful);
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, acados_field, mod_val);

        std::vector<double> bad_val(mod_val.size() + 1, 0.0);
        EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("constraints.lh_e", bad_val)).successful);
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, acados_field, mod_val);
    }
    {
        const std::vector<int> stages{ OCP_PICAR_ARM_SAFETY_FILTER_5C21B6CB_N };
        const std::vector<double> init_val{1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000};
        const char* acados_field = "uh";
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, acados_field, init_val);

        const auto mod_val = perturb_bound(init_val, false);
        EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("constraints.uh_e", mod_val)).successful);
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, acados_field, mod_val);

        std::vector<double> bad_val(mod_val.size() + 1, 0.0);
        EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("constraints.uh_e", bad_val)).successful);
        expect_constraint_matches(nlp_config(), nlp_dims(), nlp_in(), stages, acados_field, mod_val);
    }
}

TEST_F(OcpNodeSolverInspectionTest, RosConfigAndOptionsEnforceValidation) {
    const double default_ts = static_cast<double>(0.05);
    EXPECT_DOUBLE_EQ(node_config().ts, default_ts);
    EXPECT_EQ(node_config().threads, 1);
    EXPECT_FALSE(node_config().verbose);

    EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("ts", default_ts)).successful);
    EXPECT_DOUBLE_EQ(node_config().ts, default_ts);

    const double new_ts = default_ts * 2.0;
    EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("ts", new_ts)).successful);
    EXPECT_DOUBLE_EQ(node_config().ts, new_ts);

    EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("ts", 0.0)).successful);
    EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("ts", -0.5)).successful);
    EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("ts", std::numeric_limits<double>::quiet_NaN())).successful);
    EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("ts", std::numeric_limits<double>::infinity())).successful);
    EXPECT_DOUBLE_EQ(node_config().ts, new_ts);

    EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("verbose", true)).successful);
    EXPECT_TRUE(node_config().verbose);
    EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("verbose", false)).successful);
    EXPECT_FALSE(node_config().verbose);

    const int default_print_level = 0;
    EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("solver_options.print_level", default_print_level == 0 ? 1 : 0)).successful);
    EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter("solver_options.print_level", default_print_level)).successful);

    EXPECT_FALSE(node_->has_parameter("nonexistent_parameter"));
    EXPECT_THROW(
        node_->set_parameter(rclcpp::Parameter("nonexistent_parameter", 1.0)),
        rclcpp::exceptions::ParameterNotDeclaredException
    );
    EXPECT_THROW(
        node_->set_parameters({rclcpp::Parameter("nonexistent_parameter", 1.0)}),
        rclcpp::exceptions::ParameterNotDeclaredException
    );
}

} // namespace
