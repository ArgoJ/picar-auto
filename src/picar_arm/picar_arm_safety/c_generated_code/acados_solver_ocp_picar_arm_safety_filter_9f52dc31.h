/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * Licensed under the 2-Clause BSD License.
 */

#ifndef ACADOS_SOLVER_ocp_picar_arm_safety_filter_9f52dc31_H_
#define ACADOS_SOLVER_ocp_picar_arm_safety_filter_9f52dc31_H_

#include "acados/utils/types.h"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NX     6
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NZ     0
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NU     3
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NP     7
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NP_GLOBAL     0
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NBX    6
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NBX0   6
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NBU    3
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NSBX   0
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NSBU   0
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NSH    9
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NSH0   0
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NSG    0
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NSPHI  0
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NSHN   9
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NSGN   0
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NSPHIN 0
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NSPHI0 0
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NSBXN  0
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NS     9
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NS0    0
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NSN    9
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NG     0
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NBXN   0
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NGN    0
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NY0    9
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NY     9
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NYN    3
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_N      10
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NH     9
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NHN    9
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NH0    0
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NPHI0  0
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NPHI   0
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NPHIN  0
#define OCP_PICAR_ARM_SAFETY_FILTER_9F52DC31_NR     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct ocp_picar_arm_safety_filter_9f52dc31_solver_capsule
{
    // acados objects
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_out *sens_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
    ocp_nlp_plan_t *nlp_solver_plan;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;

    // number of expected runtime parameters
    unsigned int nlp_np;

    /* external functions */

    // dynamics

    external_function_external_param_casadi *expl_vde_forw;
    external_function_external_param_casadi *expl_vde_forw_p;
    external_function_external_param_casadi *expl_ode_fun;
    external_function_external_param_casadi *expl_vde_adj;




    // cost

    external_function_external_param_casadi *cost_y_fun;
    external_function_external_param_casadi *cost_y_fun_jac_ut_xt;



    external_function_external_param_casadi cost_y_0_fun;
    external_function_external_param_casadi cost_y_0_fun_jac_ut_xt;



    external_function_external_param_casadi cost_y_e_fun;
    external_function_external_param_casadi cost_y_e_fun_jac_ut_xt;


    // constraints
    external_function_external_param_casadi *nl_constr_h_fun_jac;
    external_function_external_param_casadi *nl_constr_h_fun;









    external_function_external_param_casadi nl_constr_h_e_fun_jac;
    external_function_external_param_casadi nl_constr_h_e_fun;




} ocp_picar_arm_safety_filter_9f52dc31_solver_capsule;

ACADOS_SYMBOL_EXPORT ocp_picar_arm_safety_filter_9f52dc31_solver_capsule * ocp_picar_arm_safety_filter_9f52dc31_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int ocp_picar_arm_safety_filter_9f52dc31_acados_free_capsule(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int ocp_picar_arm_safety_filter_9f52dc31_acados_create(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int ocp_picar_arm_safety_filter_9f52dc31_acados_reset(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule* capsule, int reset_qp_solver_mem, int reset_numerical_values, int reset_solver_options, int reset_x_to_x0_bar);

/**
 * Generic version of ocp_picar_arm_safety_filter_9f52dc31_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
ACADOS_SYMBOL_EXPORT int ocp_picar_arm_safety_filter_9f52dc31_acados_create_with_discretization(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
ACADOS_SYMBOL_EXPORT int ocp_picar_arm_safety_filter_9f52dc31_acados_update_time_steps(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule * capsule, int N, double* new_time_steps);
/**
 * This function is used for updating an already initialized solver with a different number of qp_cond_N.
 */
ACADOS_SYMBOL_EXPORT int ocp_picar_arm_safety_filter_9f52dc31_acados_update_qp_solver_cond_N(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int ocp_picar_arm_safety_filter_9f52dc31_acados_update_params(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule * capsule, int stage, double *value, int np);
ACADOS_SYMBOL_EXPORT int ocp_picar_arm_safety_filter_9f52dc31_acados_update_params_sparse(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule * capsule, int stage, int *idx, double *p, int n_update);
ACADOS_SYMBOL_EXPORT int ocp_picar_arm_safety_filter_9f52dc31_acados_set_p_global_and_precompute_dependencies(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule* capsule, double* data, int data_len);

ACADOS_SYMBOL_EXPORT int ocp_picar_arm_safety_filter_9f52dc31_acados_solve(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int ocp_picar_arm_safety_filter_9f52dc31_acados_setup_qp_matrices_and_factorize(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule* capsule);



ACADOS_SYMBOL_EXPORT int ocp_picar_arm_safety_filter_9f52dc31_acados_free(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void ocp_picar_arm_safety_filter_9f52dc31_acados_print_stats(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int ocp_picar_arm_safety_filter_9f52dc31_acados_custom_update(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule* capsule, double* data, int data_len);

ACADOS_SYMBOL_EXPORT ocp_nlp_in *ocp_picar_arm_safety_filter_9f52dc31_acados_get_nlp_in(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *ocp_picar_arm_safety_filter_9f52dc31_acados_get_nlp_out(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *ocp_picar_arm_safety_filter_9f52dc31_acados_get_sens_out(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver *ocp_picar_arm_safety_filter_9f52dc31_acados_get_nlp_solver(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config *ocp_picar_arm_safety_filter_9f52dc31_acados_get_nlp_config(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void *ocp_picar_arm_safety_filter_9f52dc31_acados_get_nlp_opts(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims *ocp_picar_arm_safety_filter_9f52dc31_acados_get_nlp_dims(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t *ocp_picar_arm_safety_filter_9f52dc31_acados_get_nlp_plan(ocp_picar_arm_safety_filter_9f52dc31_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_ocp_picar_arm_safety_filter_9f52dc31_H_
