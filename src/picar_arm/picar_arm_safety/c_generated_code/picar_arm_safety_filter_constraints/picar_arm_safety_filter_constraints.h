/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * Licensed under the 2-Clause BSD License.
 */

#ifndef picar_arm_safety_filter_CONSTRAINTS
#define picar_arm_safety_filter_CONSTRAINTS

#ifdef __cplusplus
extern "C" {
#endif









int picar_arm_safety_filter_constr_h_fun_jac_uxt_zt(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int picar_arm_safety_filter_constr_h_fun_jac_uxt_zt_work(int *, int *, int *, int *);
const int *picar_arm_safety_filter_constr_h_fun_jac_uxt_zt_sparsity_in(int);
const int *picar_arm_safety_filter_constr_h_fun_jac_uxt_zt_sparsity_out(int);
int picar_arm_safety_filter_constr_h_fun_jac_uxt_zt_n_in(void);
int picar_arm_safety_filter_constr_h_fun_jac_uxt_zt_n_out(void);

int picar_arm_safety_filter_constr_h_fun(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int picar_arm_safety_filter_constr_h_fun_work(int *, int *, int *, int *);
const int *picar_arm_safety_filter_constr_h_fun_sparsity_in(int);
const int *picar_arm_safety_filter_constr_h_fun_sparsity_out(int);
int picar_arm_safety_filter_constr_h_fun_n_in(void);
int picar_arm_safety_filter_constr_h_fun_n_out(void);














int picar_arm_safety_filter_constr_h_e_fun_jac_uxt_zt(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int picar_arm_safety_filter_constr_h_e_fun_jac_uxt_zt_work(int *, int *, int *, int *);
const int *picar_arm_safety_filter_constr_h_e_fun_jac_uxt_zt_sparsity_in(int);
const int *picar_arm_safety_filter_constr_h_e_fun_jac_uxt_zt_sparsity_out(int);
int picar_arm_safety_filter_constr_h_e_fun_jac_uxt_zt_n_in(void);
int picar_arm_safety_filter_constr_h_e_fun_jac_uxt_zt_n_out(void);

int picar_arm_safety_filter_constr_h_e_fun(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int picar_arm_safety_filter_constr_h_e_fun_work(int *, int *, int *, int *);
const int *picar_arm_safety_filter_constr_h_e_fun_sparsity_in(int);
const int *picar_arm_safety_filter_constr_h_e_fun_sparsity_out(int);
int picar_arm_safety_filter_constr_h_e_fun_n_in(void);
int picar_arm_safety_filter_constr_h_e_fun_n_out(void);











#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // picar_arm_safety_filter_CONSTRAINTS
