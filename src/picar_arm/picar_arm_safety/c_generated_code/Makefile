#
# Copyright (c) The acados authors.
#
# This file is part of acados.
#
# Licensed under the 2-Clause BSD License.





# define sources and use make's implicit rules to generate object files (*.o)

# model
MODEL_SRC=

MODEL_SRC+= picar_arm_safety_filter_model/picar_arm_safety_filter_expl_ode_fun.c
MODEL_SRC+= picar_arm_safety_filter_model/picar_arm_safety_filter_expl_vde_forw.c
MODEL_SRC+= picar_arm_safety_filter_model/picar_arm_safety_filter_expl_vde_adj.c

MODEL_OBJ := $(MODEL_SRC:.c=.o)
# optimal control problem - mostly CasADi exports
OCP_SRC=


OCP_SRC+= picar_arm_safety_filter_constraints/picar_arm_safety_filter_constr_h_fun_jac_uxt_zt.c
OCP_SRC+= picar_arm_safety_filter_constraints/picar_arm_safety_filter_constr_h_fun.c
OCP_SRC+= picar_arm_safety_filter_constraints/picar_arm_safety_filter_constr_h_e_fun_jac_uxt_zt.c
OCP_SRC+= picar_arm_safety_filter_constraints/picar_arm_safety_filter_constr_h_e_fun.c

OCP_SRC+= acados_solver_ocp_picar_arm_safety_filter_809ed34e.c

OCP_OBJ := $(OCP_SRC:.c=.o)
# for sim solver
SIM_SRC= acados_sim_solver_ocp_picar_arm_safety_filter_809ed34e.c
SIM_OBJ := $(SIM_SRC:.c=.o)

# for target example_sim
EX_SIM_SRC= main_sim_ocp_picar_arm_safety_filter_809ed34e.c
EX_SIM_OBJ := $(EX_SIM_SRC:.c=.o)
EX_SIM_EXE := $(EX_SIM_SRC:.c=)
# for target example
EX_SRC= main_ocp_picar_arm_safety_filter_809ed34e.c
EX_OBJ := $(EX_SRC:.c=.o)
EX_EXE := $(EX_SRC:.c=)


# combine model, (potentially) sim and ocp object files
OBJ=
OBJ+= $(MODEL_OBJ)
OBJ+= $(SIM_OBJ)
OBJ+= $(OCP_OBJ)

EXTERNAL_DIR=
EXTERNAL_LIB=

INCLUDE_PATH = /home/pirob/acados/include
LIB_PATH = /home/pirob/acados/lib

# preprocessor flags for make's implicit rules
CPPFLAGS+= -I$(INCLUDE_PATH)
CPPFLAGS+= -I$(INCLUDE_PATH)/acados
CPPFLAGS+= -I$(INCLUDE_PATH)/blasfeo/include
CPPFLAGS+= -I$(INCLUDE_PATH)/hpipm/include


# define the c-compiler flags for make's implicit rules
CFLAGS = -fPIC -std=c99   -O2

# # Debugging
# CFLAGS += -g3 -fno-diagnostics-show-line-numbers -g

# linker flags
LDFLAGS+= -L$(LIB_PATH)


# link to libraries
LDLIBS+= -lacados
LDLIBS+= -lhpipm
LDLIBS+= -lblasfeo
LDLIBS+= -lm
LDLIBS+= 

# libraries
LIBACADOS_SOLVER=libacados_solver_ocp_picar_arm_safety_filter_809ed34e.so
LIBACADOS_OCP_SOLVER=libacados_ocp_solver_ocp_picar_arm_safety_filter_809ed34e.so
LIBACADOS_SIM_SOLVER=lib$(SIM_SRC:.c=.so)

# virtual targets
.PHONY : all clean

	all: clean example_sim example
shared_lib: bundled_shared_lib ocp_shared_lib sim_shared_lib

# some linker targets
example: $(EX_OBJ) $(OBJ)
	$(CC) $^ -o $(EX_EXE) $(LDFLAGS) $(LDLIBS)

example_sim: $(EX_SIM_OBJ) $(MODEL_OBJ) $(SIM_OBJ)
	$(CC) $^ -o $(EX_SIM_EXE) $(LDFLAGS) $(LDLIBS)

sim_shared_lib: $(SIM_OBJ) $(MODEL_OBJ)
	$(CC) -shared $^ -o $(LIBACADOS_SIM_SOLVER) $(LDFLAGS) $(LDLIBS)
bundled_shared_lib: $(OBJ)
	$(CC) -shared $^ -o $(LIBACADOS_SOLVER) $(LDFLAGS) $(LDLIBS)

ocp_shared_lib: $(OCP_OBJ) $(MODEL_OBJ)
	$(CC) -shared $^ -o $(LIBACADOS_OCP_SOLVER) $(LDFLAGS) $(LDLIBS) \
	-L$(EXTERNAL_DIR) -l$(EXTERNAL_LIB)


# Cython targets
ocp_cython_c: ocp_shared_lib
	cython \
	-o acados_ocp_solver_pyx.c \
	-I $(INCLUDE_PATH)/../interfaces/acados_template/acados_template \
	$(INCLUDE_PATH)/../interfaces/acados_template/acados_template/acados_ocp_solver_pyx.pyx \
	-I /home/pirob/ros2_ws/src/picar_arm/picar_arm_safety/c_generated_code \

ocp_cython_o: ocp_cython_c
	$(CC) $(ACADOS_FLAGS) -c -O2 \
	-fPIC \
	-DNPY_NO_DEPRECATED_API=NPY_1_7_API_VERSION \
	-o acados_ocp_solver_pyx.o \
	-I $(INCLUDE_PATH)/blasfeo/include/ \
	-I $(INCLUDE_PATH)/hpipm/include/ \
	-I $(INCLUDE_PATH) \
	-I /usr/lib/aarch64-linux-gnu/python3-numpy/numpy/_core/include \
	-I /usr/include/python3.14 \
	acados_ocp_solver_pyx.c \

ocp_cython: ocp_cython_o
	$(CC) $(ACADOS_FLAGS) -shared \
	-o acados_ocp_solver_pyx.so \
	-Wl,-rpath=$(LIB_PATH) \
	acados_ocp_solver_pyx.o \
	$(abspath .)/libacados_ocp_solver_ocp_picar_arm_safety_filter_809ed34e.so \
	$(LDFLAGS) $(LDLIBS)


# Sim Cython targets
sim_cython_c: sim_shared_lib
	cython \
	-o acados_sim_solver_pyx.c \
	-I $(INCLUDE_PATH)/../interfaces/acados_template/acados_template \
	$(INCLUDE_PATH)/../interfaces/acados_template/acados_template/acados_sim_solver_pyx.pyx \
	-I /home/pirob/ros2_ws/src/picar_arm/picar_arm_safety/c_generated_code \

sim_cython_o: sim_cython_c
	$(CC) $(ACADOS_FLAGS) -c -O2 \
	-fPIC \
	-DNPY_NO_DEPRECATED_API=NPY_1_7_API_VERSION \
	-o acados_sim_solver_pyx.o \
	-I $(INCLUDE_PATH)/blasfeo/include/ \
	-I $(INCLUDE_PATH)/hpipm/include/ \
	-I $(INCLUDE_PATH) \
	-I /usr/lib/aarch64-linux-gnu/python3-numpy/numpy/_core/include \
	-I /usr/include/python3.14 \
	acados_sim_solver_pyx.c \

sim_cython: sim_cython_o
	$(CC) $(ACADOS_FLAGS) -shared \
	-o acados_sim_solver_pyx.so \
	-Wl,-rpath=$(LIB_PATH) \
	acados_sim_solver_pyx.o \
	$(abspath .)/libacados_sim_solver_ocp_picar_arm_safety_filter_809ed34e.so \
	$(LDFLAGS) $(LDLIBS)

clean:
	$(RM) $(OBJ) $(EX_OBJ) $(EX_SIM_OBJ)
	$(RM) $(LIBACADOS_SOLVER) $(LIBACADOS_OCP_SOLVER) $(LIBACADOS_SIM_SOLVER)
	$(RM) $(EX_EXE) $(EX_SIM_EXE)
clean_ocp_shared_lib:
	$(RM) $(LIBACADOS_OCP_SOLVER)
	$(RM) $(OCP_OBJ)

clean_ocp_cython:
	$(RM) libacados_ocp_solver_ocp_picar_arm_safety_filter_809ed34e.so
	$(RM) acados_solver_ocp_picar_arm_safety_filter_809ed34e.o
	$(RM) acados_ocp_solver_pyx.so
	$(RM) acados_ocp_solver_pyx.o

clean_sim_cython:
	$(RM) libacados_sim_solver_ocp_picar_arm_safety_filter_809ed34e.so
	$(RM) acados_sim_solver_ocp_picar_arm_safety_filter_809ed34e.o
	$(RM) acados_sim_solver_pyx.so
	$(RM) acados_sim_solver_pyx.o
