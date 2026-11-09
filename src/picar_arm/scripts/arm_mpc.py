#!/usr/bin/env python3
"""
Acados OCP & ROS 2 Node Generator für den PiCar-Arm Sicherheitsfilter (3-DOF + Greifer).

Dieses Skript:
1. Definiert die 3D-Arm-Kinematik (4 Glieder + Zahnrad-Greifer) in CasADi.
2. Formuliert das Optimal Control Problem (OCP) mit LINEAR_LS Kosten + Slack-Variablen:
   - Bodenschutz: Greiferspitze, Handgelenk und Ellenbogen bleiben über z_min.
   - 1x Verbotene 3D-Box (Superquadrik 4. Ordnung): Schützt das gesamte PiCar-Chassis,
     Reifen und Aufbauten vor Selbstkollision durch den Arm.
3. Exportiert den C-Code sowie die fertigen ROS 2 Subpackages:
   - picar_arm_safety_interface (Custom Messages)
   - picar_arm_safety (C++ ROS 2 Solver Node)
   direkt als Subpackages in dieses Paketverzeichnis.

Ausführung:
    ~/.aenv/bin/python src/picar_arm/scripts/arm_mpc.py --build
"""

import os
import argparse
import numpy as np
import casadi as ca

from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from acados_template.ros2.ocp_node import AcadosOcpRosOptions


def export_picar_arm_safety_model() -> AcadosModel:
    """
    Erstellt das CasADi-Modell für den 3-DOF PiCar-Arm (Pan, Tilt, Lift)
    inkl. Greifer-Geometrie, Bodenschutz und 1x 3D-Box-Ausschluss (Fahrzeug-Chassis).
    """
    model_name = "picar_arm_safety_filter"

    # =========================================================================
    # 1. ZUSTANDSVEKTOR (x in R^6) & STELLGRÖSSEN (u in R^3)
    # =========================================================================
    q1 = ca.SX.sym("q1")  # CH1: Basis-Drehung (Pan) [rad]
    q2 = ca.SX.sym("q2")  # CH2: Schulter-Neigung (Tilt) [rad]
    q3 = ca.SX.sym("q3")  # CH3: Ellenbogen-Winkel (Lift) [rad]
    q = ca.vertcat(q1, q2, q3)

    v1 = ca.SX.sym("v1")  # Winkelgeschwindigkeit Achse 1 [rad/s]
    v2 = ca.SX.sym("v2")  # Winkelgeschwindigkeit Achse 2 [rad/s]
    v3 = ca.SX.sym("v3")  # Winkelgeschwindigkeit Achse 3 [rad/s]
    v = ca.vertcat(v1, v2, v3)

    x = ca.vertcat(q, v)
    xdot = ca.SX.sym("xdot", 6, 1)

    vc1 = ca.SX.sym("vc1")  # Soll-Geschwindigkeit Achse 1 [rad/s]
    vc2 = ca.SX.sym("vc2")  # Soll-Geschwindigkeit Achse 2 [rad/s]
    vc3 = ca.SX.sym("vc3")  # Soll-Geschwindigkeit Achse 3 [rad/s]
    u = ca.vertcat(vc1, vc2, vc3)

    # =========================================================================
    # 2. LAUFZEIT-PARAMETER (p in R^7) FÜR GREIFER & VERBOTENE BOX
    # =========================================================================
    # Index 0: Aktueller Öffnungswinkel des Greifer-Servos (0 = zu, >0 = offen)
    q4_param = ca.SX.sym("q4_gripper", 1)

    # Index 1..3: Zentrum der verbotenen Box im Car-Frame [m]
    box_c = ca.SX.sym("box_center", 3)  # [x_c, y_c, z_c]

    # Index 4..6: Halbachsen / halbe Kantenlängen der Box [m]
    box_d = ca.SX.sym("box_half_dims", 3)  # [d_x, d_y, d_z]

    p = ca.vertcat(q4_param, box_c, box_d)

    # =========================================================================
    # 3. SYSTEMDYNAMIK MIT SERVO-LAG (PT1-Verzögerung)
    # =========================================================================
    tau_v = ca.DM([0.08, 0.08, 0.08])  # Zeitkonstanten der 3 Servos [s]

    f_expl = ca.vertcat(
        v,
        (u - v) / tau_v
    )

    # =========================================================================
    # 4. KINEMATISCHE KETTE AB CAR-URSPRUNG (base_link = [0, 0, 0])
    #    Exakt abgestimmt auf arm_handler_node.hpp
    # =========================================================================
    p_shoulder = ca.DM([0.08, 0.0, 0.175])  # Schultergelenk-Position [m]

    L2 = 0.106      # Oberarmlänge (Schulter-Servo bis Ellenbogen-Servo) [m]
    L3 = 0.109      # Unterarmlänge (Ellenbogen-Servo bis Greifer-Servo) [m]
    L4 = 0.075      # Länge der rotierenden Greiferfinger [m]
    d_gear = 0.015  # Halber Abstand zwischen den beiden Zahnrad-Drehpunkten [m]

    p_elbow = p_shoulder + L2 * ca.vertcat(
        ca.cos(q1) * ca.cos(q2),
        ca.sin(q1) * ca.cos(q2),
        ca.sin(q2)
    )

    pitch_L3 = q2 + q3
    e_fwd = ca.vertcat(
        ca.cos(q1) * ca.cos(pitch_L3),
        ca.sin(q1) * ca.cos(pitch_L3),
        ca.sin(pitch_L3)
    )
    e_side = ca.vertcat(
        -ca.sin(q1),
        ca.cos(q1),
        0.0
    )

    p_mid_forearm = p_elbow + (0.5 * L3) * e_fwd
    p_gripper_servo = p_elbow + L3 * e_fwd
    p_ee = p_gripper_servo + (L4 * ca.cos(q4_param)) * e_fwd

    half_width = d_gear + L4 * ca.sin(q4_param)
    p_jaw_left = p_ee + half_width * e_side
    p_jaw_right = p_ee - half_width * e_side

    # =========================================================================
    # 5. NICHTLINEARE SICHERHEITS-CONSTRAINTS h(x, p) >= 0
    # =========================================================================
    constraints = []

    # --- A. Bodenschutz (3 Constraints) ---
    z_min_ee = 0.015      # Greiferspitze mind. 1.5 cm über Boden
    z_min_grip = 0.025    # Greiferservo mind. 2.5 cm über Boden
    z_min_elbow = 0.030   # Ellenbogen mind. 3.0 cm über Boden

    constraints.append(p_ee[2] - z_min_ee)
    constraints.append(p_gripper_servo[2] - z_min_grip)
    constraints.append(p_elbow[2] - z_min_elbow)

    # --- B. Box-Ausschluss (Superquadrik 4. Ordnung für das Chassis) ---
    box_test_points = [
        p_elbow,
        p_mid_forearm,
        p_gripper_servo,
        p_ee,
        p_jaw_left,
        p_jaw_right
    ]

    dx_safe = ca.fmax(box_d[0], 0.001)
    dy_safe = ca.fmax(box_d[1], 0.001)
    dz_safe = ca.fmax(box_d[2], 0.001)

    for pt in box_test_points:
        norm_x = (pt[0] - box_c[0]) / dx_safe
        norm_y = (pt[1] - box_c[1]) / dy_safe
        norm_z = (pt[2] - box_c[2]) / dz_safe
        box_dist = norm_x**4 + norm_y**4 + norm_z**4 - 1.0
        constraints.append(box_dist)

    con_h_expr = ca.vertcat(*constraints)

    # =========================================================================
    # 6. ACADOS MODELL ZUSAMMENBAUEN
    # =========================================================================
    model = AcadosModel()
    model.name = model_name
    model.x = x
    model.xdot = xdot
    model.u = u
    model.p = p
    model.f_expl_expr = f_expl
    model.f_impl_expr = xdot - f_expl

    model.con_h_expr = con_h_expr
    model.con_h_expr_e = con_h_expr

    return model


def create_arm_ocp(
    horizon_N: int = 10,
    tf: float = 0.5,
    package_name: str = "picar_arm_safety",
    ros_output_dir: str = None,
    code_export_dir: str = None
) -> AcadosOcp:
    """
    Erstellt das AcadosOcp Problem für den PiCar-Arm mit LINEAR_LS Kosten & Slack-Variablen.
    """
    if ros_output_dir is None:
        ros_output_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    model = export_picar_arm_safety_model()
    ocp = AcadosOcp()
    ocp.model = model

    nx = 6
    nu = 3
    ny = 9
    ny_e = 3
    nh = 3 + 6   # 3x Bodenschutz + 6x Box-Prüfpunkte = 9

    ocp.solver_options.N_horizon = horizon_N
    ocp.solver_options.tf = tf
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.qp_solver_cond_N = min(5, horizon_N)

    # =========================================================================
    # LINEAR_LS KOSTENFUNKTION: y = Vx * x + Vu * u = [q; u - v; u]
    # =========================================================================
    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_0 = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    Vx = np.zeros((ny, nx))
    Vx[0:3, 0:3] = np.eye(3)    # +q
    Vx[3:6, 3:6] = -np.eye(3)   # -v (zusammen mit Vu -> u - v)

    Vu = np.zeros((ny, nu))
    Vu[3:6, 0:3] = np.eye(3)    # +u (für u - v)
    Vu[6:9, 0:3] = np.eye(3)    # +u

    Vx_e = np.zeros((ny_e, nx))
    Vx_e[0:3, 0:3] = np.eye(3)  # +q_N

    ocp.cost.Vx = Vx
    ocp.cost.Vu = Vu
    ocp.cost.Vx_0 = Vx.copy()
    ocp.cost.Vu_0 = Vu.copy()
    ocp.cost.Vx_e = Vx_e

    W = np.diag([
        5.0, 5.0, 5.0,
        0.1, 0.1, 0.1,
        0.2, 0.2, 0.2
    ])
    W_0 = W.copy()
    W_e = np.diag([5.0, 5.0, 5.0])

    ocp.cost.W = W
    ocp.cost.W_0 = W_0
    ocp.cost.W_e = W_e

    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_0 = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_e)

    # =========================================================================
    # CONSTRAINTS & SLACK-VARIABLEN
    # =========================================================================
    ocp.constraints.lbx = np.array([-1.4, -0.8, -1.6, -3.0, -3.0, -3.0])
    ocp.constraints.ubx = np.array([ 1.4,  1.6,  1.6,  3.0,  3.0,  3.0])
    ocp.constraints.idxbx = np.arange(nx)

    ocp.constraints.lbu = np.array([-3.0, -3.0, -3.0])
    ocp.constraints.ubu = np.array([ 3.0,  3.0,  3.0])
    ocp.constraints.idxbu = np.arange(nu)

    ocp.constraints.x0 = np.zeros(nx)

    ocp.constraints.lh = np.zeros(nh)
    ocp.constraints.uh = 1e9 * np.ones(nh)
    ocp.constraints.lh_e = np.zeros(nh)
    ocp.constraints.uh_e = 1e9 * np.ones(nh)

    ocp.constraints.idxsh = np.arange(nh)
    ocp.constraints.idxsh_e = np.arange(nh)
    ocp.constraints.lsh = np.zeros(nh)
    ocp.constraints.ush = np.zeros(nh)
    ocp.constraints.lsh_e = np.zeros(nh)
    ocp.constraints.ush_e = np.zeros(nh)

    ocp.cost.Zl = 1000.0 * np.ones(nh)
    ocp.cost.Zu = np.zeros(nh)
    ocp.cost.zl = 100.0 * np.ones(nh)
    ocp.cost.zu = np.zeros(nh)

    ocp.cost.Zl_e = 1000.0 * np.ones(nh)
    ocp.cost.Zu_e = np.zeros(nh)
    ocp.cost.zl_e = 100.0 * np.ones(nh)
    ocp.cost.zu_e = np.zeros(nh)

    p_init = np.array([
        0.0,    # q4_gripper
        0.0,    # box_xc
        0.0,    # box_yc
        0.06,   # box_zc
        0.12,   # box_dx
        0.05,   # box_dy
        0.07    # box_dz
    ])
    ocp.parameter_values = p_init

    ros_opts = AcadosOcpRosOptions()
    ros_opts.package_name = package_name
    ros_opts.node_name = f"{package_name}_node"
    ros_opts.generated_code_dir = ros_output_dir
    ros_opts.control_topic = "arm_control"
    ros_opts.state_topic = "arm_state"
    ros_opts.reference_topic = "arm_reference"
    ros_opts.parameters_topic = "arm_params"
    ocp.ros_opts = ros_opts

    if code_export_dir is None:
        code_export_dir = os.path.join(ros_output_dir, package_name, "c_generated_code")
    ocp.code_gen_options.code_export_directory = os.path.abspath(code_export_dir)

    return ocp


def main():
    default_ros_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser(description="Export Acados OCP & ROS 2 Subpackages für PiCar-Arm")
    parser.add_argument("--horizon", "-N", type=int, default=10, help="Prädiktionshorizont N (Standard: 10)")
    parser.add_argument("--tf", type=float, default=0.5, help="Zeithorizont in Sekunden (Standard: 0.5s)")
    parser.add_argument("--package_name", type=str, default="picar_arm_safety", help="Generierter ROS 2 Package-Name")
    parser.add_argument("--ros_dir", type=str, default=default_ros_dir, help="Zielverzeichnis für Subpackages")
    parser.add_argument("--build", action="store_true", help="Kompiliere libacados_ocp_solver direkt nach der Generierung")

    args = parser.parse_args()

    print(f"=================================================================")
    print(f"PiCar-Arm Acados OCP Safety Filter Generator (LINEAR_LS)")
    print(f"  - Horizont (N):      {args.horizon}")
    print(f"  - Zeithorizont (Tf): {args.tf:.2f} s (dt = {args.tf / args.horizon:.3f} s)")
    print(f"  - ROS 2 Subpackage:  {args.package_name}")
    print(f"  - Zielverzeichnis:   {args.ros_dir}")
    print(f"=================================================================")

    code_export_dir = os.path.join(args.ros_dir, args.package_name, "c_generated_code")
    ocp = create_arm_ocp(
        horizon_N=args.horizon,
        tf=args.tf,
        package_name=args.package_name,
        ros_output_dir=args.ros_dir,
        code_export_dir=code_export_dir
    )

    print("[1/2] Generiere Acados C-Solver und ROS 2 Subpackages...")
    AcadosOcpSolver.generate(ocp)
    print("  -> Generierung erfolgreich abgeschlossen.")

    if args.build:
        print("[2/2] Kompiliere libacados_ocp_solver Shared Library...")
        AcadosOcpSolver.build(code_export_dir)
        print("  -> Kompilierung erfolgreich abgeschlossen.")
    else:
        print("[2/2] Kompilierung übersprungen (via --build ausführbar).")


if __name__ == "__main__":
    main()
