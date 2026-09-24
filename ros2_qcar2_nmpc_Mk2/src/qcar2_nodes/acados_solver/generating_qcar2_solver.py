#!/usr/bin/env python3
"""Generate the acados C solver used by qcar2_nmpc_controller.

Run this script on the QCar2 after acados and acados_template are available.
The generated files are written to:

    qcar2_nodes/acados_solver/qcar2_solver

The C++ node detects the generated header/library during the next colcon build.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from casadi import SX, cos, sin, tan, vertcat

PACKAGE_DIR = Path(__file__).resolve().parents[1]
CONFIG_FILE = PACKAGE_DIR / "config" / "qcar2_nmpc.yaml"
SOLVER_DIR = PACKAGE_DIR / "acados_solver" / "qcar2_solver"


def _diag(values):
    return np.diag(np.asarray(values, dtype=float))


# Export the same single-track model as the simulation project: x=[X,Y,psi,vx], u=[delta,ax].
def export_qcar2_single_track_model(wheelbase: float) -> AcadosModel:
    model = AcadosModel()
    model.name = "qcar2_single_track"

    X = SX.sym("X")
    Y = SX.sym("Y")
    psi = SX.sym("psi")
    vx = SX.sym("vx")
    states = vertcat(X, Y, psi, vx)

    delta = SX.sym("delta")
    ax = SX.sym("ax")
    controls = vertcat(delta, ax)

    X_dot = SX.sym("X_dot")
    Y_dot = SX.sym("Y_dot")
    psi_dot = SX.sym("psi_dot")
    vx_dot = SX.sym("vx_dot")
    xdot = vertcat(X_dot, Y_dot, psi_dot, vx_dot)

    f_expl = vertcat(
        vx * cos(psi),
        vx * sin(psi),
        (vx / wheelbase) * tan(delta),
        ax,
    )

    model.x = states
    model.u = controls
    model.xdot = xdot
    model.f_expl_expr = f_expl
    model.f_impl_expr = xdot - f_expl
    return model


# Build the OCP definition with the same model, constraints, cost, and ERK/SQP_RTI settings as simulation.
def build_ocp(parameters: dict) -> AcadosOcp:
    wheelbase = float(parameters["wheelbase"])
    horizon_steps = int(parameters["mpc_N"])
    sample_time = float(parameters["Ts"])

    model = export_qcar2_single_track_model(wheelbase)
    ocp = AcadosOcp()
    ocp.model = model

    nx = int(model.x.size()[0])
    nu = int(model.u.size()[0])
    ny = nx + nu

    ocp.dims.N = horizon_steps
    ocp.solver_options.tf = horizon_steps * sample_time

    Q = _diag(parameters["Q"])
    R = _diag(parameters["R"])
    Qe = _diag(parameters["Qe"])

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    ocp.cost.W = np.block([[Q, np.zeros((nx, nu))], [np.zeros((nu, nx)), R]])
    ocp.cost.W_e = Qe
    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[nx:, :] = np.eye(nu)
    ocp.cost.Vx_e = np.eye(nx)
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(nx)

    ocp.constraints.lbu = np.array([parameters["delta_min"], parameters["ax_min"]], dtype=float)
    ocp.constraints.ubu = np.array([parameters["delta_max"], parameters["ax_max"]], dtype=float)
    ocp.constraints.idxbu = np.array([0, 1], dtype=int)

    ocp.constraints.idxbx = np.array([3], dtype=int)
    ocp.constraints.lbx = np.array([parameters["vx_min"]], dtype=float)
    ocp.constraints.ubx = np.array([parameters["vx_max"]], dtype=float)
    ocp.constraints.x0 = np.zeros(nx)

    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.print_level = 0

    SOLVER_DIR.mkdir(parents=True, exist_ok=True)
    ocp.code_export_directory = str(SOLVER_DIR)
    return ocp


# Load qcar2_nmpc.yaml and generate/build the acados solver in qcar2_solver/.
def main() -> None:
    with CONFIG_FILE.open("r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    parameters = data["qcar2_nmpc_controller"]["ros__parameters"]
    ocp = build_ocp(parameters)
    json_file = SOLVER_DIR / "acados_ocp_qcar2_single_track.json"
    AcadosOcpSolver(ocp, json_file=str(json_file), generate=True, build=True)
    print(f"Generated qcar2_single_track acados solver in: {SOLVER_DIR}")


if __name__ == "__main__":
    main()
