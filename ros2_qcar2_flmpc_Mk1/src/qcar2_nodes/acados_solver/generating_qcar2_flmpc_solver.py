#!/usr/bin/env python3
"""Generate the discrete acados FLMPC solver used by qcar2_flmpc_controller."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from casadi import SX, vertcat

PACKAGE_DIR = Path(__file__).resolve().parents[1]
CONFIG_FILE = PACKAGE_DIR / "config" / "qcar2_flmpc.yaml"
SOLVER_DIR = PACKAGE_DIR / "acados_solver" / "qcar2_flmpc_solver"


# Build a diagonal matrix from one YAML weight vector.
def diagonal_matrix(values):
    return np.diag(np.asarray(values, dtype=float))


# Export the exact discrete double-integrator flat model.
def export_qcar2_flmpc_model(sample_time: float) -> AcadosModel:
    model = AcadosModel()
    model.name = "qcar2_flmpc"

    X = SX.sym("X")
    X_dot = SX.sym("X_dot")
    Y = SX.sym("Y")
    Y_dot = SX.sym("Y_dot")
    flat_state = vertcat(X, X_dot, Y, Y_dot)

    v1 = SX.sym("v1")
    v2 = SX.sym("v2")
    virtual_input = vertcat(v1, v2)

    half_sample_time_squared = 0.5 * sample_time * sample_time
    next_flat_state = vertcat(
        X + sample_time * X_dot + half_sample_time_squared * v1,
        X_dot + sample_time * v1,
        Y + sample_time * Y_dot + half_sample_time_squared * v2,
        Y_dot + sample_time * v2,
    )

    model.x = flat_state
    model.u = virtual_input
    model.disc_dyn_expr = next_flat_state
    return model


# Build the FLMPC OCP with exact next-position cost and mapped input constraints.
def build_ocp(parameters: dict) -> AcadosOcp:
    horizon_steps = int(parameters["mpc_N"])
    sample_time = float(parameters["Ts"])

    model = export_qcar2_flmpc_model(sample_time)
    ocp = AcadosOcp()
    ocp.model = model

    nx = int(model.x.size()[0])
    nu = int(model.u.size()[0])
    ny = 4

    ocp.dims.N = horizon_steps
    ocp.solver_options.tf = horizon_steps * sample_time

    position_weights = diagonal_matrix(parameters["Q"])
    virtual_input_weights = diagonal_matrix(parameters["R"])
    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.W = np.block(
        [
            [position_weights, np.zeros((2, 2))],
            [np.zeros((2, 2)), virtual_input_weights],
        ]
    )

    half_sample_time_squared = 0.5 * sample_time * sample_time
    ocp.cost.Vx = np.array(
        [
            [1.0, sample_time, 0.0, 0.0],
            [0.0, 0.0, 1.0, sample_time],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
        ],
        dtype=float,
    )
    ocp.cost.Vu = np.array(
        [
            [half_sample_time_squared, 0.0],
            [0.0, half_sample_time_squared],
            [1.0, 0.0],
            [0.0, 1.0],
        ],
        dtype=float,
    )
    ocp.cost.yref = np.zeros(ny)


    initial_constraint_matrix = np.eye(nu)
    ocp.constraints.C = np.zeros((2, nx))
    ocp.constraints.D = initial_constraint_matrix
    ocp.constraints.lg = np.array(
        [np.tan(float(parameters["delta_min"])), float(parameters["ax_min"])],
        dtype=float,
    )
    ocp.constraints.ug = np.array(
        [np.tan(float(parameters["delta_max"])), float(parameters["ax_max"])],
        dtype=float,
    )
    ocp.constraints.x0 = np.zeros(nx)

    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "DISCRETE"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.print_level = 0

    # Use unit stage scaling because the requested discrete objective is an unscaled sum.
    ocp.solver_options.cost_scaling = np.ones(horizon_steps + 1)

    SOLVER_DIR.mkdir(parents=True, exist_ok=True)
    ocp.code_export_directory = str(SOLVER_DIR)
    return ocp


# Load the FLMPC YAML and generate the acados C solver.
def main() -> None:
    with CONFIG_FILE.open("r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    parameters = data["qcar2_flmpc_controller"]["ros__parameters"]
    ocp = build_ocp(parameters)
    json_file = SOLVER_DIR / "acados_ocp_qcar2_flmpc.json"
    AcadosOcpSolver(ocp, json_file=str(json_file), generate=True, build=True)
    print(f"Generated qcar2_flmpc acados solver in: {SOLVER_DIR}")


if __name__ == "__main__":
    main()
