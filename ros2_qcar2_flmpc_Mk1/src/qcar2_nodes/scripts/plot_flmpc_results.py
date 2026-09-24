#!/usr/bin/env python3

import argparse
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# Parse the result directory passed by the C++ result logger.
def parse_arguments():
    parser = argparse.ArgumentParser(description="Generate FLMPC result plots from exported CSV files.")
    parser.add_argument("result_directory", type=Path, help="Directory containing one completed FLMPC run.")
    return parser.parse_args()


# Load one CSV file and normalize scalar rows into one-dimensional arrays.
def load_csv(result_directory, file_name):
    path = result_directory / file_name
    if not path.exists() or path.stat().st_size == 0:
        return None

    data = np.genfromtxt(path, delimiter=",", names=True)
    if data.size == 0:
        return None
    return np.atleast_1d(data)


# Wrap vectorized angle errors into the interval [-pi, pi].
def wrap_to_pi(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


# Plot measured states together with their reference signals.
def plot_states(result_directory, states, reference_states):
    if states is None or reference_states is None:
        return

    figure, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    axes[0].plot(states["t"], states["X"], label="X")
    axes[0].plot(reference_states["t"], reference_states["X_ref"], "--", label="X_ref")
    axes[0].set_ylabel("X [m]")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(states["t"], states["Y"], label="Y")
    axes[1].plot(reference_states["t"], reference_states["Y_ref"], "--", label="Y_ref")
    axes[1].set_ylabel("Y [m]")
    axes[1].legend()
    axes[1].grid(True)

    axes[2].plot(states["t"], states["psi"], label="psi")
    axes[2].plot(reference_states["t"], reference_states["psi_ref"], "--", label="psi_ref")
    axes[2].set_ylabel("psi [rad]")
    axes[2].legend()
    axes[2].grid(True)

    axes[3].plot(states["t"], states["vx"], label="vx measured")
    axes[3].plot(reference_states["t"], reference_states["vx_ref"], "--", label="vx_ref")
    axes[3].set_ylabel("vx [m/s]")
    axes[3].set_xlabel("t [s]")
    axes[3].legend()
    axes[3].grid(True)

    figure.tight_layout()
    figure.savefig(result_directory / "states.svg")
    plt.close(figure)


# Plot state tracking errors computed from measured and reference signals.
def plot_state_errors(result_directory, states, reference_states):
    if states is None or reference_states is None:
        return

    x_error = states["X"] - reference_states["X_ref"]
    y_error = states["Y"] - reference_states["Y_ref"]
    psi_error = wrap_to_pi(states["psi"] - reference_states["psi_ref"])
    vx_error = states["vx"] - reference_states["vx_ref"]

    figure, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    axes[0].plot(states["t"], x_error, label="e_X")
    axes[0].set_ylabel("e_X [m]")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(states["t"], y_error, label="e_Y")
    axes[1].set_ylabel("e_Y [m]")
    axes[1].legend()
    axes[1].grid(True)

    axes[2].plot(states["t"], psi_error, label="e_psi")
    axes[2].set_ylabel("e_psi [rad]")
    axes[2].legend()
    axes[2].grid(True)

    axes[3].plot(states["t"], vx_error, label="e_vx")
    axes[3].set_ylabel("e_vx [m/s]")
    axes[3].set_xlabel("t [s]")
    axes[3].legend()
    axes[3].grid(True)

    figure.tight_layout()
    figure.savefig(result_directory / "states_error.svg")
    plt.close(figure)


# Plot commanded controls together with their reference values.
def plot_controls(result_directory, controls, reference_states, reference_controls):
    if controls is None or reference_states is None or reference_controls is None:
        return

    figure, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(controls["t"], controls["desired_speed"], label="desired_speed")
    axes[0].plot(reference_states["t"], reference_states["vx_ref"], "--", label="vx_ref")
    axes[0].set_ylabel("speed [m/s]")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(controls["t"], controls["delta"], label="delta")
    axes[1].plot(reference_controls["t"], reference_controls["delta_ref"], "--", label="delta_ref")
    axes[1].set_ylabel("delta [rad]")
    axes[1].legend()
    axes[1].grid(True)

    axes[2].plot(controls["t"], controls["ax"], label="ax")
    axes[2].plot(reference_controls["t"], reference_controls["ax_ref"], "--", label="ax_ref")
    axes[2].set_ylabel("ax [m/s²]")
    axes[2].set_xlabel("t [s]")
    axes[2].legend()
    axes[2].grid(True)

    figure.tight_layout()
    figure.savefig(result_directory / "controls.svg")
    plt.close(figure)


# Plot control tracking errors relative to the reference controls.
def plot_control_errors(result_directory, controls, reference_states, reference_controls):
    if controls is None or reference_states is None or reference_controls is None:
        return

    speed_error = controls["desired_speed"] - reference_states["vx_ref"]
    delta_error = controls["delta"] - reference_controls["delta_ref"]
    ax_error = controls["ax"] - reference_controls["ax_ref"]

    figure, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(controls["t"], speed_error, label="e_speed")
    axes[0].set_ylabel("e_speed [m/s]")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(controls["t"], delta_error, label="e_delta")
    axes[1].set_ylabel("e_delta [rad]")
    axes[1].legend()
    axes[1].grid(True)

    axes[2].plot(controls["t"], ax_error, label="e_ax")
    axes[2].set_ylabel("e_ax [m/s²]")
    axes[2].set_xlabel("t [s]")
    axes[2].legend()
    axes[2].grid(True)

    figure.tight_layout()
    figure.savefig(result_directory / "control_error.svg")
    plt.close(figure)


# Plot solver execution time for every recorded control step.
def plot_solve_times(result_directory, solve_times):
    if solve_times is None:
        return

    figure, axis = plt.subplots(figsize=(10, 4))
    axis.plot(solve_times["t"], solve_times["solve_time"], label="solve_time")
    axis.set_xlabel("t [s]")
    axis.set_ylabel("solve time [s]")
    axis.grid(True)
    axis.legend()

    figure.tight_layout()
    figure.savefig(result_directory / "solve_times.svg")
    plt.close(figure)


# Plot the applied FLMPC virtual inputs in two stacked subplots.
def plot_virtual_inputs(result_directory, virtual_inputs):
    if virtual_inputs is None:
        return

    figure, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    axes[0].plot(virtual_inputs["t"], virtual_inputs["v1"], label="v1")
    axes[0].set_ylabel("v1")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(virtual_inputs["t"], virtual_inputs["v2"], label="v2")
    axes[1].set_ylabel("v2")
    axes[1].set_xlabel("t [s]")
    axes[1].legend()
    axes[1].grid(True)

    figure.tight_layout()
    figure.savefig(result_directory / "virtual_inputs.svg")
    plt.close(figure)


# Plot the measured planar path against the reference trajectory.
def plot_trajectory(result_directory, states, reference_states):
    if states is None or reference_states is None:
        return

    figure, axis = plt.subplots(figsize=(7, 7))
    axis.plot(reference_states["X_ref"], reference_states["Y_ref"], "--", label="reference")
    axis.plot(states["X"], states["Y"], label="actual")
    axis.set_xlabel("X [m]")
    axis.set_ylabel("Y [m]")
    axis.axis("equal")
    axis.grid(True)
    axis.legend()

    figure.tight_layout()
    figure.savefig(result_directory / "trajectory_tracking.svg")
    plt.close(figure)


# Load one completed run and generate all available SVG result plots.
def main():
    arguments = parse_arguments()
    result_directory = arguments.result_directory.expanduser().resolve()
    if not result_directory.is_dir():
        raise NotADirectoryError(f"Result directory does not exist: {result_directory}")

    states = load_csv(result_directory, "states.csv")
    controls = load_csv(result_directory, "controls.csv")
    reference_states = load_csv(result_directory, "reference_used.csv")
    reference_controls = load_csv(result_directory, "reference_controls.csv")
    solve_times = load_csv(result_directory, "solve_times.csv")
    virtual_inputs = load_csv(result_directory, "virtual_inputs.csv")

    plot_states(result_directory, states, reference_states)
    plot_state_errors(result_directory, states, reference_states)
    plot_controls(result_directory, controls, reference_states, reference_controls)
    plot_control_errors(result_directory, controls, reference_states, reference_controls)
    plot_solve_times(result_directory, solve_times)
    plot_virtual_inputs(result_directory, virtual_inputs)
    plot_trajectory(result_directory, states, reference_states)


if __name__ == "__main__":
    main()
