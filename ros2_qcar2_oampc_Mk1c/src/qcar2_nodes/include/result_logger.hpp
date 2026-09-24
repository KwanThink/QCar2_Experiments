#ifndef RESULT_LOGGER_HPP_
#define RESULT_LOGGER_HPP_

#include <string>
#include <vector>

#include "qcar2_traj.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qcar2_oampc
{

struct VehicleState;
struct SolverResult;

struct RunLogSample
{
  double t = 0.0;
  double x = 0.0;
  double y = 0.0;
  double psi = 0.0;
  double vx_measured = 0.0;
  double vx_command = 0.0;
  double delta_command = 0.0;
  double ax_command = 0.0;
  double v1_command = 0.0;
  double v2_command = 0.0;
  double x_ref = 0.0;
  double y_ref = 0.0;
  double psi_ref = 0.0;
  double vx_ref = 0.0;
  double delta_ref = 0.0;
  double ax_ref = 0.0;
  double solve_time = 0.0;
  double slack_value = 0.0;
  bool solver_success = false;
  std::string solver_status;
  int number_of_config_obstacles = 0;
  int active_obstacles = 0;
  int active_edges = 0;
  int binary_variables = 0;
  int total_binary_variables = 0;
};

class ResultLogger
{
public:
  // Store result export settings used for each completed tracking run.
  ResultLogger(
    bool enabled,
    std::string result_directory,
    std::string run_directory_prefix,
    std::string source_config_file,
    std::string obstacle_map_file);

  // Clear all samples and counters before a new tracking run.
  void reset();

  // Store the complete active reference so the XY plot is independent of run duration.
  void setFullReferenceTrajectory(const qcar2_flmpc::ReferenceTrajectory & trajectory);

  // Count one solver failure for the current run metrics.
  void incrementSolverFailures();

  // Append one measured, reference, control, and solver-complexity sample.
  void recordSample(
    double elapsed_time,
    const VehicleState & current_state,
    const qcar2_flmpc::ReferenceSample & reference_sample,
    const SolverResult & solver_result,
    double longitudinal_velocity_command,
    double solve_time);

  // Export CSV files, metrics, snapshots, and SVG plots for the current run.
  void exportRunResults(const std::string & stop_reason, const rclcpp::Logger & logger);

private:
  // Create the next available OAMPC Mk1c run output directory.
  std::string createNextResultDirectory() const;

  // Copy the exact source parameter YAML into the completed run directory.
  void copySourceConfig(const std::string & run_directory, const rclcpp::Logger & logger) const;

  // Copy the exact processed obstacle geometry into the completed run directory.
  void copyObstacleMap(const std::string & run_directory, const rclcpp::Logger & logger) const;

  // Wrap an angle into the interval from minus pi to pi.
  static double normalizeAngle(double angle);

  // Quote one CSV string field safely.
  static std::string quoteCsvString(const std::string & value);

  bool enabled_ = true;
  std::string result_directory_;
  std::string run_directory_prefix_;
  std::string source_config_file_;
  std::string obstacle_map_file_;
  std::vector<RunLogSample> run_log_;
  std::vector<qcar2_flmpc::ReferenceSample> full_reference_;
  int solver_failures_ = 0;
  bool run_exported_ = false;
};

}  // namespace qcar2_oampc

#endif  // RESULT_LOGGER_HPP_
