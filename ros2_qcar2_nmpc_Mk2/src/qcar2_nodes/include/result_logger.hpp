#ifndef RESULT_LOGGER_HPP_
#define RESULT_LOGGER_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace qcar2_nmpc
{

struct VehicleState;
struct ReferenceSample;
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
  double x_ref = 0.0;
  double y_ref = 0.0;
  double psi_ref = 0.0;
  double vx_ref = 0.0;
  double delta_ref = 0.0;
  double ax_ref = 0.0;
  double solve_time = 0.0;
  int solver_status = 0;
};

class ResultLogger
{
public:
  // Store result export settings used for each completed tracking run.
  ResultLogger(
    bool enabled,
    std::string result_directory,
    std::string run_directory_prefix,
    std::string source_config_file);

  // Clear all samples and counters before a new tracking run.
  void reset();

  // Count one solver failure for the current run metrics.
  void incrementSolverFailures();

  // Append one measured, reference, control, and solve-time sample.
  void recordSample(
    double elapsed_time,
    const VehicleState & current_state,
    const ReferenceSample & reference_sample,
    const SolverResult & solver_result,
    double longitudinal_velocity_command,
    double solve_time);

  // Export CSV files, metrics, and SVG plots for the current run.
  void exportRunResults(const std::string & stop_reason, const rclcpp::Logger & logger);

private:
  // Create the next available run_Mk2_<shape_type>_x output directory.
  std::string createNextResultDirectory() const;

  // Copy the source parameter YAML into the completed run directory.
  void copySourceConfig(const std::string & run_directory, const rclcpp::Logger & logger) const;

  // Wrap an angle into the interval [-pi, pi].
  static double normalizeAngle(double angle);

  bool enabled_ = true;
  std::string result_directory_;
  std::string run_directory_prefix_;
  std::string source_config_file_;
  std::vector<RunLogSample> run_log_;
  int solver_failures_ = 0;
  bool run_exported_ = false;
};

}  // namespace qcar2_nmpc

#endif  // RESULT_LOGGER_HPP_
