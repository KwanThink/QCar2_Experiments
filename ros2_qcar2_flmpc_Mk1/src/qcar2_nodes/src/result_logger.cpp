#include "result_logger.hpp"
#include "qcar2_flmpc_controller.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace qcar2_flmpc
{
namespace
{
constexpr double kPi = 3.1415926535897932384626433832795;

// Quote one shell argument so result paths with spaces remain safe.
std::string shellQuote(const std::string & value)
{
  std::string quoted = "'";
  for (const char character : value) {
    if (character == '\'') {
      quoted += "'\\''";
    } else {
      quoted += character;
    }
  }
  quoted += "'";
  return quoted;
}
}  // namespace

// Store output settings used for every completed tracking run.
ResultLogger::ResultLogger(
  bool enabled,
  std::string result_directory,
  std::string run_directory_prefix,
  std::string source_config_file)
: enabled_(enabled),
  result_directory_(std::move(result_directory)),
  run_directory_prefix_(std::move(run_directory_prefix)),
  source_config_file_(std::move(source_config_file))
{
}

// Clear all recorded samples and counters before a new run.
void ResultLogger::reset()
{
  run_log_.clear();
  solver_failures_ = 0;
  run_exported_ = false;
}

// Increment the solver failure counter for the active run.
void ResultLogger::incrementSolverFailures()
{
  ++solver_failures_;
}

// Append one measured state, reference, command, and solve-time sample.
void ResultLogger::recordSample(
  double elapsed_time,
  const VehicleState & current_state,
  const ReferenceSample & reference_sample,
  const SolverResult & solver_result,
  double longitudinal_velocity_command,
  double solve_time)
{
  if (!enabled_) {
    return;
  }

  RunLogSample sample;
  sample.t = elapsed_time;
  sample.x = current_state.x;
  sample.y = current_state.y;
  // Save yaw on the same 2*pi branch as psi_ref so plots do not show a false jump between +pi and -pi.
  sample.psi = reference_sample.psi + ResultLogger::normalizeAngle(
    current_state.psi - reference_sample.psi
  );
  sample.vx_measured = current_state.vx;
  sample.vx_command = solver_result.success ? longitudinal_velocity_command : 0.0;
  sample.delta_command = solver_result.success ? solver_result.delta : 0.0;
  sample.ax_command = solver_result.success ? solver_result.ax : 0.0;
  sample.v1_command = solver_result.success ? solver_result.v1 : 0.0;
  sample.v2_command = solver_result.success ? solver_result.v2 : 0.0;
  sample.x_ref = reference_sample.x;
  sample.y_ref = reference_sample.y;
  sample.psi_ref = reference_sample.psi;
  sample.vx_ref = reference_sample.vx;
  sample.delta_ref = reference_sample.delta;
  sample.ax_ref = reference_sample.ax;
  sample.solve_time = solve_time;
  sample.solver_status = solver_result.status;
  run_log_.push_back(sample);
}

// Create and return the next available run_Mk1_x directory.
std::string ResultLogger::createNextResultDirectory() const
{
  const std::filesystem::path root(result_directory_);
  std::filesystem::create_directories(root);

  for (int index = 1; index < 100000; ++index) {
    std::ostringstream directory_name;
    directory_name << run_directory_prefix_ << index;
    const std::filesystem::path candidate = root / directory_name.str();
    std::error_code error_code;
    if (std::filesystem::create_directory(candidate, error_code)) {
      return candidate.string();
    }
    if (error_code) {
      throw std::runtime_error("cannot create result directory: " + candidate.string());
    }
  }

  throw std::runtime_error("no free " + run_directory_prefix_ + "x result directory index was found");
}

// Copy the parameter YAML used by the launch file into the run directory.
void ResultLogger::copySourceConfig(
  const std::string & run_directory,
  const rclcpp::Logger & logger) const
{
  std::filesystem::path source_path(source_config_file_);

  if (source_path.empty()) {
    try {
      const std::string package_share_directory =
        ament_index_cpp::get_package_share_directory("qcar2_nodes");
      source_path =
        std::filesystem::path(package_share_directory) / "config" / "qcar2_flmpc.yaml";
    } catch (const std::exception & exception) {
      RCLCPP_WARN(
        logger,
        "Run data were saved, but the FLMPC source configuration could not be located: %s",
        exception.what());
      return;
    }
  }

  std::error_code error_code;
  if (!std::filesystem::is_regular_file(source_path, error_code)) {
    RCLCPP_WARN(
      logger,
      "Run data were saved, but the FLMPC source configuration does not exist: %s",
      source_path.string().c_str());
    return;
  }

  const std::filesystem::path destination_path =
    std::filesystem::path(run_directory) / "run_config.yaml";
  std::filesystem::copy_file(
    source_path,
    destination_path,
    std::filesystem::copy_options::overwrite_existing,
    error_code);

  if (error_code) {
    RCLCPP_WARN(
      logger,
      "Run data were saved, but the FLMPC configuration could not be copied to %s: %s",
      destination_path.string().c_str(),
      error_code.message().c_str());
  }
}

// Export CSV files, summary metrics, and SVG plots for the active run.
void ResultLogger::exportRunResults(const std::string & stop_reason, const rclcpp::Logger & logger)
{
  if (!enabled_ || run_exported_ || run_log_.empty()) {
    return;
  }

  run_exported_ = true;

  std::string result_directory;
  try {
    result_directory = createNextResultDirectory();
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(logger, "Failed to create FLMPC result directory: %s", exception.what());
    return;
  }

  const std::filesystem::path result_path(result_directory);

  // Open one result file with enough precision for later numerical analysis.
  auto open_file = [&](const std::string & file_name) {
    std::ofstream file(result_path / file_name);
    file << std::setprecision(18);
    return file;
  };

  {
    auto file = open_file("states.csv");
    file << "t,X,Y,psi,vx\n";
    for (const auto & sample : run_log_) {
      file << sample.t << ',' << sample.x << ',' << sample.y << ',' << sample.psi << ',' << sample.vx_measured << '\n';
    }
  }

  {
    auto file = open_file("controls.csv");
    file << "t,desired_speed,delta,ax\n";
    for (const auto & sample : run_log_) {
      file << sample.t << ',' << sample.vx_command << ',' << sample.delta_command << ',' << sample.ax_command << '\n';
    }
  }

  {
    auto file = open_file("reference_used.csv");
    file << "t,X_ref,Y_ref,psi_ref,vx_ref\n";
    for (const auto & sample : run_log_) {
      file << sample.t << ',' << sample.x_ref << ',' << sample.y_ref << ',' << sample.psi_ref << ',' << sample.vx_ref << '\n';
    }
  }

  {
    auto file = open_file("reference_controls.csv");
    file << "t,delta_ref,ax_ref\n";
    for (const auto & sample : run_log_) {
      file << sample.t << ',' << sample.delta_ref << ',' << sample.ax_ref << '\n';
    }
  }

  {
    auto file = open_file("virtual_inputs.csv");
    file << "t,v1,v2\n";
    for (const auto & sample : run_log_) {
      file << sample.t << ',' << sample.v1_command << ',' << sample.v2_command << '\n';
    }
  }

  {
    auto file = open_file("solve_times.csv");
    file << "t,solve_time,status\n";
    for (const auto & sample : run_log_) {
      file << sample.t << ',' << sample.solve_time << ',' << sample.solver_status << '\n';
    }
  }

  double sum_squared_x_error = 0.0;
  double sum_squared_y_error = 0.0;
  double sum_squared_position_error = 0.0;
  double sum_squared_psi_error = 0.0;
  double sum_squared_vx_error = 0.0;
  double max_position_error = 0.0;
  double max_abs_desired_speed = 0.0;
  double max_abs_delta = 0.0;
  double max_abs_ax = 0.0;

  for (const auto & sample : run_log_) {
    const double x_error = sample.x - sample.x_ref;
    const double y_error = sample.y - sample.y_ref;
    const double psi_error = normalizeAngle(sample.psi - sample.psi_ref);
    const double vx_error = sample.vx_measured - sample.vx_ref;
    const double position_error = std::hypot(x_error, y_error);

    sum_squared_x_error += x_error * x_error;
    sum_squared_y_error += y_error * y_error;
    sum_squared_position_error += position_error * position_error;
    sum_squared_psi_error += psi_error * psi_error;
    sum_squared_vx_error += vx_error * vx_error;
    max_position_error = std::max(max_position_error, position_error);
    max_abs_desired_speed = std::max(max_abs_desired_speed, std::abs(sample.vx_command));
    max_abs_delta = std::max(max_abs_delta, std::abs(sample.delta_command));
    max_abs_ax = std::max(max_abs_ax, std::abs(sample.ax_command));
  }

  const double count = static_cast<double>(run_log_.size());
  {
    auto file = open_file("metrics.json");
    file << "{\n";
    file << "  \"stop_reason\": \"" << stop_reason << "\",\n";
    file << "  \"num_steps\": " << run_log_.size() << ",\n";
    file << "  \"num_solver_failures\": " << solver_failures_ << ",\n";
    file << "  \"rmse_X\": " << std::sqrt(sum_squared_x_error / count) << ",\n";
    file << "  \"rmse_Y\": " << std::sqrt(sum_squared_y_error / count) << ",\n";
    file << "  \"rmse_position\": " << std::sqrt(sum_squared_position_error / count) << ",\n";
    file << "  \"rmse_psi\": " << std::sqrt(sum_squared_psi_error / count) << ",\n";
    file << "  \"rmse_vx\": " << std::sqrt(sum_squared_vx_error / count) << ",\n";
    file << "  \"max_position_error\": " << max_position_error << ",\n";
    file << "  \"max_abs_desired_speed\": " << max_abs_desired_speed << ",\n";
    file << "  \"max_abs_delta\": " << max_abs_delta << ",\n";
    file << "  \"max_abs_ax\": " << max_abs_ax << "\n";
    file << "}\n";
  }

  copySourceConfig(result_directory, logger);

  std::string plot_script_path;
  try {
    const std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("qcar2_nodes");
    plot_script_path =
      (std::filesystem::path(package_share_directory) / "scripts" / "plot_flmpc_results.py").string();
  } catch (const std::exception & exception) {
    RCLCPP_WARN(
      logger,
      "CSV and metrics were saved to %s, but the plotting script could not be located: %s",
      result_directory.c_str(),
      exception.what());
    return;
  }

  const std::string command =
    "cd " + shellQuote(result_directory) +
    " && python3 " + shellQuote(plot_script_path) +
    " " + shellQuote(result_directory) +
    " > plot_stdout.log 2> plot_stderr.log";
  const int plot_status = std::system(command.c_str());
  if (plot_status != 0) {
    RCLCPP_WARN(
      logger,
      "CSV and metrics were saved to %s, but SVG plotting failed. Check plot_stderr.log in that folder.",
      result_directory.c_str());
  } else {
    std::error_code error_code;
    const auto stdout_path = result_path / "plot_stdout.log";
    const auto stderr_path = result_path / "plot_stderr.log";
    if (std::filesystem::exists(stdout_path, error_code) &&
      std::filesystem::file_size(stdout_path, error_code) == 0U)
    {
      std::filesystem::remove(stdout_path, error_code);
    }
    if (std::filesystem::exists(stderr_path, error_code) &&
      std::filesystem::file_size(stderr_path, error_code) == 0U)
    {
      std::filesystem::remove(stderr_path, error_code);
    }
    RCLCPP_INFO(logger, "Saved FLMPC run results to %s", result_directory.c_str());
  }
}

// Wrap an angle into the interval [-pi, pi].
double ResultLogger::normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

}  // namespace qcar2_flmpc
