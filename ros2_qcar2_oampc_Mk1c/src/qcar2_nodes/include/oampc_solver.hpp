#ifndef OAMPC_SOLVER_HPP_
#define OAMPC_SOLVER_HPP_

#include "obstacle_map.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "gurobi_c++.h"
#include "rclcpp/rclcpp.hpp"

namespace qcar2_oampc
{

struct OAMPCParameters
{
  double wheelbase = 0.25725;
  double epsilon = 0.001;
  double sample_time = 0.03;
  int horizon_steps = 30;
  double vx_min = -2.0;
  double vx_max = 2.0;
  double delta_min = -0.5236;
  double delta_max = 0.5236;
  double ax_min = -1.0;
  double ax_max = 1.0;
  std::array<double, 2> position_weights{{120.0, 120.0}};
  std::array<double, 2> virtual_input_weights{{15.0, 15.0}};
  double big_m = 100.0;
  double gamma = 0.35;
  double slack_weight = 100000000.0;
  bool gurobi_output_flag = false;
  double gurobi_time_limit = 0.025;
  int maximum_active_obstacles = 4;
};

struct SolverResult
{
  int status = -1;
  std::string status_name = "NOT_SOLVED";
  double delta = 0.0;
  double ax = 0.0;
  double v1 = 0.0;
  double v2 = 0.0;
  double slack_value = 0.0;
  bool success = false;
  int number_of_config_obstacles = 0;
  int active_obstacles = 0;
  int active_edges = 0;
  int binary_variables = 0;
  int total_binary_variables = 0;
};

class OAMPCSolver
{
public:
  // Store OAMPC parameters and loaded geometry for persistent model construction.
  OAMPCSolver(const OAMPCParameters & parameters, const ObstacleMap & obstacle_map);

  // Release the persistent Gurobi model and environment.
  ~OAMPCSolver();

  // Build the persistent Gurobi MIQP model once at controller startup.
  bool initialize(const rclcpp::Logger & logger);

  // Solve one OAMPC step with stage-dependent input sets and active obstacles.
  SolverResult solve(
    const std::array<double, 4> & current_physical_state,
    const std::vector<std::array<double, 4>> & state_reference_horizon,
    const std::vector<std::array<double, 2>> & input_reference_horizon,
    const std::vector<const Obstacle *> & active_obstacles);

  // Clear every cached solution and explicit Gurobi start value.
  void resetWarmStart();

  // Return whether the persistent Gurobi model is available.
  bool isAvailable() const { return model_available_; }

private:
  struct StageInputConstraints
  {
    GRBConstr steering_lower;
    GRBConstr steering_upper;
    GRBConstr acceleration_lower;
    GRBConstr acceleration_upper;
  };

  // Build all variables and fixed constraints of the persistent MIQP model.
  void buildPersistentModel();

  // Update one persistent obstacle slot for an active polygon or inactivity.
  void updateObstacleSlot(int obstacle_slot, const Obstacle * obstacle);

  // Keep persistent slot identities while assigning the current active obstacles.
  void assignActiveObstaclesToSlots(const std::vector<const Obstacle *> & active_obstacles);

  // Convert a physical state into flat coordinates.
  static std::array<double, 4> physicalStateToFlatState(
    const std::array<double, 4> & physical_state);

  // Convert one physical reference input into a virtual-input initial guess.
  std::array<double, 2> physicalInputToVirtualInput(
    const std::array<double, 4> & physical_state,
    const std::array<double, 2> & physical_input) const;

  // Reconstruct signed speed and heading from one flat prediction stage.
  static std::array<double, 2> reconstructNominalPhysicalState(
    const std::array<double, 4> & flat_state,
    const std::array<double, 4> & reference_physical_state);

  // Move an angle onto the branch nearest a reference angle.
  static double unwrapToReference(double angle, double reference_angle);

  // Update every horizon-stage steering and acceleration input mapping.
  void updateVirtualInputConstraints(
    const std::array<double, 4> & current_physical_state,
    const std::vector<std::array<double, 4>> & state_reference_horizon);

  // Apply a shifted warm start that respects persistent obstacle-slot identities.
  void applyShiftedWarmStart(
    const std::array<double, 4> & current_flat_state,
    const std::vector<std::array<double, 4>> & state_reference_horizon,
    const std::vector<std::array<double, 2>> & input_reference_horizon);

  // Build the current quadratic tracking and virtual-input objective.
  GRBQuadExpr buildObjective(
    const std::vector<std::array<double, 4>> & state_reference_horizon);

  // Save one usable Gurobi solution for the next shifted warm start.
  bool storeSolutionForWarmStart();

  // Return whether a complete previous solution can be shifted.
  bool warmStartIsAvailable() const;

  // Return whether the current Gurobi status contains a usable control action.
  bool solutionIsUsable(int status) const;

  // Return a stable readable name for a Gurobi status code.
  static std::string statusName(int status);

  // Convert the first virtual input into physical steering and acceleration.
  void virtualInputToPhysicalInput(
    double v1,
    double v2,
    double heading,
    double signed_velocity,
    double & steering_angle,
    double & longitudinal_acceleration) const;

  OAMPCParameters parameters_;
  const ObstacleMap & obstacle_map_;
  bool model_available_ = false;
  std::unique_ptr<GRBEnv> environment_;
  std::unique_ptr<GRBModel> model_;

  std::vector<std::vector<GRBVar>> flat_state_variables_;
  std::vector<std::vector<GRBVar>> virtual_input_variables_;
  GRBVar slack_variable_;
  std::vector<std::vector<std::vector<GRBVar>>> alpha_variables_;
  std::vector<StageInputConstraints> input_constraints_;
  std::vector<std::vector<std::vector<GRBConstr>>> obstacle_face_constraints_;
  std::vector<std::vector<GRBConstr>> obstacle_sum_constraints_;

  int number_of_active_slots_ = 0;
  int number_of_template_edges_ = 0;
  int total_binary_variables_ = 0;
  std::vector<int> slot_obstacle_ids_;

  bool has_warm_start_ = false;
  std::vector<std::array<double, 4>> previous_flat_state_solution_;
  std::vector<std::array<double, 2>> previous_virtual_input_solution_;
  std::vector<std::vector<std::vector<double>>> previous_alpha_solution_;
  double previous_slack_solution_ = 0.0;
  std::vector<int> previous_slot_obstacle_ids_;
};

}  // namespace qcar2_oampc

#endif  // OAMPC_SOLVER_HPP_
