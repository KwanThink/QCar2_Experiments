#include "oampc_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace qcar2_oampc
{
namespace
{
constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kNominalSpeedThreshold = 1.0e-4;

// Return whether one scalar solver value is finite.
bool isFinite(double value)
{
  return std::isfinite(value);
}

}  // namespace

// Store OAMPC parameters and loaded geometry for persistent model construction.
OAMPCSolver::OAMPCSolver(
  const OAMPCParameters & parameters,
  const ObstacleMap & obstacle_map)
: parameters_(parameters), obstacle_map_(obstacle_map)
{
}

// Release the persistent Gurobi model and environment.
OAMPCSolver::~OAMPCSolver() = default;

// Build the persistent Gurobi MIQP model once at controller startup.
bool OAMPCSolver::initialize(const rclcpp::Logger & logger)
{
  try {
    if (parameters_.horizon_steps <= 0) {
      throw std::runtime_error("mpc_N must be positive.");
    }
    if (parameters_.maximum_active_obstacles <= 0) {
      throw std::runtime_error("max_active_obstacles must be positive.");
    }
    if (parameters_.big_m <= 0.0 || parameters_.gamma < 0.0 || parameters_.slack_weight < 0.0) {
      throw std::runtime_error("OAMPC obstacle parameters are invalid.");
    }
    if (!std::isfinite(parameters_.gurobi_time_limit) || parameters_.gurobi_time_limit <= 0.0) {
      throw std::runtime_error("gurobi_time_limit must be finite and positive.");
    }

    environment_ = std::make_unique<GRBEnv>(true);
    environment_->set(GRB_IntParam_OutputFlag, parameters_.gurobi_output_flag ? 1 : 0);
    environment_->start();
    model_ = std::make_unique<GRBModel>(*environment_);
    model_->set(GRB_DoubleParam_TimeLimit, parameters_.gurobi_time_limit);
    buildPersistentModel();
    model_available_ = true;
    RCLCPP_INFO(
      logger,
      "Persistent Gurobi OAMPC Mk1c model created: N=%d, obstacle slots=%d, max edges=%d, allocated binaries=%d.",
      parameters_.horizon_steps,
      number_of_active_slots_,
      number_of_template_edges_,
      total_binary_variables_);
    return true;
  } catch (const GRBException & exception) {
    RCLCPP_ERROR(
      logger,
      "Failed to initialize Gurobi OAMPC model. Gurobi error %d: %s",
      exception.getErrorCode(),
      exception.getMessage().c_str());
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(logger, "Failed to initialize Gurobi OAMPC model: %s", exception.what());
  }

  model_available_ = false;
  model_.reset();
  environment_.reset();
  return false;
}

// Build all variables and fixed constraints of the persistent MIQP model.
void OAMPCSolver::buildPersistentModel()
{
  if (!model_) {
    throw std::runtime_error("Gurobi model is not allocated.");
  }

  const int horizon_steps = parameters_.horizon_steps;
  number_of_active_slots_ = parameters_.maximum_active_obstacles;
  number_of_template_edges_ = static_cast<int>(obstacle_map_.maximumEdgeCount());
  total_binary_variables_ =
    horizon_steps * number_of_active_slots_ * number_of_template_edges_;
  slot_obstacle_ids_.assign(static_cast<std::size_t>(number_of_active_slots_), -1);

  flat_state_variables_.assign(
    static_cast<std::size_t>(horizon_steps + 1),
    std::vector<GRBVar>(4));
  for (int stage = 0; stage <= horizon_steps; ++stage) {
    for (int state_index = 0; state_index < 4; ++state_index) {
      const std::string name =
        "z_" + std::to_string(stage) + "_" + std::to_string(state_index);
      flat_state_variables_[static_cast<std::size_t>(stage)][static_cast<std::size_t>(state_index)] =
        model_->addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, name);
    }
  }

  virtual_input_variables_.assign(
    static_cast<std::size_t>(horizon_steps),
    std::vector<GRBVar>(2));
  for (int stage = 0; stage < horizon_steps; ++stage) {
    for (int input_index = 0; input_index < 2; ++input_index) {
      const std::string name =
        "v_" + std::to_string(stage) + "_" + std::to_string(input_index);
      virtual_input_variables_[static_cast<std::size_t>(stage)][static_cast<std::size_t>(input_index)] =
        model_->addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, name);
    }
  }

  slack_variable_ = model_->addVar(
    0.0,
    GRB_INFINITY,
    0.0,
    GRB_CONTINUOUS,
    "obstacle_slack");

  alpha_variables_.assign(
    static_cast<std::size_t>(number_of_active_slots_),
    std::vector<std::vector<GRBVar>>(
      static_cast<std::size_t>(number_of_template_edges_),
      std::vector<GRBVar>(static_cast<std::size_t>(horizon_steps))));
  for (int obstacle_slot = 0; obstacle_slot < number_of_active_slots_; ++obstacle_slot) {
    for (int edge_slot = 0; edge_slot < number_of_template_edges_; ++edge_slot) {
      for (int stage = 0; stage < horizon_steps; ++stage) {
        const std::string name =
          "alpha_" + std::to_string(obstacle_slot) + "_" +
          std::to_string(edge_slot) + "_" + std::to_string(stage);
        alpha_variables_[static_cast<std::size_t>(obstacle_slot)]
          [static_cast<std::size_t>(edge_slot)]
          [static_cast<std::size_t>(stage)] =
          model_->addVar(0.0, 0.0, 0.0, GRB_BINARY, name);
      }
    }
  }

  const double sample_time = parameters_.sample_time;
  const double half_sample_time_squared = 0.5 * sample_time * sample_time;
  for (int stage = 0; stage < horizon_steps; ++stage) {
    const auto & current_state = flat_state_variables_[static_cast<std::size_t>(stage)];
    const auto & next_state = flat_state_variables_[static_cast<std::size_t>(stage + 1)];
    const auto & input = virtual_input_variables_[static_cast<std::size_t>(stage)];
    model_->addConstr(
      next_state[0] == current_state[0] + sample_time * current_state[1] +
      half_sample_time_squared * input[0],
      "flat_dynamics_X_" + std::to_string(stage));
    model_->addConstr(
      next_state[1] == current_state[1] + sample_time * input[0],
      "flat_dynamics_Xdot_" + std::to_string(stage));
    model_->addConstr(
      next_state[2] == current_state[2] + sample_time * current_state[3] +
      half_sample_time_squared * input[1],
      "flat_dynamics_Y_" + std::to_string(stage));
    model_->addConstr(
      next_state[3] == current_state[3] + sample_time * input[1],
      "flat_dynamics_Ydot_" + std::to_string(stage));
  }

  const Workspace & workspace = obstacle_map_.workspace();
  const double x_lower = workspace.x_min + parameters_.gamma;
  const double x_upper = workspace.x_max - parameters_.gamma;
  const double y_lower = workspace.y_min + parameters_.gamma;
  const double y_upper = workspace.y_max - parameters_.gamma;
  if (x_lower >= x_upper || y_lower >= y_upper) {
    throw std::runtime_error("Workspace is too small for the configured gamma safety buffer.");
  }
  for (int stage = 1; stage <= horizon_steps; ++stage) {
    const auto & state = flat_state_variables_[static_cast<std::size_t>(stage)];
    model_->addConstr(state[0] >= x_lower, "workspace_X_lower_" + std::to_string(stage));
    model_->addConstr(state[0] <= x_upper, "workspace_X_upper_" + std::to_string(stage));
    model_->addConstr(state[2] >= y_lower, "workspace_Y_lower_" + std::to_string(stage));
    model_->addConstr(state[2] <= y_upper, "workspace_Y_upper_" + std::to_string(stage));
  }

  input_constraints_.clear();
  input_constraints_.reserve(static_cast<std::size_t>(horizon_steps));
  const double steering_lower_bound = std::tan(parameters_.delta_min);
  const double steering_upper_bound = std::tan(parameters_.delta_max);
  for (int stage = 0; stage < horizon_steps; ++stage) {
    GRBLinExpr steering_expression = 0.0;
    GRBLinExpr acceleration_expression = 0.0;
    StageInputConstraints constraints;
    constraints.steering_lower = model_->addConstr(
      steering_expression >= steering_lower_bound,
      "W_tan_delta_lower_" + std::to_string(stage));
    constraints.steering_upper = model_->addConstr(
      steering_expression <= steering_upper_bound,
      "W_tan_delta_upper_" + std::to_string(stage));
    constraints.acceleration_lower = model_->addConstr(
      acceleration_expression >= parameters_.ax_min,
      "W_ax_lower_" + std::to_string(stage));
    constraints.acceleration_upper = model_->addConstr(
      acceleration_expression <= parameters_.ax_max,
      "W_ax_upper_" + std::to_string(stage));
    input_constraints_.push_back(constraints);
  }

  obstacle_face_constraints_.assign(
    static_cast<std::size_t>(number_of_active_slots_),
    std::vector<std::vector<GRBConstr>>(
      static_cast<std::size_t>(number_of_template_edges_),
      std::vector<GRBConstr>(static_cast<std::size_t>(horizon_steps))));
  obstacle_sum_constraints_.assign(
    static_cast<std::size_t>(number_of_active_slots_),
    std::vector<GRBConstr>(static_cast<std::size_t>(horizon_steps)));

  for (int obstacle_slot = 0; obstacle_slot < number_of_active_slots_; ++obstacle_slot) {
    for (int stage = 0; stage < horizon_steps; ++stage) {
      GRBLinExpr alpha_sum = 0.0;
      for (int edge_slot = 0; edge_slot < number_of_template_edges_; ++edge_slot) {
        alpha_sum += alpha_variables_[static_cast<std::size_t>(obstacle_slot)]
          [static_cast<std::size_t>(edge_slot)]
          [static_cast<std::size_t>(stage)];
      }
      obstacle_sum_constraints_[static_cast<std::size_t>(obstacle_slot)]
        [static_cast<std::size_t>(stage)] = model_->addConstr(
        alpha_sum >= 0.0,
        "obs_alpha_sum_s" + std::to_string(obstacle_slot) + "_j" + std::to_string(stage));

      for (int edge_slot = 0; edge_slot < number_of_template_edges_; ++edge_slot) {
        const GRBVar & alpha = alpha_variables_[static_cast<std::size_t>(obstacle_slot)]
          [static_cast<std::size_t>(edge_slot)]
          [static_cast<std::size_t>(stage)];
        GRBLinExpr expression = -slack_variable_ + parameters_.big_m * alpha;
        obstacle_face_constraints_[static_cast<std::size_t>(obstacle_slot)]
          [static_cast<std::size_t>(edge_slot)]
          [static_cast<std::size_t>(stage)] = model_->addConstr(
          expression <= parameters_.big_m,
          "obs_face_s" + std::to_string(obstacle_slot) + "_e" +
          std::to_string(edge_slot) + "_j" + std::to_string(stage));
      }
    }
  }

  model_->setObjective(GRBQuadExpr(), GRB_MINIMIZE);
  model_->update();
}

// Update one persistent obstacle slot for an active polygon or inactivity.
void OAMPCSolver::updateObstacleSlot(int obstacle_slot, const Obstacle * obstacle)
{
  const int edge_count = obstacle == nullptr ? 0 : static_cast<int>(obstacle->edgeCount());
  for (int stage = 0; stage < parameters_.horizon_steps; ++stage) {
    obstacle_sum_constraints_[static_cast<std::size_t>(obstacle_slot)]
      [static_cast<std::size_t>(stage)].set(
      GRB_DoubleAttr_RHS,
      obstacle == nullptr ? 0.0 : 1.0);

    for (int edge_slot = 0; edge_slot < number_of_template_edges_; ++edge_slot) {
      GRBVar & alpha = alpha_variables_[static_cast<std::size_t>(obstacle_slot)]
        [static_cast<std::size_t>(edge_slot)]
        [static_cast<std::size_t>(stage)];
      GRBConstr & face_constraint = obstacle_face_constraints_[static_cast<std::size_t>(obstacle_slot)]
        [static_cast<std::size_t>(edge_slot)]
        [static_cast<std::size_t>(stage)];
      const bool valid_edge = obstacle != nullptr && edge_slot < edge_count;
      alpha.set(GRB_DoubleAttr_LB, 0.0);
      alpha.set(GRB_DoubleAttr_UB, valid_edge ? 1.0 : 0.0);

      GRBVar & x_position = flat_state_variables_[static_cast<std::size_t>(stage + 1)][0];
      GRBVar & y_position = flat_state_variables_[static_cast<std::size_t>(stage + 1)][2];
      if (valid_edge) {
        const auto & row = obstacle->halfspace_matrix[static_cast<std::size_t>(edge_slot)];
        const double face_bound = obstacle->halfspace_vector[static_cast<std::size_t>(edge_slot)];
        model_->chgCoeff(face_constraint, x_position, -row[0]);
        model_->chgCoeff(face_constraint, y_position, -row[1]);
        face_constraint.set(
          GRB_DoubleAttr_RHS,
          -face_bound - parameters_.gamma + parameters_.big_m);
      } else {
        model_->chgCoeff(face_constraint, x_position, 0.0);
        model_->chgCoeff(face_constraint, y_position, 0.0);
        face_constraint.set(GRB_DoubleAttr_RHS, parameters_.big_m);
      }
    }
  }
}

// Keep persistent slot identities while assigning the current active obstacles.
void OAMPCSolver::assignActiveObstaclesToSlots(
  const std::vector<const Obstacle *> & active_obstacles)
{
  if (active_obstacles.size() > static_cast<std::size_t>(number_of_active_slots_)) {
    throw std::runtime_error("Active obstacle count exceeds the configured persistent slot count.");
  }

  std::vector<int> requested_ids;
  requested_ids.reserve(active_obstacles.size());
  for (const Obstacle * obstacle : active_obstacles) {
    if (obstacle == nullptr) {
      throw std::runtime_error("Active obstacle list contains a null obstacle.");
    }
    requested_ids.push_back(obstacle->id);
  }

  for (int obstacle_slot = 0; obstacle_slot < number_of_active_slots_; ++obstacle_slot) {
    const int current_id = slot_obstacle_ids_[static_cast<std::size_t>(obstacle_slot)];
    if (current_id < 0) {
      continue;
    }
    if (std::find(requested_ids.begin(), requested_ids.end(), current_id) == requested_ids.end()) {
      slot_obstacle_ids_[static_cast<std::size_t>(obstacle_slot)] = -1;
      updateObstacleSlot(obstacle_slot, nullptr);
    }
  }

  for (const Obstacle * obstacle : active_obstacles) {
    if (std::find(slot_obstacle_ids_.begin(), slot_obstacle_ids_.end(), obstacle->id) !=
      slot_obstacle_ids_.end())
    {
      continue;
    }
    const auto empty_slot = std::find(slot_obstacle_ids_.begin(), slot_obstacle_ids_.end(), -1);
    if (empty_slot == slot_obstacle_ids_.end()) {
      throw std::runtime_error("No free persistent obstacle slot is available.");
    }
    const int obstacle_slot = static_cast<int>(
      std::distance(slot_obstacle_ids_.begin(), empty_slot));
    slot_obstacle_ids_[static_cast<std::size_t>(obstacle_slot)] = obstacle->id;
    updateObstacleSlot(obstacle_slot, obstacle);
  }
}

// Convert a physical state into flat coordinates.
std::array<double, 4> OAMPCSolver::physicalStateToFlatState(
  const std::array<double, 4> & physical_state)
{
  const double heading = physical_state[2];
  const double signed_velocity = physical_state[3];
  return {
    physical_state[0],
    signed_velocity * std::cos(heading),
    physical_state[1],
    signed_velocity * std::sin(heading)};
}

// Convert one physical reference input into a virtual-input initial guess.
std::array<double, 2> OAMPCSolver::physicalInputToVirtualInput(
  const std::array<double, 4> & physical_state,
  const std::array<double, 2> & physical_input) const
{
  const double heading = physical_state[2];
  const double signed_velocity = physical_state[3];
  const double steering_angle = physical_input[0];
  const double longitudinal_acceleration = physical_input[1];
  const double lateral_acceleration =
    (signed_velocity * signed_velocity / parameters_.wheelbase) * std::tan(steering_angle);
  return {
    longitudinal_acceleration * std::cos(heading) - lateral_acceleration * std::sin(heading),
    longitudinal_acceleration * std::sin(heading) + lateral_acceleration * std::cos(heading)};
}

// Reconstruct signed speed and heading from one flat prediction stage.
std::array<double, 2> OAMPCSolver::reconstructNominalPhysicalState(
  const std::array<double, 4> & flat_state,
  const std::array<double, 4> & reference_physical_state)
{
  const double x_velocity = flat_state[1];
  const double y_velocity = flat_state[3];
  const double flat_speed = std::hypot(x_velocity, y_velocity);
  const double reference_heading = reference_physical_state[2];
  const double reference_signed_velocity = reference_physical_state[3];

  if (flat_speed <= kNominalSpeedThreshold) {
    return {reference_heading, reference_signed_velocity};
  }

  double direction_sign = 1.0;
  if (reference_signed_velocity < -kNominalSpeedThreshold) {
    direction_sign = -1.0;
  } else if (reference_signed_velocity <= kNominalSpeedThreshold) {
    const double velocity_along_reference_heading =
      x_velocity * std::cos(reference_heading) + y_velocity * std::sin(reference_heading);
    direction_sign = velocity_along_reference_heading < 0.0 ? -1.0 : 1.0;
  }

  double body_heading = std::atan2(y_velocity, x_velocity);
  if (direction_sign < 0.0) {
    body_heading += kPi;
  }
  body_heading = unwrapToReference(body_heading, reference_heading);
  return {body_heading, direction_sign * flat_speed};
}

// Move an angle onto the branch nearest a reference angle.
double OAMPCSolver::unwrapToReference(double angle, double reference_angle)
{
  double unwrapped = angle;
  while (unwrapped - reference_angle > kPi) {
    unwrapped -= 2.0 * kPi;
  }
  while (unwrapped - reference_angle < -kPi) {
    unwrapped += 2.0 * kPi;
  }
  return unwrapped;
}

// Update every horizon-stage steering and acceleration input mapping.
void OAMPCSolver::updateVirtualInputConstraints(
  const std::array<double, 4> & current_physical_state,
  const std::vector<std::array<double, 4>> & state_reference_horizon)
{
  const bool use_previous_prediction = warmStartIsAvailable();
  for (int stage = 0; stage < parameters_.horizon_steps; ++stage) {
    double nominal_heading = current_physical_state[2];
    double nominal_signed_velocity = current_physical_state[3];
    if (stage > 0) {
      if (use_previous_prediction) {
        const std::size_t previous_state_index = std::min(
          static_cast<std::size_t>(stage + 1),
          previous_flat_state_solution_.size() - 1U);
        const std::array<double, 2> nominal_physical_state = reconstructNominalPhysicalState(
          previous_flat_state_solution_[previous_state_index],
          state_reference_horizon[static_cast<std::size_t>(stage)]);
        nominal_heading = nominal_physical_state[0];
        nominal_signed_velocity = nominal_physical_state[1];
      } else {
        nominal_heading = state_reference_horizon[static_cast<std::size_t>(stage)][2];
        nominal_signed_velocity = state_reference_horizon[static_cast<std::size_t>(stage)][3];
      }
    }

    const double denominator =
      nominal_signed_velocity * nominal_signed_velocity + parameters_.epsilon;
    const double steering_scale = parameters_.wheelbase / denominator;
    const double steering_v1 = -steering_scale * std::sin(nominal_heading);
    const double steering_v2 = steering_scale * std::cos(nominal_heading);
    const double acceleration_v1 = std::cos(nominal_heading);
    const double acceleration_v2 = std::sin(nominal_heading);

    GRBVar & v1 = virtual_input_variables_[static_cast<std::size_t>(stage)][0];
    GRBVar & v2 = virtual_input_variables_[static_cast<std::size_t>(stage)][1];
    StageInputConstraints & constraints = input_constraints_[static_cast<std::size_t>(stage)];
    model_->chgCoeff(constraints.steering_lower, v1, steering_v1);
    model_->chgCoeff(constraints.steering_lower, v2, steering_v2);
    model_->chgCoeff(constraints.steering_upper, v1, steering_v1);
    model_->chgCoeff(constraints.steering_upper, v2, steering_v2);
    model_->chgCoeff(constraints.acceleration_lower, v1, acceleration_v1);
    model_->chgCoeff(constraints.acceleration_lower, v2, acceleration_v2);
    model_->chgCoeff(constraints.acceleration_upper, v1, acceleration_v1);
    model_->chgCoeff(constraints.acceleration_upper, v2, acceleration_v2);
  }
}

// Apply a shifted warm start that respects persistent obstacle-slot identities.
void OAMPCSolver::applyShiftedWarmStart(
  const std::array<double, 4> & current_flat_state,
  const std::vector<std::array<double, 4>> & state_reference_horizon,
  const std::vector<std::array<double, 2>> & input_reference_horizon)
{
  const std::size_t horizon_steps = static_cast<std::size_t>(parameters_.horizon_steps);
  for (std::size_t state_index = 0; state_index < 4U; ++state_index) {
    flat_state_variables_[0][state_index].set(
      GRB_DoubleAttr_Start,
      current_flat_state[state_index]);
  }

  if (!warmStartIsAvailable()) {
    for (std::size_t stage = 1U; stage <= horizon_steps; ++stage) {
      const std::array<double, 4> flat_reference =
        physicalStateToFlatState(state_reference_horizon[stage]);
      for (std::size_t state_index = 0; state_index < 4U; ++state_index) {
        flat_state_variables_[stage][state_index].set(
          GRB_DoubleAttr_Start,
          flat_reference[state_index]);
      }
    }
    for (std::size_t stage = 0U; stage < horizon_steps; ++stage) {
      const std::array<double, 2> virtual_reference = physicalInputToVirtualInput(
        state_reference_horizon[stage],
        input_reference_horizon[stage]);
      virtual_input_variables_[stage][0].set(GRB_DoubleAttr_Start, virtual_reference[0]);
      virtual_input_variables_[stage][1].set(GRB_DoubleAttr_Start, virtual_reference[1]);
    }
    for (int obstacle_slot = 0; obstacle_slot < number_of_active_slots_; ++obstacle_slot) {
      const Obstacle * obstacle = obstacle_map_.findObstacleById(
        slot_obstacle_ids_[static_cast<std::size_t>(obstacle_slot)]);
      const int edge_count = obstacle == nullptr ? 0 : static_cast<int>(obstacle->edgeCount());
      for (int edge_slot = 0; edge_slot < number_of_template_edges_; ++edge_slot) {
        for (int stage = 0; stage < parameters_.horizon_steps; ++stage) {
          GRBVar & alpha = alpha_variables_[static_cast<std::size_t>(obstacle_slot)]
            [static_cast<std::size_t>(edge_slot)]
            [static_cast<std::size_t>(stage)];
          alpha.set(
            GRB_DoubleAttr_Start,
            edge_slot < edge_count ? GRB_UNDEFINED : 0.0);
        }
      }
    }
    slack_variable_.set(GRB_DoubleAttr_Start, 0.0);
    return;
  }

  for (std::size_t stage = 1U; stage < horizon_steps; ++stage) {
    const std::array<double, 4> & shifted_state = previous_flat_state_solution_[stage + 1U];
    for (std::size_t state_index = 0; state_index < 4U; ++state_index) {
      flat_state_variables_[stage][state_index].set(
        GRB_DoubleAttr_Start,
        shifted_state[state_index]);
    }
  }

  const std::array<double, 4> & last_state = previous_flat_state_solution_.back();
  const std::array<double, 2> & last_input = previous_virtual_input_solution_.back();
  const double sample_time = parameters_.sample_time;
  const double half_sample_time_squared = 0.5 * sample_time * sample_time;
  const std::array<double, 4> terminal_state{{
    last_state[0] + sample_time * last_state[1] + half_sample_time_squared * last_input[0],
    last_state[1] + sample_time * last_input[0],
    last_state[2] + sample_time * last_state[3] + half_sample_time_squared * last_input[1],
    last_state[3] + sample_time * last_input[1]}};
  for (std::size_t state_index = 0; state_index < 4U; ++state_index) {
    flat_state_variables_[horizon_steps][state_index].set(
      GRB_DoubleAttr_Start,
      terminal_state[state_index]);
  }

  for (std::size_t stage = 0U; stage < horizon_steps; ++stage) {
    const std::size_t source_stage = std::min(stage + 1U, horizon_steps - 1U);
    virtual_input_variables_[stage][0].set(
      GRB_DoubleAttr_Start,
      previous_virtual_input_solution_[source_stage][0]);
    virtual_input_variables_[stage][1].set(
      GRB_DoubleAttr_Start,
      previous_virtual_input_solution_[source_stage][1]);
  }

  for (int obstacle_slot = 0; obstacle_slot < number_of_active_slots_; ++obstacle_slot) {
    const int obstacle_id = slot_obstacle_ids_[static_cast<std::size_t>(obstacle_slot)];
    const int previous_obstacle_id =
      previous_slot_obstacle_ids_[static_cast<std::size_t>(obstacle_slot)];
    const Obstacle * obstacle = obstacle_map_.findObstacleById(obstacle_id);
    const int edge_count = obstacle == nullptr ? 0 : static_cast<int>(obstacle->edgeCount());
    const bool remains_active = obstacle_id >= 0 && obstacle_id == previous_obstacle_id;

    for (int edge_slot = 0; edge_slot < number_of_template_edges_; ++edge_slot) {
      const bool valid_edge = obstacle != nullptr && edge_slot < edge_count;
      for (std::size_t stage = 0U; stage < horizon_steps; ++stage) {
        GRBVar & alpha = alpha_variables_[static_cast<std::size_t>(obstacle_slot)]
          [static_cast<std::size_t>(edge_slot)][stage];
        if (!valid_edge) {
          alpha.set(GRB_DoubleAttr_Start, 0.0);
        } else if (remains_active) {
          const std::size_t source_stage = std::min(stage + 1U, horizon_steps - 1U);
          const double previous_value = previous_alpha_solution_[static_cast<std::size_t>(obstacle_slot)]
            [static_cast<std::size_t>(edge_slot)][source_stage];
          alpha.set(GRB_DoubleAttr_Start, std::round(previous_value));
        } else {
          alpha.set(GRB_DoubleAttr_Start, GRB_UNDEFINED);
        }
      }
    }
  }
  slack_variable_.set(GRB_DoubleAttr_Start, std::max(0.0, previous_slack_solution_));
}

// Build the current quadratic tracking and virtual-input objective.
GRBQuadExpr OAMPCSolver::buildObjective(
  const std::vector<std::array<double, 4>> & state_reference_horizon)
{
  GRBQuadExpr objective = 0.0;
  for (int stage = 0; stage < parameters_.horizon_steps; ++stage) {
    const std::size_t stage_index = static_cast<std::size_t>(stage);
    GRBVar & x_position = flat_state_variables_[stage_index + 1U][0];
    GRBVar & y_position = flat_state_variables_[stage_index + 1U][2];
    GRBVar & v1 = virtual_input_variables_[stage_index][0];
    GRBVar & v2 = virtual_input_variables_[stage_index][1];
    const double x_reference = state_reference_horizon[stage_index + 1U][0];
    const double y_reference = state_reference_horizon[stage_index + 1U][1];
    const GRBLinExpr x_error = x_position - x_reference;
    const GRBLinExpr y_error = y_position - y_reference;
    objective += parameters_.position_weights[0] * x_error * x_error;
    objective += parameters_.position_weights[1] * y_error * y_error;
    objective += parameters_.virtual_input_weights[0] * v1 * v1;
    objective += parameters_.virtual_input_weights[1] * v2 * v2;
  }
  objective += parameters_.slack_weight * slack_variable_;
  return objective;
}

// Save one usable Gurobi solution for the next shifted warm start.
bool OAMPCSolver::storeSolutionForWarmStart()
{
  const std::size_t horizon_steps = static_cast<std::size_t>(parameters_.horizon_steps);
  std::vector<std::array<double, 4>> flat_state_solution(horizon_steps + 1U);
  std::vector<std::array<double, 2>> virtual_input_solution(horizon_steps);

  for (std::size_t stage = 0U; stage <= horizon_steps; ++stage) {
    for (std::size_t state_index = 0U; state_index < 4U; ++state_index) {
      const double value = flat_state_variables_[stage][state_index].get(GRB_DoubleAttr_X);
      if (!isFinite(value)) {
        return false;
      }
      flat_state_solution[stage][state_index] = value;
    }
  }
  for (std::size_t stage = 0U; stage < horizon_steps; ++stage) {
    for (std::size_t input_index = 0U; input_index < 2U; ++input_index) {
      const double value = virtual_input_variables_[stage][input_index].get(GRB_DoubleAttr_X);
      if (!isFinite(value)) {
        return false;
      }
      virtual_input_solution[stage][input_index] = value;
    }
  }

  std::vector<std::vector<std::vector<double>>> alpha_solution(
    static_cast<std::size_t>(number_of_active_slots_),
    std::vector<std::vector<double>>(
      static_cast<std::size_t>(number_of_template_edges_),
      std::vector<double>(horizon_steps, 0.0)));
  for (int obstacle_slot = 0; obstacle_slot < number_of_active_slots_; ++obstacle_slot) {
    for (int edge_slot = 0; edge_slot < number_of_template_edges_; ++edge_slot) {
      for (std::size_t stage = 0U; stage < horizon_steps; ++stage) {
        const double value = alpha_variables_[static_cast<std::size_t>(obstacle_slot)]
          [static_cast<std::size_t>(edge_slot)][stage].get(GRB_DoubleAttr_X);
        if (!isFinite(value)) {
          return false;
        }
        alpha_solution[static_cast<std::size_t>(obstacle_slot)]
          [static_cast<std::size_t>(edge_slot)][stage] = value;
      }
    }
  }

  const double slack_value = slack_variable_.get(GRB_DoubleAttr_X);
  if (!isFinite(slack_value)) {
    return false;
  }

  previous_flat_state_solution_ = std::move(flat_state_solution);
  previous_virtual_input_solution_ = std::move(virtual_input_solution);
  previous_alpha_solution_ = std::move(alpha_solution);
  previous_slack_solution_ = std::max(0.0, slack_value);
  previous_slot_obstacle_ids_ = slot_obstacle_ids_;
  has_warm_start_ = true;
  return true;
}

// Return whether a complete previous solution can be shifted.
bool OAMPCSolver::warmStartIsAvailable() const
{
  const std::size_t horizon_steps = static_cast<std::size_t>(parameters_.horizon_steps);
  return has_warm_start_ &&
         previous_flat_state_solution_.size() == horizon_steps + 1U &&
         previous_virtual_input_solution_.size() == horizon_steps &&
         previous_alpha_solution_.size() == static_cast<std::size_t>(number_of_active_slots_) &&
         previous_slot_obstacle_ids_.size() == static_cast<std::size_t>(number_of_active_slots_);
}

// Return whether the current Gurobi status contains a usable control action.
bool OAMPCSolver::solutionIsUsable(int status) const
{
  if (status == GRB_OPTIMAL) {
    return true;
  }
  const bool allowed_suboptimal_status =
    status == GRB_SUBOPTIMAL || status == GRB_TIME_LIMIT ||
    status == GRB_INTERRUPTED || status == GRB_USER_OBJ_LIMIT;
  return allowed_suboptimal_status && model_->get(GRB_IntAttr_SolCount) > 0;
}

// Return a stable readable name for a Gurobi status code.
std::string OAMPCSolver::statusName(int status)
{
  switch (status) {
    case GRB_LOADED:
      return "LOADED";
    case GRB_OPTIMAL:
      return "OPTIMAL";
    case GRB_INFEASIBLE:
      return "INFEASIBLE";
    case GRB_INF_OR_UNBD:
      return "INF_OR_UNBD";
    case GRB_UNBOUNDED:
      return "UNBOUNDED";
    case GRB_CUTOFF:
      return "CUTOFF";
    case GRB_ITERATION_LIMIT:
      return "ITERATION_LIMIT";
    case GRB_NODE_LIMIT:
      return "NODE_LIMIT";
    case GRB_TIME_LIMIT:
      return "TIME_LIMIT";
    case GRB_SOLUTION_LIMIT:
      return "SOLUTION_LIMIT";
    case GRB_INTERRUPTED:
      return "INTERRUPTED";
    case GRB_NUMERIC:
      return "NUMERIC";
    case GRB_SUBOPTIMAL:
      return "SUBOPTIMAL";
    case GRB_INPROGRESS:
      return "INPROGRESS";
    case GRB_USER_OBJ_LIMIT:
      return "USER_OBJ_LIMIT";
    default:
      return "STATUS_" + std::to_string(status);
  }
}

// Convert the first virtual input into physical steering and acceleration.
void OAMPCSolver::virtualInputToPhysicalInput(
  double v1,
  double v2,
  double heading,
  double signed_velocity,
  double & steering_angle,
  double & longitudinal_acceleration) const
{
  const double denominator = signed_velocity * signed_velocity + parameters_.epsilon;
  const double steering_argument =
    (parameters_.wheelbase / denominator) *
    (-v1 * std::sin(heading) + v2 * std::cos(heading));
  steering_angle = std::atan(steering_argument);
  longitudinal_acceleration = v1 * std::cos(heading) + v2 * std::sin(heading);
}

// Solve one OAMPC step with stage-dependent input sets and active obstacles.
SolverResult OAMPCSolver::solve(
  const std::array<double, 4> & current_physical_state,
  const std::vector<std::array<double, 4>> & state_reference_horizon,
  const std::vector<std::array<double, 2>> & input_reference_horizon,
  const std::vector<const Obstacle *> & active_obstacles)
{
  SolverResult result;
  result.number_of_config_obstacles = static_cast<int>(obstacle_map_.obstacles().size());
  result.active_obstacles = static_cast<int>(active_obstacles.size());
  for (const Obstacle * obstacle : active_obstacles) {
    if (obstacle != nullptr) {
      result.active_edges += static_cast<int>(obstacle->edgeCount());
    }
  }
  result.binary_variables = parameters_.horizon_steps * result.active_edges;
  result.total_binary_variables = total_binary_variables_;

  if (!model_available_ || !model_) {
    result.status = -98;
    result.status_name = "MODEL_UNAVAILABLE";
    return result;
  }

  const std::size_t horizon_steps = static_cast<std::size_t>(parameters_.horizon_steps);
  if (state_reference_horizon.size() != horizon_steps + 1U ||
    input_reference_horizon.size() != horizon_steps)
  {
    resetWarmStart();
    result.status = -97;
    result.status_name = "INVALID_HORIZON";
    return result;
  }

  try {
    assignActiveObstaclesToSlots(active_obstacles);
    const std::array<double, 4> current_flat_state =
      physicalStateToFlatState(current_physical_state);
    for (std::size_t state_index = 0U; state_index < 4U; ++state_index) {
      GRBVar & variable = flat_state_variables_[0][state_index];
      variable.set(GRB_DoubleAttr_LB, current_flat_state[state_index]);
      variable.set(GRB_DoubleAttr_UB, current_flat_state[state_index]);
    }

    updateVirtualInputConstraints(current_physical_state, state_reference_horizon);
    applyShiftedWarmStart(
      current_flat_state,
      state_reference_horizon,
      input_reference_horizon);
    model_->setObjective(buildObjective(state_reference_horizon), GRB_MINIMIZE);
    model_->update();
    model_->optimize();

    result.status = model_->get(GRB_IntAttr_Status);
    result.status_name = "MIQP_" + statusName(result.status);
    result.success = solutionIsUsable(result.status);
    if (!result.success) {
      resetWarmStart();
      return result;
    }

    result.v1 = virtual_input_variables_[0][0].get(GRB_DoubleAttr_X);
    result.v2 = virtual_input_variables_[0][1].get(GRB_DoubleAttr_X);
    result.slack_value = std::max(0.0, slack_variable_.get(GRB_DoubleAttr_X));
    virtualInputToPhysicalInput(
      result.v1,
      result.v2,
      current_physical_state[2],
      current_physical_state[3],
      result.delta,
      result.ax);
    if (!isFinite(result.v1) || !isFinite(result.v2) ||
      !isFinite(result.delta) || !isFinite(result.ax) || !isFinite(result.slack_value))
    {
      resetWarmStart();
      result.status = -96;
      result.status_name = "NONFINITE_SOLUTION";
      result.success = false;
      return result;
    }

    if (!storeSolutionForWarmStart()) {
      resetWarmStart();
      result.status = -95;
      result.status_name = "INVALID_WARM_START_SOLUTION";
      result.success = false;
      return result;
    }
    return result;
  } catch (const GRBException &) {
    resetWarmStart();
    result.status = -100;
    result.status_name = "MIQP_EXCEPTION";
    result.success = false;
    return result;
  } catch (const std::exception &) {
    resetWarmStart();
    result.status = -101;
    result.status_name = "SOLVER_EXCEPTION";
    result.success = false;
    return result;
  }
}

// Clear every cached solution and explicit Gurobi start value.
void OAMPCSolver::resetWarmStart()
{
  has_warm_start_ = false;
  previous_flat_state_solution_.clear();
  previous_virtual_input_solution_.clear();
  previous_alpha_solution_.clear();
  previous_slack_solution_ = 0.0;
  previous_slot_obstacle_ids_.clear();

  if (!model_) {
    return;
  }
  for (auto & stage : flat_state_variables_) {
    for (GRBVar & variable : stage) {
      variable.set(GRB_DoubleAttr_Start, GRB_UNDEFINED);
    }
  }
  for (auto & stage : virtual_input_variables_) {
    for (GRBVar & variable : stage) {
      variable.set(GRB_DoubleAttr_Start, GRB_UNDEFINED);
    }
  }
  for (auto & obstacle_slot : alpha_variables_) {
    for (auto & edge_slot : obstacle_slot) {
      for (GRBVar & variable : edge_slot) {
        variable.set(GRB_DoubleAttr_Start, GRB_UNDEFINED);
      }
    }
  }
  slack_variable_.set(GRB_DoubleAttr_Start, GRB_UNDEFINED);
}

}  // namespace qcar2_oampc
