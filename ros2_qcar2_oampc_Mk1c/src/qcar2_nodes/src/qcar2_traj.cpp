#include "qcar2_traj.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace qcar2_flmpc
{
namespace
{
constexpr double kEpsilon = 1.0e-9;
constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kMinimumSpeedForCurvature = 0.03;

// Return the Euclidean distance between two planar points.
double distanceBetween(
  const QCar2BezierTrajectoryGenerator::Point2D & a,
  const QCar2BezierTrajectoryGenerator::Point2D & b)
{
  const double dx = b[0] - a[0];
  const double dy = b[1] - a[1];
  return std::hypot(dx, dy);
}

// Return a normalized 2D vector; near-zero vectors are replaced by zero.
QCar2BezierTrajectoryGenerator::Point2D unitVector(
  const QCar2BezierTrajectoryGenerator::Point2D & vector)
{
  const double norm = std::hypot(vector[0], vector[1]);
  if (norm < kEpsilon) {
    return {0.0, 0.0};
  }
  return {vector[0] / norm, vector[1] / norm};
}

// Return the signed 2D cross product used by the curvature estimate.
double cross2(
  const QCar2BezierTrajectoryGenerator::Point2D & a,
  const QCar2BezierTrajectoryGenerator::Point2D & b)
{
  return a[0] * b[1] - a[1] * b[0];
}

// Keep a heading continuous with respect to a previous heading value.
double unwrapToPrevious(double angle, double previous_angle)
{
  double unwrapped = angle;
  while (unwrapped - previous_angle > kPi) {
    unwrapped -= 2.0 * kPi;
  }
  while (unwrapped - previous_angle < -kPi) {
    unwrapped += 2.0 * kPi;
  }
  return unwrapped;
}

// Return true when a trajectory scalar is finite.
bool isFinite(double value)
{
  return std::isfinite(value);
}

// Reject non-finite trajectory parameters with a clear configuration error.
void requireFinite(double value, const std::string & name)
{
  if (!isFinite(value)) {
    throw std::runtime_error(name + " must be finite.");
  }
}

// Compute one Bernstein basis value of degree n.
double bernstein(int degree, int index, double s)
{
  static constexpr double binomial_5[6] = {1.0, 5.0, 10.0, 10.0, 5.0, 1.0};
  static constexpr double binomial_4[5] = {1.0, 4.0, 6.0, 4.0, 1.0};
  static constexpr double binomial_3[4] = {1.0, 3.0, 3.0, 1.0};

  double coefficient = 1.0;
  if (degree == 5) {
    coefficient = binomial_5[index];
  } else if (degree == 4) {
    coefficient = binomial_4[index];
  } else if (degree == 3) {
    coefficient = binomial_3[index];
  }

  return coefficient * std::pow(1.0 - s, degree - index) * std::pow(s, index);
}

}  // namespace

// Store and validate all fixed-reference trajectory parameters.
QCar2BezierTrajectoryGenerator::QCar2BezierTrajectoryGenerator(const TrajectoryParameters & parameters)
: parameters_(parameters)
{
  validateFixedReferenceConfig();
}

// Validate the fixed reference waypoints, timing, and heading configuration.
void QCar2BezierTrajectoryGenerator::validateFixedReferenceConfig() const
{
  requireFinite(parameters_.wheelbase, "wheelbase");
  requireFinite(parameters_.sample_time, "sample_time");
  requireFinite(parameters_.minimum_segment_time, "minimum_segment_time");
  requireFinite(parameters_.intermediate_tangent_scale, "intermediate_tangent_scale");
  requireFinite(parameters_.minimum_goal_distance, "minimum_goal_distance");
  requireFinite(parameters_.theta_start, "theta_start");
  requireFinite(parameters_.theta_end, "theta_end");

  if (parameters_.wheelbase <= kEpsilon) {
    throw std::runtime_error("wheelbase must be positive.");
  }
  if (parameters_.sample_time <= kEpsilon) {
    throw std::runtime_error("sample_time must be positive.");
  }
  if (parameters_.minimum_segment_time <= kEpsilon) {
    throw std::runtime_error("minimum_segment_time must be positive.");
  }
  if (parameters_.number_of_waypoints_start < 2) {
    throw std::runtime_error("number_of_waypoints_start must be at least 2.");
  }
  if (parameters_.segment_times_start.size() !=
    static_cast<std::size_t>(parameters_.number_of_waypoints_start - 1))
  {
    std::ostringstream message;
    message << "segment_times_start size must be number_of_waypoints_start - 1. Expected "
            << (parameters_.number_of_waypoints_start - 1) << ", got "
            << parameters_.segment_times_start.size() << ".";
    throw std::runtime_error(message.str());
  }
  if (parameters_.waypoints_ref_xy.size() < 4 || parameters_.waypoints_ref_xy.size() % 2 != 0) {
    throw std::runtime_error(
      "waypoints_ref_xy must be a flat even-size list containing at least two waypoints [x0,y0,x1,y1,...].");
  }

  const std::size_t number_of_ref_waypoints = parameters_.waypoints_ref_xy.size() / 2;
  if (parameters_.segment_times_ref.size() != number_of_ref_waypoints - 1) {
    std::ostringstream message;
    message << "segment_times_ref size must be number_of_ref_waypoints - 1. Expected "
            << (number_of_ref_waypoints - 1) << ", got " << parameters_.segment_times_ref.size() << ".";
    throw std::runtime_error(message.str());
  }

  for (std::size_t i = 0; i < parameters_.segment_times_start.size(); ++i) {
    const double segment_time = parameters_.segment_times_start[i];
    if (!isFinite(segment_time) || segment_time < parameters_.minimum_segment_time) {
      std::ostringstream message;
      message << "segment_times_start[" << i << "] must be finite and >= minimum_segment_time.";
      throw std::runtime_error(message.str());
    }
  }
  for (std::size_t i = 0; i < parameters_.segment_times_ref.size(); ++i) {
    const double segment_time = parameters_.segment_times_ref[i];
    if (!isFinite(segment_time) || segment_time < parameters_.minimum_segment_time) {
      std::ostringstream message;
      message << "segment_times_ref[" << i << "] must be finite and >= minimum_segment_time.";
      throw std::runtime_error(message.str());
    }
  }
  for (std::size_t i = 0; i < parameters_.waypoints_ref_xy.size(); ++i) {
    if (!isFinite(parameters_.waypoints_ref_xy[i])) {
      std::ostringstream message;
      message << "waypoints_ref_xy[" << i << "] must be finite.";
      throw std::runtime_error(message.str());
    }
  }
}

// Convert the flat YAML waypoint list into planar reference points.
std::vector<QCar2BezierTrajectoryGenerator::Point2D>
QCar2BezierTrajectoryGenerator::parseReferenceWaypoints() const
{
  validateFixedReferenceConfig();

  std::vector<Point2D> points;
  points.reserve(parameters_.waypoints_ref_xy.size() / 2);
  for (std::size_t i = 0; i + 1 < parameters_.waypoints_ref_xy.size(); i += 2) {
    points.push_back({parameters_.waypoints_ref_xy[i], parameters_.waypoints_ref_xy[i + 1]});
  }
  return points;
}

// Return the fixed map waypoints used for RViz2 preview.
std::vector<QCar2BezierTrajectoryGenerator::Point2D>
QCar2BezierTrajectoryGenerator::referenceWaypoints() const
{
  return parseReferenceWaypoints();
}

// Generate S0 -> automatic connector -> P0 -> ... -> PN as one sampled trajectory.
ReferenceTrajectory QCar2BezierTrajectoryGenerator::generateFullReferenceTrajectory(
  const Pose2D & start_pose) const
{
  requireFinite(start_pose.x, "start_pose.x");
  requireFinite(start_pose.y, "start_pose.y");
  requireFinite(start_pose.yaw, "start_pose.yaw");

  const auto ref_points = parseReferenceWaypoints();
  const Pose2D p0_pose{ref_points.front()[0], ref_points.front()[1], parameters_.theta_start};
  const Point2D start_point{start_pose.x, start_pose.y};
  const double distance_s0_to_p0 = distanceBetween(start_point, ref_points.front());

  std::vector<Point2D> full_points;
  std::vector<double> full_segment_times;
  std::size_t p0_index = 0;

  if (distance_s0_to_p0 < parameters_.minimum_goal_distance) {
    // The car is already at, or very close to, P0. Do not create a nearly-zero
    // S0 -> P0 Hermite connector because that would either fail validation or
    // make a bad stop-like point at P0. Start directly on the fixed reference.
    // Keep the actual TF position as the first point so the active path begins at S0.
    full_points = ref_points;
    full_points.front() = start_point;
    full_segment_times = parameters_.segment_times_ref;
    p0_index = 0;
  } else {
    // Normal case: build S0 -> A1 -> ... -> P0 first, then append P1 -> ... -> PN.
    full_points = buildStartGeometricWaypoints(start_pose, p0_pose);
    p0_index = full_points.size() - 1;
    full_points.reserve(full_points.size() + ref_points.size() - 1);
    for (std::size_t i = 1; i < ref_points.size(); ++i) {
      full_points.push_back(ref_points[i]);
    }

    full_segment_times = parameters_.segment_times_start;
    full_segment_times.reserve(parameters_.segment_times_start.size() + parameters_.segment_times_ref.size());
    full_segment_times.insert(
      full_segment_times.end(),
      parameters_.segment_times_ref.begin(),
      parameters_.segment_times_ref.end());
  }

  ReferenceTrajectory trajectory;
  trajectory.segment_times = full_segment_times;
  trajectory.boundary_waypoints = buildBoundaryWaypoints(
    full_points,
    trajectory.segment_times,
    start_pose.yaw,
    p0_index);

  double time_offset = 0.0;
  for (std::size_t segment_index = 0; segment_index < trajectory.segment_times.size(); ++segment_index) {
    const auto control_points = computeBezier5ControlPoints(
      trajectory.boundary_waypoints[segment_index],
      trajectory.boundary_waypoints[segment_index + 1],
      trajectory.segment_times[segment_index]);

    const bool include_endpoint = segment_index == trajectory.segment_times.size() - 1;
    appendBezierSegmentSamples(
      control_points,
      trajectory.boundary_waypoints[segment_index],
      trajectory.boundary_waypoints[segment_index + 1],
      trajectory.segment_times[segment_index],
      time_offset,
      static_cast<int>(segment_index),
      include_endpoint,
      trajectory);

    time_offset += trajectory.segment_times[segment_index];
  }

  if (!trajectory.samples.empty()) {
    auto & first_sample = trajectory.samples.front();
    const auto & first_boundary = trajectory.boundary_waypoints.front();
    first_sample.x = first_boundary.x;
    first_sample.y = first_boundary.y;
    first_sample.psi = start_pose.yaw;
    first_sample.vx = 0.0;
    first_sample.delta = parameters_.zero_endpoint_steering ? 0.0 : first_boundary.delta;
    first_sample.ax = 0.0;

    for (std::size_t i = 1; i < trajectory.samples.size(); ++i) {
      trajectory.samples[i].psi = unwrapToPrevious(
        trajectory.samples[i].psi,
        trajectory.samples[i - 1].psi);
    }

    auto & last_sample = trajectory.samples.back();
    const auto & last_boundary = trajectory.boundary_waypoints.back();
    last_sample.x = last_boundary.x;
    last_sample.y = last_boundary.y;
    if (trajectory.samples.size() >= 2) {
      last_sample.psi = unwrapToPrevious(
        parameters_.theta_end,
        trajectory.samples[trajectory.samples.size() - 2].psi);
    } else {
      last_sample.psi = parameters_.theta_end;
    }
    last_sample.vx = 0.0;
    last_sample.delta = parameters_.zero_endpoint_steering ? 0.0 : last_boundary.delta;
    last_sample.ax = 0.0;
  }

  return trajectory;
}

// Using Hermite curves to compute start geometry S0 -> P0.
std::vector<QCar2BezierTrajectoryGenerator::Point2D>
QCar2BezierTrajectoryGenerator::buildStartGeometricWaypoints(
  const Pose2D & start_pose,
  const Pose2D & p0_pose) const
{
  const Point2D start_point{start_pose.x, start_pose.y};
  const Point2D p0_point{p0_pose.x, p0_pose.y};
  const double chord_length = distanceBetween(start_point, p0_point);

  if (chord_length < parameters_.minimum_goal_distance) {
    throw std::runtime_error(
      "Current TF pose S0 is too close to P0 to generate a stable start trajectory. "
      "Move/relocalize the car farther from P0 or reduce minimum_goal_distance.");
  }

  const double tangent_length = parameters_.intermediate_tangent_scale * chord_length;
  const Point2D start_tangent{
    tangent_length * std::cos(start_pose.yaw),
    tangent_length * std::sin(start_pose.yaw)};
  const Point2D p0_tangent{
    tangent_length * std::cos(p0_pose.yaw),
    tangent_length * std::sin(p0_pose.yaw)};

  const auto sample_cubic_hermite = [&](double s) {
    const double s2 = s * s;
    const double s3 = s2 * s;
    const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
    const double h10 = s3 - 2.0 * s2 + s;
    const double h01 = -2.0 * s3 + 3.0 * s2;
    const double h11 = s3 - s2;
    return Point2D{
      h00 * start_point[0] + h10 * start_tangent[0] + h01 * p0_point[0] + h11 * p0_tangent[0],
      h00 * start_point[1] + h10 * start_tangent[1] + h01 * p0_point[1] + h11 * p0_tangent[1]};
  };

  constexpr int number_of_samples = 240;
  std::vector<Point2D> dense_points;
  std::vector<double> cumulative_lengths;
  dense_points.reserve(number_of_samples + 1);
  cumulative_lengths.reserve(number_of_samples + 1);

  for (int sample_index = 0; sample_index <= number_of_samples; ++sample_index) {
    const double s = static_cast<double>(sample_index) / static_cast<double>(number_of_samples);
    dense_points.push_back(sample_cubic_hermite(s));
    if (sample_index == 0) {
      cumulative_lengths.push_back(0.0);
    } else {
      cumulative_lengths.push_back(
        cumulative_lengths.back() + distanceBetween(dense_points[sample_index - 1], dense_points[sample_index]));
    }
  }

  const double total_length = cumulative_lengths.back();
  if (total_length < parameters_.minimum_goal_distance) {
    throw std::runtime_error("Generated start geometry S0 -> P0 is too short.");
  }

  const int last_waypoint_index = parameters_.number_of_waypoints_start - 1;
  std::vector<Point2D> waypoints;
  waypoints.reserve(static_cast<std::size_t>(parameters_.number_of_waypoints_start));

  for (int waypoint_index = 0; waypoint_index < parameters_.number_of_waypoints_start; ++waypoint_index) {
    const double target_length = total_length * static_cast<double>(waypoint_index) /
      static_cast<double>(last_waypoint_index);
    auto upper = std::lower_bound(cumulative_lengths.begin(), cumulative_lengths.end(), target_length);
    std::size_t index = static_cast<std::size_t>(std::distance(cumulative_lengths.begin(), upper));

    if (index == 0) {
      waypoints.push_back(start_point);
      continue;
    }
    if (index >= cumulative_lengths.size()) {
      waypoints.push_back(p0_point);
      continue;
    }

    const double previous_length = cumulative_lengths[index - 1];
    const double next_length = cumulative_lengths[index];
    const double ratio = (target_length - previous_length) / std::max(next_length - previous_length, kEpsilon);
    const Point2D previous_point = dense_points[index - 1];
    const Point2D next_point = dense_points[index];
    waypoints.push_back({
      previous_point[0] + ratio * (next_point[0] - previous_point[0]),
      previous_point[1] + ratio * (next_point[1] - previous_point[1])});
  }

  waypoints.front() = start_point;
  waypoints.back() = p0_point;
  return waypoints;
}

// Build physical boundary states and inputs for every full-trajectory waypoint.
std::vector<BoundaryWaypoint> QCar2BezierTrajectoryGenerator::buildBoundaryWaypoints(
  const std::vector<Point2D> & points,
  const std::vector<double> & segment_times,
  double start_yaw,
  std::size_t p0_index) const
{
  if (points.size() < 2) {
    throw std::runtime_error("The QCar2 FLMPC trajectory generator expects at least two waypoints.");
  }
  if (segment_times.size() != points.size() - 1) {
    throw std::runtime_error("Full segment time vector size must be exactly full_points.size() - 1.");
  }
  for (double segment_time : segment_times) {
    if (!isFinite(segment_time) || segment_time < parameters_.minimum_segment_time) {
      throw std::runtime_error("Each full segment time must be finite and >= minimum_segment_time.");
    }
  }

  const auto headings = computeWaypointHeadings(points, start_yaw, p0_index);
  const auto speeds = computeWaypointSpeeds(points, segment_times);
  const auto accelerations = computeWaypointAccelerations(speeds, segment_times);

  std::vector<BoundaryWaypoint> boundary_waypoints(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double curvature = computeDiscreteCurvature(points, i);
    boundary_waypoints[i].x = points[i][0];
    boundary_waypoints[i].y = points[i][1];
    boundary_waypoints[i].psi = headings[i];
    boundary_waypoints[i].vx = speeds[i];
    boundary_waypoints[i].delta = std::atan(parameters_.wheelbase * curvature);
    boundary_waypoints[i].ax = accelerations[i];
  }

  boundary_waypoints.front().vx = 0.0;
  boundary_waypoints.back().vx = 0.0;
  boundary_waypoints.front().ax = 0.0;
  boundary_waypoints.back().ax = 0.0;
  if (parameters_.zero_endpoint_steering) {
    boundary_waypoints.front().delta = 0.0;
    boundary_waypoints.back().delta = 0.0;
  }
  return boundary_waypoints;
}

// Compute continuous waypoint headings while enforcing theta_start and theta_end.
std::vector<double> QCar2BezierTrajectoryGenerator::computeWaypointHeadings(
  const std::vector<Point2D> & points,
  double start_yaw,
  std::size_t p0_index) const
{
  std::vector<double> headings(points.size(), 0.0);
  headings.front() = start_yaw;

  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    if (i == p0_index) {
      headings[i] = parameters_.theta_start;
      continue;
    }

    const Point2D previous_direction = unitVector({points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]});
    const Point2D next_direction = unitVector({points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1]});
    Point2D tangent{previous_direction[0] + next_direction[0], previous_direction[1] + next_direction[1]};

    if (std::hypot(tangent[0], tangent[1]) < kEpsilon) {
      if (std::hypot(next_direction[0], next_direction[1]) >= kEpsilon) {
        tangent = next_direction;
      } else if (std::hypot(previous_direction[0], previous_direction[1]) >= kEpsilon) {
        tangent = previous_direction;
      } else {
        tangent = {std::cos(headings[i - 1]), std::sin(headings[i - 1])};
      }
    }

    headings[i] = std::atan2(tangent[1], tangent[0]);
  }

  headings.back() = parameters_.theta_end;
  for (std::size_t i = 1; i < headings.size(); ++i) {
    headings[i] = unwrapToPrevious(headings[i], headings[i - 1]);
  }
  return headings;
}

// Compute longitudinal waypoint speeds from segment distances and times.
std::vector<double> QCar2BezierTrajectoryGenerator::computeWaypointSpeeds(
  const std::vector<Point2D> & points,
  const std::vector<double> & segment_times) const
{
  const std::size_t number_of_segments = points.size() - 1;
  std::vector<double> segment_speeds(number_of_segments, 0.0);
  for (std::size_t i = 0; i < number_of_segments; ++i) {
    segment_speeds[i] = distanceBetween(points[i], points[i + 1]) /
      std::max(segment_times[i], kEpsilon);
  }

  std::vector<double> waypoint_speeds(points.size(), 0.0);
  waypoint_speeds.front() = 0.0;
  waypoint_speeds.back() = 0.0;

  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    waypoint_speeds[i] = 0.5 * (segment_speeds[i - 1] + segment_speeds[i]);
  }

  return waypoint_speeds;
}

// Compute waypoint accelerations from neighboring speed transitions.
std::vector<double> QCar2BezierTrajectoryGenerator::computeWaypointAccelerations(
  const std::vector<double> & speeds,
  const std::vector<double> & segment_times) const
{
  std::vector<double> accelerations(speeds.size(), 0.0);
  if (speeds.size() < 2) {
    return accelerations;
  }

  std::vector<double> segment_accelerations(segment_times.size(), 0.0);
  for (std::size_t i = 0; i < segment_times.size(); ++i) {
    segment_accelerations[i] = (speeds[i + 1] - speeds[i]) / std::max(segment_times[i], kEpsilon);
  }

  accelerations.front() = segment_accelerations.front();
  accelerations.back() = segment_accelerations.back();
  for (std::size_t i = 1; i + 1 < speeds.size(); ++i) {
    accelerations[i] = 0.5 * (segment_accelerations[i - 1] + segment_accelerations[i]);
  }
  return accelerations;
}

// Estimate signed curvature from three neighboring geometric waypoints.
double QCar2BezierTrajectoryGenerator::computeDiscreteCurvature(
  const std::vector<Point2D> & points,
  std::size_t index) const
{
  if (index == 0 || index + 1 == points.size()) {
    if (parameters_.zero_endpoint_steering || points.size() < 3) {
      return 0.0;
    }
    index = index == 0 ? 1 : points.size() - 2;
  }

  const Point2D a_vector{points[index][0] - points[index - 1][0], points[index][1] - points[index - 1][1]};
  const Point2D b_vector{points[index + 1][0] - points[index][0], points[index + 1][1] - points[index][1]};
  const Point2D chord{points[index + 1][0] - points[index - 1][0], points[index + 1][1] - points[index - 1][1]};

  const double a = std::hypot(a_vector[0], a_vector[1]);
  const double b = std::hypot(b_vector[0], b_vector[1]);
  const double c = std::hypot(chord[0], chord[1]);
  if (std::min({a, b, c}) < kEpsilon) {
    return 0.0;
  }
  return 2.0 * cross2(a_vector, b_vector) / (a * b * c);
}

// Compute the six fifth-order Bezier control points for one segment.
QCar2BezierTrajectoryGenerator::ControlPolygon
QCar2BezierTrajectoryGenerator::computeBezier5ControlPoints(
  const BoundaryWaypoint & start,
  const BoundaryWaypoint & finish,
  double segment_time) const
{
  const double T = std::max(segment_time, kEpsilon);

  const Point2D start_heading{std::cos(start.psi), std::sin(start.psi)};
  const Point2D start_normal{-std::sin(start.psi), std::cos(start.psi)};
  const double start_psi_dot = (start.vx / parameters_.wheelbase) * std::tan(start.delta);
  const Point2D z0{start.x, start.y};
  const Point2D zd0{start.vx * start_heading[0], start.vx * start_heading[1]};
  const Point2D zdd0{
    start.ax * start_heading[0] + start.vx * start_psi_dot * start_normal[0],
    start.ax * start_heading[1] + start.vx * start_psi_dot * start_normal[1]};

  const Point2D finish_heading{std::cos(finish.psi), std::sin(finish.psi)};
  const Point2D finish_normal{-std::sin(finish.psi), std::cos(finish.psi)};
  const double finish_psi_dot = (finish.vx / parameters_.wheelbase) * std::tan(finish.delta);
  const Point2D zf{finish.x, finish.y};
  const Point2D zdf{finish.vx * finish_heading[0], finish.vx * finish_heading[1]};
  const Point2D zddf{
    finish.ax * finish_heading[0] + finish.vx * finish_psi_dot * finish_normal[0],
    finish.ax * finish_heading[1] + finish.vx * finish_psi_dot * finish_normal[1]};

  ControlPolygon control_points{};
  control_points[0] = z0;
  control_points[1] = {control_points[0][0] + T * zd0[0] / 5.0, control_points[0][1] + T * zd0[1] / 5.0};
  control_points[2] = {
    2.0 * control_points[1][0] - control_points[0][0] + T * T * zdd0[0] / 20.0,
    2.0 * control_points[1][1] - control_points[0][1] + T * T * zdd0[1] / 20.0};
  control_points[5] = zf;
  control_points[4] = {control_points[5][0] - T * zdf[0] / 5.0, control_points[5][1] - T * zdf[1] / 5.0};
  control_points[3] = {
    2.0 * control_points[4][0] - control_points[5][0] + T * T * zddf[0] / 20.0,
    2.0 * control_points[4][1] - control_points[5][1] + T * T * zddf[1] / 20.0};

  return control_points;
}

// Sample one Bezier segment and append stable physical reference values.
void QCar2BezierTrajectoryGenerator::appendBezierSegmentSamples(
  const ControlPolygon & control_points,
  const BoundaryWaypoint & start_boundary,
  const BoundaryWaypoint & finish_boundary,
  double segment_time,
  double time_offset,
  int segment_index,
  bool include_endpoint,
  ReferenceTrajectory & trajectory) const
{
  const double T = std::max(segment_time, kEpsilon);
  const int number_of_regular_samples = std::max(1, static_cast<int>(std::ceil(T / parameters_.sample_time)));

  const auto evaluate_sample = [&](double local_time) {
    const double s = std::min(std::max(local_time / T, 0.0), 1.0);

    Point2D z{0.0, 0.0};
    Point2D zd{0.0, 0.0};
    Point2D zdd{0.0, 0.0};

    std::array<Point2D, 5> d1{};
    std::array<Point2D, 4> d2{};
    for (int i = 0; i < 5; ++i) {
      d1[i] = {control_points[i + 1][0] - control_points[i][0], control_points[i + 1][1] - control_points[i][1]};
    }
    for (int i = 0; i < 4; ++i) {
      d2[i] = {
        control_points[i + 2][0] - 2.0 * control_points[i + 1][0] + control_points[i][0],
        control_points[i + 2][1] - 2.0 * control_points[i + 1][1] + control_points[i][1]};
    }

    for (int i = 0; i < 6; ++i) {
      const double basis = bernstein(5, i, s);
      z[0] += control_points[i][0] * basis;
      z[1] += control_points[i][1] * basis;
    }
    for (int i = 0; i < 5; ++i) {
      const double basis = bernstein(4, i, s);
      zd[0] += (5.0 / T) * d1[i][0] * basis;
      zd[1] += (5.0 / T) * d1[i][1] * basis;
    }
    for (int i = 0; i < 4; ++i) {
      const double basis = bernstein(3, i, s);
      zdd[0] += (20.0 / (T * T)) * d2[i][0] * basis;
      zdd[1] += (20.0 / (T * T)) * d2[i][1] * basis;
    }

    const double speed = std::hypot(zd[0], zd[1]);
    double curvature = 0.0;
    double longitudinal_acceleration = 0.0;
    bool use_boundary_kinematics = speed < kMinimumSpeedForCurvature;
    if (!use_boundary_kinematics) {
      const double safe_speed = std::max(speed, kMinimumSpeedForCurvature);
      const double curvature_numerator = zd[0] * zdd[1] - zd[1] * zdd[0];
      curvature = curvature_numerator / std::max(safe_speed * safe_speed * safe_speed, kEpsilon);
      longitudinal_acceleration = (zd[0] * zdd[0] + zd[1] * zdd[1]) / safe_speed;
    }

    ReferenceSample sample;
    sample.t = time_offset + local_time;
    sample.segment = segment_index;
    sample.x = z[0];
    sample.y = z[1];

    if (!use_boundary_kinematics) {
      sample.psi = std::atan2(zd[1], zd[0]);
    } else if (local_time <= 0.5 * T) {
      sample.psi = start_boundary.psi;
    } else {
      sample.psi = finish_boundary.psi;
    }

    sample.vx = speed;
    if (use_boundary_kinematics) {
      sample.delta = local_time <= 0.5 * T ? start_boundary.delta : finish_boundary.delta;
      sample.ax = local_time <= 0.5 * T ? start_boundary.ax : finish_boundary.ax;
    } else {
      sample.delta = std::atan(parameters_.wheelbase * curvature);
      sample.ax = longitudinal_acceleration;
    }

    if (!trajectory.samples.empty()) {
      sample.psi = unwrapToPrevious(sample.psi, trajectory.samples.back().psi);
    }
    trajectory.samples.push_back(sample);
  };

  for (int sample_index = 0; sample_index < number_of_regular_samples; ++sample_index) {
    const double local_time = sample_index * parameters_.sample_time;
    if (local_time < T - 0.5 * kEpsilon) {
      evaluate_sample(local_time);
    }
  }

  if (include_endpoint) {
    evaluate_sample(T);
  }
}

}  // namespace qcar2_flmpc
