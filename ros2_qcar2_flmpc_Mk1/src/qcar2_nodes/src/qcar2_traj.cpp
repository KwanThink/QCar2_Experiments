#include "qcar2_traj.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace qcar2_flmpc
{
namespace
{
constexpr double kEpsilon = 1.0e-9;
constexpr double kPi = 3.1415926535897932384626433832795;

constexpr double kReverseAngleThresholdDeg = 100.0;
constexpr double kReverseAngleThresholdRad = kReverseAngleThresholdDeg * kPi / 180.0;

// Return the Euclidean distance between two planar points.
double distanceBetween(const QCar2BezierTrajectoryGenerator::Point2D & a, const QCar2BezierTrajectoryGenerator::Point2D & b)
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

// Wrap one angle into the interval [-pi, pi].
double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

// Select forward or reverse motion from the goal direction.
double selectAutoDirectionSign(
  const qcar2_flmpc::Pose2D & start_pose,
  const qcar2_flmpc::Pose2D & goal_pose)
{
  const double dx = goal_pose.x - start_pose.x;
  const double dy = goal_pose.y - start_pose.y;

  const double distance = std::hypot(dx, dy);
  if (distance < kEpsilon) {
    return 1.0;
  }

  const double goal_angle = std::atan2(dy, dx);
  const double relative_angle = normalizeAngle(goal_angle - start_pose.yaw);

  if (std::abs(relative_angle) > kReverseAngleThresholdRad) {
    return -1.0;
  }

  return 1.0;
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
// QCar2BezierTrajectoryGenerator constructor to get parameters from config
QCar2BezierTrajectoryGenerator::QCar2BezierTrajectoryGenerator(const TrajectoryParameters & parameters)
: parameters_(parameters)
{
}

// Generate the complete online Bezier reference trajectory.
ReferenceTrajectory QCar2BezierTrajectoryGenerator::generateTrajectory(
  const Pose2D & start_pose,
  const Pose2D & goal_pose) const
{
  const double direction_sign = selectAutoDirectionSign(start_pose, goal_pose);

  const auto geometric_waypoints = buildAutoGeometricWaypoints(start_pose, goal_pose, direction_sign);

  ReferenceTrajectory trajectory;
  trajectory.boundary_waypoints = buildBoundaryWaypoints(geometric_waypoints, start_pose.yaw, goal_pose.yaw, trajectory.segment_times, direction_sign);

  double time_offset = 0.0;
  for (std::size_t segment_index = 0; segment_index < trajectory.segment_times.size(); ++segment_index) {
    const auto control_points = computeBezier5ControlPoints(
      trajectory.boundary_waypoints[segment_index],
      trajectory.boundary_waypoints[segment_index + 1],
      trajectory.segment_times[segment_index]);

    const bool include_endpoint = segment_index == trajectory.segment_times.size() - 1;
    appendBezierSegmentSamples(
      control_points,
      trajectory.segment_times[segment_index],
      time_offset,
      static_cast<int>(segment_index),
      include_endpoint,
      direction_sign,
      trajectory);

    time_offset += trajectory.segment_times[segment_index];
  }

  // Keep the complete reference heading continuous and on the same angular
  // branch as the measured start yaw. This avoids false +pi/-pi jumps when
  // tracking reverse trajectories.
  if (!trajectory.samples.empty()) {
    trajectory.samples.front().psi = start_pose.yaw;
    trajectory.samples.front().vx = 0.0;

    for (std::size_t i = 1; i < trajectory.samples.size(); ++i) {
      trajectory.samples[i].psi = unwrapToPrevious(
        trajectory.samples[i].psi,
        trajectory.samples[i - 1].psi);
    }

    if (trajectory.samples.size() >= 2) {
      trajectory.samples.back().psi = unwrapToPrevious(
        goal_pose.yaw,
        trajectory.samples[trajectory.samples.size() - 2].psi);
    }

    trajectory.samples.back().vx = 0.0;
  }
  
  return trajectory;
}

// Using Hermite Curves to compute
std::vector<QCar2BezierTrajectoryGenerator::Point2D>
QCar2BezierTrajectoryGenerator::buildAutoGeometricWaypoints(
  const Pose2D & start_pose,
  const Pose2D & goal_pose,
  double direction_sign) const
{
  if (parameters_.trajectory_generation_mode != "auto") {
    throw std::runtime_error("Only trajectory_generation_mode='auto' is supported in this ROS2 controller.");
  }

  if (parameters_.number_of_waypoints < 2) {
    throw std::runtime_error("number_of_waypoints must be at least 2.");
  }

  const Point2D start_point{start_pose.x, start_pose.y};
  const Point2D goal_point{goal_pose.x, goal_pose.y};
  const double chord_length = distanceBetween(start_point, goal_point);

  if (chord_length < parameters_.minimum_goal_distance) {
    throw std::runtime_error("Goal is too close to the current pose to generate a stable trajectory.");
  }

  const double tangent_length = parameters_.intermediate_tangent_scale * chord_length;
  const Point2D start_tangent{
    direction_sign * tangent_length * std::cos(start_pose.yaw),
    direction_sign * tangent_length * std::sin(start_pose.yaw)};
  const Point2D goal_tangent{
    direction_sign * tangent_length * std::cos(goal_pose.yaw),
    direction_sign * tangent_length * std::sin(goal_pose.yaw)};

  const auto sample_cubic_hermite = [&](double s) {
    const double s2 = s * s;
    const double s3 = s2 * s;
    const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
    const double h10 = s3 - 2.0 * s2 + s;
    const double h01 = -2.0 * s3 + 3.0 * s2;
    const double h11 = s3 - s2;
    return Point2D{
      h00 * start_point[0] + h10 * start_tangent[0] + h01 * goal_point[0] + h11 * goal_tangent[0],
      h00 * start_point[1] + h10 * start_tangent[1] + h01 * goal_point[1] + h11 * goal_tangent[1]};
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
    throw std::runtime_error("Generated geometric path is too short.");
  }

  const int last_waypoint_index = parameters_.number_of_waypoints - 1;
  std::vector<Point2D> waypoints;
  waypoints.reserve(static_cast<std::size_t>(parameters_.number_of_waypoints));

  for (int waypoint_index = 0; waypoint_index < parameters_.number_of_waypoints; ++waypoint_index) {
    const double target_length = total_length * static_cast<double>(waypoint_index) /
      static_cast<double>(last_waypoint_index);
    auto upper = std::lower_bound(cumulative_lengths.begin(), cumulative_lengths.end(), target_length);
    std::size_t index = static_cast<std::size_t>(std::distance(cumulative_lengths.begin(), upper));

    if (index == 0) {
      waypoints.push_back(start_point);
      continue;
    }
    if (index >= cumulative_lengths.size()) {
      waypoints.push_back(goal_point);
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
  waypoints.back() = goal_point;
  return waypoints;
}

// Build physical waypoint boundary values for every Bezier segment.
std::vector<BoundaryWaypoint> QCar2BezierTrajectoryGenerator::buildBoundaryWaypoints(
  const std::vector<Point2D> & points,
  double start_yaw,
  double goal_yaw,
  std::vector<double> & segment_times,
  double direction_sign) const
{
  if (points.size() < 2) {
    throw std::runtime_error("The QCar2 FLMPC trajectory generator expects at least two waypoints.");
  }

  if (parameters_.segment_times.empty()) {
    throw std::runtime_error("segment_times must be defined because automatic default speed is not used.");
  }

  if (parameters_.segment_times.size() != points.size() - 1) {
    throw std::runtime_error("segment_times size must be exactly number_of_waypoints - 1.");
  }
  segment_times = parameters_.segment_times;

  for (double segment_time : segment_times) {
    if (!std::isfinite(segment_time) || segment_time < parameters_.minimum_segment_time) {
      throw std::runtime_error("Each segment time must be finite and >= minimum_segment_time.");
    }
  }
  const auto headings = computeWaypointHeadings(points, start_yaw, goal_yaw, direction_sign);
  const auto speeds = computeWaypointSpeeds(points, segment_times, direction_sign);
  const auto accelerations = computeWaypointAccelerations(speeds, segment_times);

  std::vector<BoundaryWaypoint> boundary_waypoints(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double curvature = computeDiscreteCurvature(points, i);
    boundary_waypoints[i].x = points[i][0];
    boundary_waypoints[i].y = points[i][1];
    boundary_waypoints[i].psi = headings[i];
    boundary_waypoints[i].vx = speeds[i];
    boundary_waypoints[i].delta = std::atan(direction_sign * parameters_.wheelbase * curvature);
    boundary_waypoints[i].ax = accelerations[i];
  }
  return boundary_waypoints;
}

// Compute continuous vehicle headings at all trajectory waypoints.
std::vector<double> QCar2BezierTrajectoryGenerator::computeWaypointHeadings(
  const std::vector<Point2D> & points,
  double start_yaw,
  double goal_yaw,
  double direction_sign) const
{
  std::vector<double> headings(points.size(), 0.0);
  headings.front() = start_yaw;

  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const Point2D previous_direction = unitVector({points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]});
    const Point2D next_direction = unitVector({points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1]});
    Point2D tangent{previous_direction[0] + next_direction[0], previous_direction[1] + next_direction[1]};
    if (std::hypot(tangent[0], tangent[1]) < kEpsilon) {
      tangent = {points[i + 1][0] - points[i - 1][0], points[i + 1][1] - points[i - 1][1]};
    }
    
    double heading = std::atan2(tangent[1], tangent[0]);
    if (direction_sign < 0.0) {
      heading += kPi;
    }
    headings[i] = heading;
  }

  headings.back() = goal_yaw;
  for (std::size_t i = 1; i < headings.size(); ++i) {
    headings[i] = unwrapToPrevious(headings[i], headings[i - 1]);
  }
  return headings;
}

// Compute signed longitudinal speeds at all trajectory waypoints.
std::vector<double> QCar2BezierTrajectoryGenerator::computeWaypointSpeeds(
  const std::vector<Point2D> & points,
  const std::vector<double> & segment_times, 
  double direction_sign) const
{
  const std::size_t number_of_segments = points.size() - 1;
  std::vector<double> segment_speeds(number_of_segments, 0.0);
  for (std::size_t i = 0; i < number_of_segments; ++i) {
    segment_speeds[i] = direction_sign * distanceBetween(points[i], points[i + 1]) /
      std::max(segment_times[i], kEpsilon);
  }

  std::vector<double> waypoint_speeds(points.size(), 0.0);

  // Start and goal are rest boundary conditions.
  waypoint_speeds.front() = 0.0;
  waypoint_speeds.back() = 0.0;

  // Internal waypoints keep the average segment speed.
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    waypoint_speeds[i] = 0.5 * (segment_speeds[i - 1] + segment_speeds[i]);
  }

  return waypoint_speeds;
}

// Compute longitudinal accelerations at all trajectory waypoints.
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

// Estimate signed path curvature from three neighboring waypoints.
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

// Compute the six control points of one fifth-order Bezier segment.
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

// Sample one Bezier segment and append physical reference values.
void QCar2BezierTrajectoryGenerator::appendBezierSegmentSamples(
  const ControlPolygon & control_points,
  double segment_time,
  double time_offset,
  int segment_index,
  bool include_endpoint,
  double direction_sign,
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
    const double safe_speed = std::max(speed, kEpsilon);
    const double curvature_numerator = zd[0] * zdd[1] - zd[1] * zdd[0];
    const double curvature = curvature_numerator / std::max(safe_speed * safe_speed * safe_speed, kEpsilon);
    const double longitudinal_acceleration = direction_sign * (zd[0] * zdd[0] + zd[1] * zdd[1]) / safe_speed;

    ReferenceSample sample;
    sample.t = time_offset + local_time;
    sample.segment = segment_index;
    sample.x = z[0];
    sample.y = z[1];

    // Don't use atan2(0,0) when speed = 0, set psi = 0.0 temporarily 
    if (speed > kEpsilon) {
      sample.psi = std::atan2(zd[1], zd[0]);
      if (direction_sign < 0.0) {
        sample.psi += kPi;
      }
    } else if (!trajectory.samples.empty()) {
      sample.psi = trajectory.samples.back().psi;
    } else {
      sample.psi = 0.0;
    }

    sample.vx = direction_sign * speed;
    sample.delta = std::atan(direction_sign * parameters_.wheelbase * curvature);
    sample.ax = longitudinal_acceleration;

    // Set psi by the real yaw value
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
