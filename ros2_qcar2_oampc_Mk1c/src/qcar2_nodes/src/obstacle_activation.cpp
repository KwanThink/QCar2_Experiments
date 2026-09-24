#include "obstacle_activation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>

namespace qcar2_oampc
{
namespace
{
constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kGeometryEpsilon = 1.0e-9;

// Return a forward unit vector for one vehicle yaw angle.
Point2D forwardDirection(double vehicle_yaw)
{
  return Point2D{std::cos(vehicle_yaw), std::sin(vehicle_yaw)};
}

// Return the difference between two planar points as a vector.
Point2D subtractPoints(const Point2D & first, const Point2D & second)
{
  return Point2D{first.x - second.x, first.y - second.y};
}

}  // namespace

// Store the local obstacle activation parameters.
ObstacleActivationManager::ObstacleActivationManager(
  const ObstacleActivationParameters & parameters)
: parameters_(parameters)
{
  if (parameters_.activation_radius <= 0.0 || !std::isfinite(parameters_.activation_radius)) {
    throw std::runtime_error("activation_radius must be finite and positive.");
  }
  if (parameters_.field_of_view_degrees <= 0.0 ||
    !std::isfinite(parameters_.field_of_view_degrees))
  {
    throw std::runtime_error("activation_field_of_view must be finite and positive.");
  }
  parameters_.field_of_view_degrees = std::min(parameters_.field_of_view_degrees, 360.0);
  if (parameters_.maximum_active_obstacles <= 0) {
    throw std::runtime_error("max_active_obstacles must be positive.");
  }
  if (parameters_.safety_margin < 0.0 || !std::isfinite(parameters_.safety_margin)) {
    throw std::runtime_error("Obstacle activation safety margin must be finite and nonnegative.");
  }
}

// Clear all latched obstacle identifiers before a new run.
void ObstacleActivationManager::reset()
{
  active_obstacle_ids_.clear();
}

// Return whether a point lies inside a convex obstacle footprint.
bool ObstacleActivationManager::pointInsideObstacle(
  const Obstacle & obstacle,
  double point_x,
  double point_y)
{
  for (std::size_t edge_index = 0; edge_index < obstacle.edgeCount(); ++edge_index) {
    const auto & row = obstacle.halfspace_matrix[edge_index];
    const double value = row[0] * point_x + row[1] * point_y;
    if (value > obstacle.halfspace_vector[edge_index] + kGeometryEpsilon) {
      return false;
    }
  }
  return true;
}

// Return the closest point on one line segment.
Point2D ObstacleActivationManager::closestPointOnSegment(
  const Point2D & point,
  const Point2D & first_point,
  const Point2D & second_point)
{
  const double segment_x = second_point.x - first_point.x;
  const double segment_y = second_point.y - first_point.y;
  const double segment_length_squared = segment_x * segment_x + segment_y * segment_y;
  if (segment_length_squared <= kGeometryEpsilon) {
    return first_point;
  }

  const double point_x = point.x - first_point.x;
  const double point_y = point.y - first_point.y;
  double projection = (point_x * segment_x + point_y * segment_y) / segment_length_squared;
  projection = std::min(std::max(projection, 0.0), 1.0);
  return Point2D{
    first_point.x + projection * segment_x,
    first_point.y + projection * segment_y};
}

// Return the minimum distance from a point to an obstacle footprint.
double ObstacleActivationManager::obstacleFootprintDistance(
  const Obstacle & obstacle,
  double point_x,
  double point_y)
{
  if (pointInsideObstacle(obstacle, point_x, point_y)) {
    return 0.0;
  }

  const Point2D point{point_x, point_y};
  double minimum_distance = std::numeric_limits<double>::infinity();
  for (std::size_t vertex_index = 0; vertex_index < obstacle.vertices.size(); ++vertex_index) {
    const Point2D & first = obstacle.vertices[vertex_index];
    const Point2D & second = obstacle.vertices[(vertex_index + 1U) % obstacle.vertices.size()];
    const Point2D closest = closestPointOnSegment(point, first, second);
    minimum_distance = std::min(
      minimum_distance,
      std::hypot(closest.x - point.x, closest.y - point.y));
  }
  return minimum_distance;
}

// Return whether a point lies inside the local activation sector.
bool ObstacleActivationManager::pointInsideActivationRegion(
  const Point2D & point,
  const Point2D & vehicle_position,
  double vehicle_yaw,
  double activation_radius,
  double field_of_view_degrees)
{
  const double relative_x = point.x - vehicle_position.x;
  const double relative_y = point.y - vehicle_position.y;
  const double distance = std::hypot(relative_x, relative_y);
  if (distance > activation_radius + kGeometryEpsilon) {
    return false;
  }
  if (distance <= kGeometryEpsilon || field_of_view_degrees >= 360.0 - kGeometryEpsilon) {
    return true;
  }

  const double longitudinal_position =
    relative_x * std::cos(vehicle_yaw) + relative_y * std::sin(vehicle_yaw);
  const double lateral_position =
    -relative_x * std::sin(vehicle_yaw) + relative_y * std::cos(vehicle_yaw);
  const double relative_angle = std::atan2(lateral_position, longitudinal_position);
  const double half_field_of_view = 0.5 * field_of_view_degrees * kPi / 180.0;
  return std::abs(relative_angle) <= half_field_of_view + kGeometryEpsilon;
}

// Return the signed two-dimensional cross product.
double ObstacleActivationManager::cross2D(
  const Point2D & first_vector,
  const Point2D & second_vector)
{
  return first_vector.x * second_vector.y - first_vector.y * second_vector.x;
}

// Return whether a point lies on one closed line segment.
bool ObstacleActivationManager::pointOnSegment(
  const Point2D & point,
  const Point2D & first_point,
  const Point2D & second_point)
{
  const Point2D segment = subtractPoints(second_point, first_point);
  const Point2D point_vector = subtractPoints(point, first_point);
  if (std::abs(cross2D(segment, point_vector)) > kGeometryEpsilon) {
    return false;
  }

  return point.x >= std::min(first_point.x, second_point.x) - kGeometryEpsilon &&
         point.x <= std::max(first_point.x, second_point.x) + kGeometryEpsilon &&
         point.y >= std::min(first_point.y, second_point.y) - kGeometryEpsilon &&
         point.y <= std::max(first_point.y, second_point.y) + kGeometryEpsilon;
}

// Return whether two closed line segments intersect.
bool ObstacleActivationManager::segmentsIntersect(
  const Point2D & first_start,
  const Point2D & first_end,
  const Point2D & second_start,
  const Point2D & second_end)
{
  const Point2D first_direction = subtractPoints(first_end, first_start);
  const Point2D second_direction = subtractPoints(second_end, second_start);
  const double first_side_start = cross2D(first_direction, subtractPoints(second_start, first_start));
  const double first_side_end = cross2D(first_direction, subtractPoints(second_end, first_start));
  const double second_side_start = cross2D(second_direction, subtractPoints(first_start, second_start));
  const double second_side_end = cross2D(second_direction, subtractPoints(first_end, second_start));

  if (first_side_start * first_side_end < -kGeometryEpsilon &&
    second_side_start * second_side_end < -kGeometryEpsilon)
  {
    return true;
  }
  if (std::abs(first_side_start) <= kGeometryEpsilon &&
    pointOnSegment(second_start, first_start, first_end))
  {
    return true;
  }
  if (std::abs(first_side_end) <= kGeometryEpsilon &&
    pointOnSegment(second_end, first_start, first_end))
  {
    return true;
  }
  if (std::abs(second_side_start) <= kGeometryEpsilon &&
    pointOnSegment(first_start, second_start, second_end))
  {
    return true;
  }
  if (std::abs(second_side_end) <= kGeometryEpsilon &&
    pointOnSegment(first_end, second_start, second_end))
  {
    return true;
  }
  return false;
}

// Return all intersections between a segment and a circle.
std::vector<Point2D> ObstacleActivationManager::segmentCircleIntersections(
  const Point2D & first_point,
  const Point2D & second_point,
  const Point2D & circle_center,
  double circle_radius)
{
  const double segment_x = second_point.x - first_point.x;
  const double segment_y = second_point.y - first_point.y;
  const double relative_x = first_point.x - circle_center.x;
  const double relative_y = first_point.y - circle_center.y;
  const double quadratic_a = segment_x * segment_x + segment_y * segment_y;
  if (quadratic_a <= kGeometryEpsilon) {
    return {};
  }

  const double quadratic_b = 2.0 * (relative_x * segment_x + relative_y * segment_y);
  const double quadratic_c = relative_x * relative_x + relative_y * relative_y -
    circle_radius * circle_radius;
  double discriminant = quadratic_b * quadratic_b - 4.0 * quadratic_a * quadratic_c;
  if (discriminant < -kGeometryEpsilon) {
    return {};
  }
  discriminant = std::max(discriminant, 0.0);

  const double square_root = std::sqrt(discriminant);
  const double denominator = 2.0 * quadratic_a;
  const std::array<double, 2> parameters{{
    (-quadratic_b - square_root) / denominator,
    (-quadratic_b + square_root) / denominator}};

  std::vector<Point2D> intersections;
  for (double parameter : parameters) {
    if (parameter < -kGeometryEpsilon || parameter > 1.0 + kGeometryEpsilon) {
      continue;
    }
    parameter = std::min(std::max(parameter, 0.0), 1.0);
    const Point2D intersection{
      first_point.x + parameter * segment_x,
      first_point.y + parameter * segment_y};
    if (intersections.empty() ||
      std::hypot(
        intersection.x - intersections.back().x,
        intersection.y - intersections.back().y) > kGeometryEpsilon)
    {
      intersections.push_back(intersection);
    }
  }
  return intersections;
}

// Return whether any part of an obstacle intersects the activation sector.
bool ObstacleActivationManager::obstacleIntersectsActivationRegion(
  const Obstacle & obstacle,
  const Point2D & vehicle_position,
  double vehicle_yaw,
  double activation_radius,
  double field_of_view_degrees)
{
  if (obstacleFootprintDistance(obstacle, vehicle_position.x, vehicle_position.y) >
    activation_radius + kGeometryEpsilon)
  {
    return false;
  }
  if (field_of_view_degrees >= 360.0 - kGeometryEpsilon) {
    return true;
  }
  if (pointInsideObstacle(obstacle, vehicle_position.x, vehicle_position.y)) {
    return true;
  }

  for (const Point2D & vertex : obstacle.vertices) {
    if (pointInsideActivationRegion(
        vertex,
        vehicle_position,
        vehicle_yaw,
        activation_radius,
        field_of_view_degrees))
    {
      return true;
    }
  }

  const double half_field_of_view = 0.5 * field_of_view_degrees * kPi / 180.0;
  const double lower_angle = vehicle_yaw - half_field_of_view;
  const double upper_angle = vehicle_yaw + half_field_of_view;
  const Point2D lower_boundary_end{
    vehicle_position.x + activation_radius * std::cos(lower_angle),
    vehicle_position.y + activation_radius * std::sin(lower_angle)};
  const Point2D upper_boundary_end{
    vehicle_position.x + activation_radius * std::cos(upper_angle),
    vehicle_position.y + activation_radius * std::sin(upper_angle)};

  for (std::size_t vertex_index = 0; vertex_index < obstacle.vertices.size(); ++vertex_index) {
    const Point2D & first = obstacle.vertices[vertex_index];
    const Point2D & second = obstacle.vertices[(vertex_index + 1U) % obstacle.vertices.size()];
    if (segmentsIntersect(first, second, vehicle_position, lower_boundary_end) ||
      segmentsIntersect(first, second, vehicle_position, upper_boundary_end))
    {
      return true;
    }

    const std::vector<Point2D> circle_intersections = segmentCircleIntersections(
      first,
      second,
      vehicle_position,
      activation_radius);
    for (const Point2D & intersection : circle_intersections) {
      if (pointInsideActivationRegion(
          intersection,
          vehicle_position,
          vehicle_yaw,
          activation_radius,
          field_of_view_degrees))
      {
        return true;
      }
    }
  }
  return false;
}

// Return whether the complete obstacle footprint is behind the vehicle by gamma.
bool ObstacleActivationManager::obstacleHasBeenPassed(
  const Obstacle & obstacle,
  const Point2D & vehicle_position,
  double vehicle_yaw,
  double safety_margin)
{
  if (obstacle.vertices.empty()) {
    return false;
  }

  const Point2D forward = forwardDirection(vehicle_yaw);
  double furthest_forward_position = -std::numeric_limits<double>::infinity();
  for (const Point2D & vertex : obstacle.vertices) {
    const double relative_x = vertex.x - vehicle_position.x;
    const double relative_y = vertex.y - vehicle_position.y;
    const double longitudinal_position = relative_x * forward.x + relative_y * forward.y;
    furthest_forward_position = std::max(furthest_forward_position, longitudinal_position);
  }
  return furthest_forward_position < -safety_margin;
}

// Update and return the currently latched active obstacles.
std::vector<const Obstacle *> ObstacleActivationManager::update(
  const std::vector<Obstacle> & obstacles,
  double vehicle_x,
  double vehicle_y,
  double vehicle_yaw)
{
  if (obstacles.empty()) {
    active_obstacle_ids_.clear();
    return {};
  }

  const Point2D vehicle_position{vehicle_x, vehicle_y};
  std::vector<int> ids_to_deactivate;
  for (int obstacle_id : active_obstacle_ids_) {
    const auto iterator = std::find_if(
      obstacles.begin(),
      obstacles.end(),
      [obstacle_id](const Obstacle & obstacle) {return obstacle.id == obstacle_id;});
    if (iterator == obstacles.end() ||
      obstacleHasBeenPassed(*iterator, vehicle_position, vehicle_yaw, parameters_.safety_margin))
    {
      ids_to_deactivate.push_back(obstacle_id);
    }
  }
  for (int obstacle_id : ids_to_deactivate) {
    active_obstacle_ids_.erase(obstacle_id);
  }

  struct Candidate
  {
    double distance = 0.0;
    int obstacle_id = -1;
  };
  std::vector<Candidate> candidates;
  for (const Obstacle & obstacle : obstacles) {
    if (active_obstacle_ids_.count(obstacle.id) != 0U) {
      continue;
    }
    const double distance = obstacleFootprintDistance(obstacle, vehicle_x, vehicle_y);
    if (distance > parameters_.activation_radius + kGeometryEpsilon) {
      continue;
    }
    if (!obstacleIntersectsActivationRegion(
        obstacle,
        vehicle_position,
        vehicle_yaw,
        parameters_.activation_radius,
        parameters_.field_of_view_degrees))
    {
      continue;
    }
    candidates.push_back(Candidate{distance, obstacle.id});
  }

  std::sort(
    candidates.begin(),
    candidates.end(),
    [](const Candidate & first, const Candidate & second) {
      return std::tie(first.distance, first.obstacle_id) <
             std::tie(second.distance, second.obstacle_id);
    });

  const int available_slots = parameters_.maximum_active_obstacles -
    static_cast<int>(active_obstacle_ids_.size());
  for (int index = 0; index < available_slots && index < static_cast<int>(candidates.size()); ++index) {
    active_obstacle_ids_.insert(candidates[static_cast<std::size_t>(index)].obstacle_id);
  }

  std::vector<const Obstacle *> active_obstacles;
  for (const Obstacle & obstacle : obstacles) {
    if (active_obstacle_ids_.count(obstacle.id) != 0U) {
      active_obstacles.push_back(&obstacle);
    }
  }
  std::sort(
    active_obstacles.begin(),
    active_obstacles.end(),
    [vehicle_x, vehicle_y](const Obstacle * first, const Obstacle * second) {
      const double first_distance = obstacleFootprintDistance(*first, vehicle_x, vehicle_y);
      const double second_distance = obstacleFootprintDistance(*second, vehicle_x, vehicle_y);
      if (std::abs(first_distance - second_distance) <= kGeometryEpsilon) {
        return first->id < second->id;
      }
      return first_distance < second_distance;
    });
  return active_obstacles;
}

}  // namespace qcar2_oampc
