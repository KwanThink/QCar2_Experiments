#ifndef OBSTACLE_ACTIVATION_HPP_
#define OBSTACLE_ACTIVATION_HPP_

#include "obstacle_map.hpp"

#include <set>
#include <vector>

namespace qcar2_oampc
{

struct ObstacleActivationParameters
{
  double activation_radius = 2.0;
  double field_of_view_degrees = 180.0;
  int maximum_active_obstacles = 4;
  double safety_margin = 0.35;
};

class ObstacleActivationManager
{
public:
  // Store the local obstacle activation parameters.
  explicit ObstacleActivationManager(const ObstacleActivationParameters & parameters);

  // Clear all latched obstacle identifiers before a new run.
  void reset();

  // Update and return the currently latched active obstacles.
  std::vector<const Obstacle *> update(
    const std::vector<Obstacle> & obstacles,
    double vehicle_x,
    double vehicle_y,
    double vehicle_yaw);

  // Return the minimum distance from a point to an obstacle footprint.
  static double obstacleFootprintDistance(
    const Obstacle & obstacle,
    double point_x,
    double point_y);

private:
  // Return whether a point lies inside a convex obstacle footprint.
  static bool pointInsideObstacle(
    const Obstacle & obstacle,
    double point_x,
    double point_y);

  // Return the closest point on one line segment.
  static Point2D closestPointOnSegment(
    const Point2D & point,
    const Point2D & first_point,
    const Point2D & second_point);

  // Return whether a point lies inside the local activation sector.
  static bool pointInsideActivationRegion(
    const Point2D & point,
    const Point2D & vehicle_position,
    double vehicle_yaw,
    double activation_radius,
    double field_of_view_degrees);

  // Return the signed two-dimensional cross product.
  static double cross2D(const Point2D & first_vector, const Point2D & second_vector);

  // Return whether a point lies on one closed line segment.
  static bool pointOnSegment(
    const Point2D & point,
    const Point2D & first_point,
    const Point2D & second_point);

  // Return whether two closed line segments intersect.
  static bool segmentsIntersect(
    const Point2D & first_start,
    const Point2D & first_end,
    const Point2D & second_start,
    const Point2D & second_end);

  // Return all intersections between a segment and a circle.
  static std::vector<Point2D> segmentCircleIntersections(
    const Point2D & first_point,
    const Point2D & second_point,
    const Point2D & circle_center,
    double circle_radius);

  // Return whether any part of an obstacle intersects the activation sector.
  static bool obstacleIntersectsActivationRegion(
    const Obstacle & obstacle,
    const Point2D & vehicle_position,
    double vehicle_yaw,
    double activation_radius,
    double field_of_view_degrees);

  // Return whether the complete obstacle footprint is behind the vehicle by gamma.
  static bool obstacleHasBeenPassed(
    const Obstacle & obstacle,
    const Point2D & vehicle_position,
    double vehicle_yaw,
    double safety_margin);

  ObstacleActivationParameters parameters_;
  std::set<int> active_obstacle_ids_;
};

}  // namespace qcar2_oampc

#endif  // OBSTACLE_ACTIVATION_HPP_
