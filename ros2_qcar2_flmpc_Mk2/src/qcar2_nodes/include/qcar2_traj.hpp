#ifndef QCAR2_TRAJ_HPP_
#define QCAR2_TRAJ_HPP_

#include <array>
#include <stdexcept>
#include <string>
#include <vector>

namespace qcar2_flmpc
{

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct TrajectoryParameters
{
  double wheelbase = 0.25725;
  double sample_time = 0.03;

  // Start geometry S0 -> P0 is generated automatically with cubic Hermite,
  // then the fixed reference waypoints P0 -> ... -> PN are appended before
  // one global quintic-Bezier trajectory is sampled.
  int number_of_waypoints_start = 4;
  std::vector<double> segment_times_start = {4.0, 4.0, 4.0};
  std::vector<double> waypoints_ref_xy = {0.0, 0.0, 1.0, 0.0, 1.5, 0.8, 2.0, 1.2};
  std::vector<double> segment_times_ref = {6.0, 6.0, 6.0};
  double theta_start = 0.0;
  double theta_end = 0.8;

  double minimum_segment_time = 1.0;
  double intermediate_tangent_scale = 1.0;
  double minimum_goal_distance = 0.05;
  bool zero_endpoint_steering = false;
};

struct BoundaryWaypoint
{
  double x = 0.0;
  double y = 0.0;
  double psi = 0.0;
  double vx = 0.0;
  double delta = 0.0;
  double ax = 0.0;
};

struct ReferenceSample
{
  double t = 0.0;
  int segment = 0;
  double x = 0.0;
  double y = 0.0;
  double psi = 0.0;
  double vx = 0.0;
  double delta = 0.0;
  double ax = 0.0;
};

struct ReferenceTrajectory
{
  std::vector<ReferenceSample> samples;
  std::vector<BoundaryWaypoint> boundary_waypoints;
  std::vector<double> segment_times;

  // Return whether the reference trajectory contains no samples.
  bool empty() const { return samples.empty(); }

  // Return the final trajectory sample time.
  double duration() const { return samples.empty() ? 0.0 : samples.back().t; }
};

class QCar2BezierTrajectoryGenerator
{
public:
  using Point2D = std::array<double, 2>;
  using ControlPolygon = std::array<Point2D, 6>;

  explicit QCar2BezierTrajectoryGenerator(const TrajectoryParameters & parameters);

  // Create the active trajectory tracked by FLMPC:
  // S0 -> auto Hermite start waypoints -> P0 -> P1 -> ... -> PN.
  // The RViz 2D Goal is only a trigger; start_pose comes from TF map -> base_link.
  ReferenceTrajectory generateFullReferenceTrajectory(const Pose2D & start_pose) const;

  // Return the fixed map waypoints P0 -> P1 -> ... -> PN for RViz preview.
  std::vector<Point2D> referenceWaypoints() const;

private:
  TrajectoryParameters parameters_;

  void validateFixedReferenceConfig() const;
  std::vector<Point2D> parseReferenceWaypoints() const;
  std::vector<Point2D> buildStartGeometricWaypoints(
    const Pose2D & start_pose,
    const Pose2D & p0_pose) const;
  std::vector<BoundaryWaypoint> buildBoundaryWaypoints(
    const std::vector<Point2D> & points,
    const std::vector<double> & segment_times,
    double start_yaw,
    std::size_t p0_index) const;
  std::vector<double> computeWaypointHeadings(
    const std::vector<Point2D> & points,
    double start_yaw,
    std::size_t p0_index) const;
  std::vector<double> computeWaypointSpeeds(
    const std::vector<Point2D> & points,
    const std::vector<double> & segment_times) const;
  std::vector<double> computeWaypointAccelerations(
    const std::vector<double> & speeds,
    const std::vector<double> & segment_times) const;
  double computeDiscreteCurvature(const std::vector<Point2D> & points, std::size_t index) const;
  ControlPolygon computeBezier5ControlPoints(
    const BoundaryWaypoint & start,
    const BoundaryWaypoint & finish,
    double segment_time) const;
  void appendBezierSegmentSamples(
    const ControlPolygon & control_points,
    const BoundaryWaypoint & start_boundary,
    const BoundaryWaypoint & finish_boundary,
    double segment_time,
    double time_offset,
    int segment_index,
    bool include_endpoint,
    ReferenceTrajectory & trajectory) const;
};

}  // namespace qcar2_flmpc

#endif  // QCAR2_TRAJ_HPP_
