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
  double sample_time = 0.02;
  std::string trajectory_generation_mode = "auto";
  int number_of_waypoints = 4;
  std::vector<double> segment_times = {3.0, 3.0, 3.0};
  double minimum_segment_time = 1.0;
  double intermediate_tangent_scale = 0.55;
  double minimum_goal_distance = 0.05;
  bool zero_endpoint_steering = true;
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

  // Create the complete flatness-based Bezier reference from current pose to goal pose.
  // The output samples contain state [X,Y,psi,vx] and input [delta,ax].
  ReferenceTrajectory generateTrajectory(const Pose2D & start_pose, const Pose2D & goal_pose) const;

private:
  TrajectoryParameters parameters_;

  std::vector<Point2D> buildAutoGeometricWaypoints(
    const Pose2D & start_pose,
    const Pose2D & goal_pose,
    double direction_sign) const;
  std::vector<BoundaryWaypoint> buildBoundaryWaypoints(
    const std::vector<Point2D> & points,
    double start_yaw,
    double goal_yaw,
    std::vector<double> & segment_times,
    double direction_sign) const;
  std::vector<double> computeWaypointHeadings(
    const std::vector<Point2D> & points,
    double start_yaw,
    double goal_yaw,
    double direction_sign) const;
  std::vector<double> computeWaypointSpeeds(
    const std::vector<Point2D> & points,
    const std::vector<double> & segment_times,
    double direction_sign) const;
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
    double segment_time,
    double time_offset,
    int segment_index,
    bool include_endpoint,
    double direction_sign,
    ReferenceTrajectory & trajectory) const;
};

}  // namespace qcar2_flmpc

#endif  // QCAR2_TRAJ_HPP_
