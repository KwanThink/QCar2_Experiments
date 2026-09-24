#ifndef OBSTACLE_MAP_HPP_
#define OBSTACLE_MAP_HPP_

#include <array>
#include <string>
#include <vector>

namespace qcar2_oampc
{

struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

struct Workspace
{
  double x_min = 0.0;
  double x_max = 0.0;
  double y_min = 0.0;
  double y_max = 0.0;
};

struct Obstacle
{
  int id = -1;
  std::vector<Point2D> vertices;
  std::vector<std::array<double, 2>> halfspace_matrix;
  std::vector<double> halfspace_vector;

  // Return the number of polygon edges represented by this obstacle.
  std::size_t edgeCount() const { return halfspace_vector.size(); }
};

class ObstacleMap
{
public:
  // Load and validate workspace and obstacle geometry from YAML.
  void load(const std::string & file_path);

  // Return the loaded rectangular workspace.
  const Workspace & workspace() const { return workspace_; }

  // Return all loaded convex obstacle polygons.
  const std::vector<Obstacle> & obstacles() const { return obstacles_; }

  // Return the largest edge count among all loaded obstacles.
  std::size_t maximumEdgeCount() const { return maximum_edge_count_; }

  // Return the source geometry YAML path.
  const std::string & sourceFile() const { return source_file_; }

  // Find one obstacle by its persistent identifier.
  const Obstacle * findObstacleById(int obstacle_id) const;

private:
  Workspace workspace_;
  std::vector<Obstacle> obstacles_;
  std::size_t maximum_edge_count_ = 0U;
  std::string source_file_;
};

}  // namespace qcar2_oampc

#endif  // OBSTACLE_MAP_HPP_
