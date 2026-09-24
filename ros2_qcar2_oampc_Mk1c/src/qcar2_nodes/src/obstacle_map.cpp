#include "obstacle_map.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

#include "yaml-cpp/yaml.h"

namespace qcar2_oampc
{
namespace
{
constexpr double kGeometryEpsilon = 1.0e-9;

// Return whether one scalar geometry value is finite.
bool isFinite(double value)
{
  return std::isfinite(value);
}

// Return twice the signed area of an ordered polygon.
double polygonTwiceSignedArea(const std::vector<Point2D> & vertices)
{
  double twice_area = 0.0;
  for (std::size_t index = 0; index < vertices.size(); ++index) {
    const Point2D & first = vertices[index];
    const Point2D & second = vertices[(index + 1U) % vertices.size()];
    twice_area += first.x * second.y - first.y * second.x;
  }
  return twice_area;
}

// Reverse polygon vertices when needed so they are counter-clockwise.
void ensureCounterClockwise(std::vector<Point2D> & vertices)
{
  if (polygonTwiceSignedArea(vertices) < 0.0) {
    std::reverse(vertices.begin(), vertices.end());
  }
}

// Return whether a polygon is convex with one consistent turn direction.
bool isConvexPolygon(const std::vector<Point2D> & vertices)
{
  if (vertices.size() < 3U) {
    return false;
  }

  double expected_sign = 0.0;
  for (std::size_t index = 0; index < vertices.size(); ++index) {
    const Point2D & first = vertices[index];
    const Point2D & second = vertices[(index + 1U) % vertices.size()];
    const Point2D & third = vertices[(index + 2U) % vertices.size()];
    const double first_x = second.x - first.x;
    const double first_y = second.y - first.y;
    const double second_x = third.x - second.x;
    const double second_y = third.y - second.y;
    const double cross = first_x * second_y - first_y * second_x;
    if (std::abs(cross) <= kGeometryEpsilon) {
      continue;
    }
    if (expected_sign == 0.0) {
      expected_sign = cross;
      continue;
    }
    if (cross * expected_sign < 0.0) {
      return false;
    }
  }
  return expected_sign != 0.0;
}

// Compute normalized half-spaces G p <= f for one convex polygon.
void computeHalfspaces(Obstacle & obstacle)
{
  obstacle.halfspace_matrix.clear();
  obstacle.halfspace_vector.clear();

  Point2D centroid;
  for (const Point2D & vertex : obstacle.vertices) {
    centroid.x += vertex.x;
    centroid.y += vertex.y;
  }
  const double inverse_count = 1.0 / static_cast<double>(obstacle.vertices.size());
  centroid.x *= inverse_count;
  centroid.y *= inverse_count;

  for (std::size_t edge_index = 0; edge_index < obstacle.vertices.size(); ++edge_index) {
    const Point2D & first = obstacle.vertices[edge_index];
    const Point2D & second = obstacle.vertices[(edge_index + 1U) % obstacle.vertices.size()];
    const double tangent_x = second.x - first.x;
    const double tangent_y = second.y - first.y;
    const double edge_length = std::hypot(tangent_x, tangent_y);
    if (edge_length <= kGeometryEpsilon) {
      continue;
    }

    double normal_x = tangent_y / edge_length;
    double normal_y = -tangent_x / edge_length;
    double face_bound = normal_x * first.x + normal_y * first.y;
    const double centroid_value = normal_x * centroid.x + normal_y * centroid.y;
    if (centroid_value > face_bound) {
      normal_x = -normal_x;
      normal_y = -normal_y;
      face_bound = -face_bound;
    }

    obstacle.halfspace_matrix.push_back({normal_x, normal_y});
    obstacle.halfspace_vector.push_back(face_bound);
  }

  if (obstacle.halfspace_vector.size() < 3U) {
    throw std::runtime_error("Obstacle polygon does not contain at least three valid edges.");
  }
}

}  // namespace

// Load and validate workspace and obstacle geometry from YAML.
void ObstacleMap::load(const std::string & file_path)
{
  if (file_path.empty()) {
    throw std::runtime_error("Obstacle geometry file path is empty.");
  }

  const YAML::Node root = YAML::LoadFile(file_path);
  if (!root["workspace"] || !root["obstacles"]) {
    throw std::runtime_error("Obstacle geometry YAML must contain workspace and obstacles.");
  }

  const YAML::Node workspace_node = root["workspace"];
  workspace_.x_min = workspace_node["x_min"].as<double>();
  workspace_.x_max = workspace_node["x_max"].as<double>();
  workspace_.y_min = workspace_node["y_min"].as<double>();
  workspace_.y_max = workspace_node["y_max"].as<double>();
  if (!isFinite(workspace_.x_min) || !isFinite(workspace_.x_max) ||
    !isFinite(workspace_.y_min) || !isFinite(workspace_.y_max) ||
    workspace_.x_min >= workspace_.x_max || workspace_.y_min >= workspace_.y_max)
  {
    throw std::runtime_error("Obstacle geometry YAML contains an invalid workspace rectangle.");
  }

  obstacles_.clear();
  maximum_edge_count_ = 0U;
  std::set<int> used_ids;
  const YAML::Node obstacles_node = root["obstacles"];
  if (!obstacles_node.IsSequence()) {
    throw std::runtime_error("Obstacle geometry YAML obstacles entry must be a sequence.");
  }

  for (const YAML::Node & obstacle_node : obstacles_node) {
    Obstacle obstacle;
    obstacle.id = obstacle_node["id"].as<int>();
    if (!used_ids.insert(obstacle.id).second) {
      throw std::runtime_error("Obstacle geometry YAML contains a duplicate obstacle id.");
    }

    const YAML::Node vertices_node = obstacle_node["vertices"];
    if (!vertices_node || !vertices_node.IsSequence() || vertices_node.size() < 3U) {
      throw std::runtime_error("Each obstacle must contain at least three polygon vertices.");
    }

    obstacle.vertices.reserve(vertices_node.size());
    for (const YAML::Node & vertex_node : vertices_node) {
      if (!vertex_node.IsSequence() || vertex_node.size() != 2U) {
        throw std::runtime_error("Each obstacle vertex must contain exactly two coordinates.");
      }
      Point2D vertex;
      vertex.x = vertex_node[0].as<double>();
      vertex.y = vertex_node[1].as<double>();
      if (!isFinite(vertex.x) || !isFinite(vertex.y)) {
        throw std::runtime_error("Obstacle geometry contains a non-finite vertex coordinate.");
      }
      obstacle.vertices.push_back(vertex);
    }

    ensureCounterClockwise(obstacle.vertices);
    if (!isConvexPolygon(obstacle.vertices)) {
      throw std::runtime_error("Obstacle geometry contains a non-convex polygon.");
    }
    computeHalfspaces(obstacle);
    maximum_edge_count_ = std::max(maximum_edge_count_, obstacle.edgeCount());
    obstacles_.push_back(std::move(obstacle));
  }

  source_file_ = file_path;
}

// Find one obstacle by its persistent identifier.
const Obstacle * ObstacleMap::findObstacleById(int obstacle_id) const
{
  for (const Obstacle & obstacle : obstacles_) {
    if (obstacle.id == obstacle_id) {
      return &obstacle;
    }
  }
  return nullptr;
}

}  // namespace qcar2_oampc
