#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "yaml-cpp/yaml.h"

// Minimum physical area for a valid obstacle.
constexpr double kMinObsArea = 0.05;

namespace
{
struct MapMetadata
{
  std::filesystem::path image_path;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;
  double occupied_threshold = 0.65;
  int negate = 0;
};

struct MetricPoint
{
  double x = 0.0;
  double y = 0.0;
};

struct WorkspaceGeometry
{
  double x_min = 0.0;
  double x_max = 0.0;
  double y_min = 0.0;
  double y_max = 0.0;
};

// Parse the command-line map and output paths.
void parseArguments(
  int argc,
  char ** argv,
  std::string & map_yaml_file,
  std::string & output_yaml_file)
{
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--map" && index + 1 < argc) {
      map_yaml_file = argv[++index];
    } else if (argument == "--output" && index + 1 < argc) {
      output_yaml_file = argv[++index];
    } else {
      throw std::runtime_error("Usage: generate_obstacle_map --map <map.yaml> --output <geometry.yaml>");
    }
  }
  if (map_yaml_file.empty() || output_yaml_file.empty()) {
    throw std::runtime_error("Both --map and --output are required.");
  }
}

// Load occupancy-map metadata and resolve the referenced PGM image path.
MapMetadata loadMapMetadata(const std::string & map_yaml_file)
{
  const YAML::Node root = YAML::LoadFile(map_yaml_file);
  if (!root["image"] || !root["resolution"] || !root["origin"]) {
    throw std::runtime_error("Map YAML must contain image, resolution, and origin.");
  }

  MapMetadata metadata;
  metadata.resolution = root["resolution"].as<double>();
  if (!std::isfinite(metadata.resolution) || metadata.resolution <= 0.0) {
    throw std::runtime_error("Map resolution must be finite and positive.");
  }

  const YAML::Node origin = root["origin"];
  if (!origin.IsSequence() || origin.size() < 3U) {
    throw std::runtime_error("Map origin must contain x, y, and yaw.");
  }
  metadata.origin_x = origin[0].as<double>();
  metadata.origin_y = origin[1].as<double>();
  metadata.origin_yaw = origin[2].as<double>();
  metadata.occupied_threshold = root["occupied_thresh"] ? root["occupied_thresh"].as<double>() : 0.65;
  metadata.negate = root["negate"] ? root["negate"].as<int>() : 0;

  std::filesystem::path image_path(root["image"].as<std::string>());
  if (image_path.is_relative()) {
    image_path = std::filesystem::path(map_yaml_file).parent_path() / image_path;
  }
  metadata.image_path = std::filesystem::weakly_canonical(image_path);
  return metadata;
}

// Convert a grayscale occupancy image into a binary occupied-pixel mask.
cv::Mat buildOccupiedMask(const cv::Mat & image, const MapMetadata & metadata)
{
  cv::Mat occupied(image.rows, image.cols, CV_8UC1, cv::Scalar(0));
  for (int row = 0; row < image.rows; ++row) {
    const unsigned char * source = image.ptr<unsigned char>(row);
    unsigned char * destination = occupied.ptr<unsigned char>(row);
    for (int column = 0; column < image.cols; ++column) {
      const double normalized = static_cast<double>(source[column]) / 255.0;
      const double occupancy_probability = metadata.negate == 0 ? 1.0 - normalized : normalized;
      if (occupancy_probability >= metadata.occupied_threshold) {
        destination[column] = 255;
      }
    }
  }
  return occupied;
}

// Remove connected occupied components smaller than a requested pixel area.
cv::Mat removeSmallComponents(const cv::Mat & binary_mask, int minimum_area_pixels)
{
  cv::Mat labels;
  cv::Mat statistics;
  cv::Mat centroids;
  const int number_of_labels = cv::connectedComponentsWithStats(
    binary_mask,
    labels,
    statistics,
    centroids,
    8,
    CV_32S);

  cv::Mat filtered(binary_mask.rows, binary_mask.cols, CV_8UC1, cv::Scalar(0));
  for (int label = 1; label < number_of_labels; ++label) {
    const int area = statistics.at<int>(label, cv::CC_STAT_AREA);
    if (area < minimum_area_pixels) {
      continue;
    }
    filtered.setTo(255, labels == label);
  }
  return filtered;
}

// Infer a conservative outer room pixel rectangle from dominant occupied geometry.
cv::Rect inferOuterRoomRectangle(const cv::Mat & occupied_mask, double resolution)
{
  const int minimum_component_area = std::max(
    8,
    static_cast<int>(std::round(0.0025 / (resolution * resolution))));
  const cv::Mat dominant_mask = removeSmallComponents(occupied_mask, minimum_component_area);
  std::vector<cv::Point> occupied_points;
  cv::findNonZero(dominant_mask, occupied_points);
  if (occupied_points.size() < 20U) {
    throw std::runtime_error("Not enough dominant occupied pixels were found to infer the room boundary.");
  }

  cv::Rect room_rectangle = cv::boundingRect(occupied_points);
  if (room_rectangle.width < 10 || room_rectangle.height < 10) {
    throw std::runtime_error("Inferred room boundary is too small.");
  }
  return room_rectangle;
}

// Convert one image pixel-boundary coordinate into the ROS map metric frame.
MetricPoint pixelBoundaryToMetric(
  double pixel_column,
  double pixel_row,
  int image_height,
  const MapMetadata & metadata)
{
  const double local_x = pixel_column * metadata.resolution;
  const double local_y = (static_cast<double>(image_height) - pixel_row) * metadata.resolution;
  const double cosine = std::cos(metadata.origin_yaw);
  const double sine = std::sin(metadata.origin_yaw);
  return MetricPoint{
    metadata.origin_x + cosine * local_x - sine * local_y,
    metadata.origin_y + sine * local_x + cosine * local_y};
}

// Convert the inferred room pixel rectangle into an axis-aligned metric workspace.
WorkspaceGeometry buildWorkspaceGeometry(
  const cv::Rect & room_rectangle,
  int image_height,
  const MapMetadata & metadata)
{
  const std::array<MetricPoint, 4> corners{{
    pixelBoundaryToMetric(room_rectangle.x, room_rectangle.y, image_height, metadata),
    pixelBoundaryToMetric(room_rectangle.x + room_rectangle.width, room_rectangle.y, image_height, metadata),
    pixelBoundaryToMetric(room_rectangle.x + room_rectangle.width, room_rectangle.y + room_rectangle.height, image_height, metadata),
    pixelBoundaryToMetric(room_rectangle.x, room_rectangle.y + room_rectangle.height, image_height, metadata)}};

  WorkspaceGeometry workspace;
  workspace.x_min = corners[0].x;
  workspace.x_max = corners[0].x;
  workspace.y_min = corners[0].y;
  workspace.y_max = corners[0].y;
  for (const MetricPoint & corner : corners) {
    workspace.x_min = std::min(workspace.x_min, corner.x);
    workspace.x_max = std::max(workspace.x_max, corner.x);
    workspace.y_min = std::min(workspace.y_min, corner.y);
    workspace.y_max = std::max(workspace.y_max, corner.y);
  }
  return workspace;
}

// Return twice the signed area of one metric polygon.
double polygonTwiceSignedArea(const std::vector<MetricPoint> & vertices)
{
  double twice_area = 0.0;
  for (std::size_t index = 0; index < vertices.size(); ++index) {
    const MetricPoint & first = vertices[index];
    const MetricPoint & second = vertices[(index + 1U) % vertices.size()];
    twice_area += first.x * second.y - first.y * second.x;
  }
  return twice_area;
}

// Extract four-edge oriented rectangles after removing wall-connected occupancy and small scan noise.
std::vector<std::vector<MetricPoint>> extractInteriorObstacleRectangles(
  const cv::Mat & occupied_mask,
  const cv::Rect & room_rectangle,
  const MapMetadata & metadata)
{
  cv::Mat room_occupied_mask(occupied_mask.rows, occupied_mask.cols, CV_8UC1, cv::Scalar(0));
  occupied_mask(room_rectangle).copyTo(room_occupied_mask(room_rectangle));

  cv::Mat wall_grouping_mask = room_occupied_mask.clone();
  const cv::Mat wall_grouping_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(
    wall_grouping_mask,
    wall_grouping_mask,
    cv::MORPH_CLOSE,
    wall_grouping_kernel);

  cv::Mat wall_labels;
  cv::Mat wall_statistics;
  cv::Mat wall_centroids;
  const int number_of_wall_labels = cv::connectedComponentsWithStats(
    wall_grouping_mask,
    wall_labels,
    wall_statistics,
    wall_centroids,
    8,
    CV_32S);

  std::vector<unsigned char> wall_connected_labels(
    static_cast<std::size_t>(number_of_wall_labels),
    0U);
  const int room_left = room_rectangle.x;
  const int room_right = room_rectangle.x + room_rectangle.width - 1;
  const int room_top = room_rectangle.y;
  const int room_bottom = room_rectangle.y + room_rectangle.height - 1;

  for (int column = room_left; column <= room_right; ++column) {
    const int top_label = wall_labels.at<int>(room_top, column);
    const int bottom_label = wall_labels.at<int>(room_bottom, column);
    if (top_label > 0) {
      wall_connected_labels[static_cast<std::size_t>(top_label)] = 1U;
    }
    if (bottom_label > 0) {
      wall_connected_labels[static_cast<std::size_t>(bottom_label)] = 1U;
    }
  }
  for (int row = room_top; row <= room_bottom; ++row) {
    const int left_label = wall_labels.at<int>(row, room_left);
    const int right_label = wall_labels.at<int>(row, room_right);
    if (left_label > 0) {
      wall_connected_labels[static_cast<std::size_t>(left_label)] = 1U;
    }
    if (right_label > 0) {
      wall_connected_labels[static_cast<std::size_t>(right_label)] = 1U;
    }
  }

  cv::Mat original_interior_mask(occupied_mask.rows, occupied_mask.cols, CV_8UC1, cv::Scalar(0));
  for (int row = room_top; row <= room_bottom; ++row) {
    const unsigned char * occupied_row = occupied_mask.ptr<unsigned char>(row);
    const int * wall_label_row = wall_labels.ptr<int>(row);
    unsigned char * interior_row = original_interior_mask.ptr<unsigned char>(row);
    for (int column = room_left; column <= room_right; ++column) {
      if (occupied_row[column] == 0) {
        continue;
      }

      const int wall_label = wall_label_row[column];
      if (wall_label > 0 &&
        wall_connected_labels[static_cast<std::size_t>(wall_label)] != 0U)
      {
        continue;
      }
      interior_row[column] = 255;
    }
  }

  cv::Mat grouping_mask = original_interior_mask.clone();
  const cv::Mat grouping_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(grouping_mask, grouping_mask, cv::MORPH_CLOSE, grouping_kernel);

  cv::Mat labels;
  cv::Mat statistics;
  cv::Mat centroids;
  const int number_of_labels = cv::connectedComponentsWithStats(
    grouping_mask,
    labels,
    statistics,
    centroids,
    8,
    CV_32S);

  const int minimum_obstacle_area_pixels = std::max(
    4,
    static_cast<int>(std::round(kMinObsArea / (metadata.resolution * metadata.resolution))));

  std::vector<std::vector<MetricPoint>> obstacle_rectangles;
  for (int label = 1; label < number_of_labels; ++label) {
    const int component_x = statistics.at<int>(label, cv::CC_STAT_LEFT);
    const int component_y = statistics.at<int>(label, cv::CC_STAT_TOP);
    const int component_width = statistics.at<int>(label, cv::CC_STAT_WIDTH);
    const int component_height = statistics.at<int>(label, cv::CC_STAT_HEIGHT);

    std::vector<cv::Point2f> occupied_cell_boundary_points;
    int original_occupied_pixel_count = 0;
    for (int row = component_y; row < component_y + component_height; ++row) {
      const unsigned char * original_row = original_interior_mask.ptr<unsigned char>(row);
      const int * label_row = labels.ptr<int>(row);
      for (int column = component_x; column < component_x + component_width; ++column) {
        if (label_row[column] != label || original_row[column] == 0) {
          continue;
        }

        ++original_occupied_pixel_count;
        const float left = static_cast<float>(column);
        const float right = static_cast<float>(column + 1);
        const float top = static_cast<float>(row);
        const float bottom = static_cast<float>(row + 1);
        occupied_cell_boundary_points.emplace_back(left, top);
        occupied_cell_boundary_points.emplace_back(right, top);
        occupied_cell_boundary_points.emplace_back(right, bottom);
        occupied_cell_boundary_points.emplace_back(left, bottom);
      }
    }

    if (original_occupied_pixel_count < minimum_obstacle_area_pixels) {
      continue;
    }
    if (occupied_cell_boundary_points.size() < 4U) {
      throw std::runtime_error("A retained interior obstacle does not contain enough occupied-cell boundary points.");
    }

    const cv::RotatedRect fitted_rectangle = cv::minAreaRect(occupied_cell_boundary_points);
    std::array<cv::Point2f, 4> rectangle_corners;
    fitted_rectangle.points(rectangle_corners.data());

    std::vector<MetricPoint> metric_vertices;
    metric_vertices.reserve(rectangle_corners.size());
    for (const cv::Point2f & corner : rectangle_corners) {
      metric_vertices.push_back(
        pixelBoundaryToMetric(corner.x, corner.y, occupied_mask.rows, metadata));
    }

    const double twice_area = polygonTwiceSignedArea(metric_vertices);
    if (std::abs(twice_area) <= 1.0e-12) {
      throw std::runtime_error("A retained interior obstacle produced a degenerate minimum-area rectangle.");
    }
    if (twice_area < 0.0) {
      std::reverse(metric_vertices.begin(), metric_vertices.end());
    }
    obstacle_rectangles.push_back(std::move(metric_vertices));
  }

  std::sort(
    obstacle_rectangles.begin(),
    obstacle_rectangles.end(),
    [](const std::vector<MetricPoint> & first, const std::vector<MetricPoint> & second) {
      const auto centroid = [](const std::vector<MetricPoint> & polygon) {
        MetricPoint center;
        for (const MetricPoint & point : polygon) {
          center.x += point.x;
          center.y += point.y;
        }
        center.x /= static_cast<double>(polygon.size());
        center.y /= static_cast<double>(polygon.size());
        return center;
      };
      const MetricPoint first_center = centroid(first);
      const MetricPoint second_center = centroid(second);
      if (std::abs(first_center.x - second_center.x) > 1.0e-9) {
        return first_center.x < second_center.x;
      }
      return first_center.y < second_center.y;
    });
  return obstacle_rectangles;
}

// Require every retained interior obstacle to be a valid four-vertex rectangle.
void validateObstacleRectangles(const std::vector<std::vector<MetricPoint>> & obstacle_rectangles)
{
  constexpr std::size_t expected_vertices_per_obstacle = 4U;

  for (std::size_t obstacle_index = 0; obstacle_index < obstacle_rectangles.size(); ++obstacle_index) {
    if (obstacle_rectangles[obstacle_index].size() != expected_vertices_per_obstacle) {
      throw std::runtime_error(
        "Obstacle " + std::to_string(obstacle_index) +
        " does not contain exactly 4 rectangle vertices.");
    }
  }
}

// Write only processed workspace and convex obstacle geometry to YAML.
void writeObstacleGeometry(
  const std::string & output_yaml_file,
  const WorkspaceGeometry & workspace,
  const std::vector<std::vector<MetricPoint>> & obstacle_polygons)
{
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(12);
  emitter << YAML::BeginMap;
  emitter << YAML::Key << "workspace" << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "x_min" << YAML::Value << workspace.x_min;
  emitter << YAML::Key << "x_max" << YAML::Value << workspace.x_max;
  emitter << YAML::Key << "y_min" << YAML::Value << workspace.y_min;
  emitter << YAML::Key << "y_max" << YAML::Value << workspace.y_max;
  emitter << YAML::EndMap;
  emitter << YAML::Key << "obstacles" << YAML::Value << YAML::BeginSeq;
  for (std::size_t obstacle_index = 0; obstacle_index < obstacle_polygons.size(); ++obstacle_index) {
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "id" << YAML::Value << static_cast<int>(obstacle_index);
    emitter << YAML::Key << "vertices" << YAML::Value << YAML::BeginSeq;
    for (const MetricPoint & vertex : obstacle_polygons[obstacle_index]) {
      emitter << YAML::Flow << YAML::BeginSeq << vertex.x << vertex.y << YAML::EndSeq;
    }
    emitter << YAML::EndSeq;
    emitter << YAML::EndMap;
  }
  emitter << YAML::EndSeq;
  emitter << YAML::EndMap;

  const std::filesystem::path output_path(output_yaml_file);
  std::filesystem::create_directories(output_path.parent_path());
  std::ofstream output(output_path);
  if (!output) {
    throw std::runtime_error("Cannot open obstacle geometry output file: " + output_yaml_file);
  }
  output << emitter.c_str() << '\n';
}

}  // namespace

// Generate processed OAMPC geometry from the current occupancy map.
int main(int argc, char ** argv)
{
  try {
    std::string map_yaml_file;
    std::string output_yaml_file;
    parseArguments(argc, argv, map_yaml_file, output_yaml_file);
    const MapMetadata metadata = loadMapMetadata(map_yaml_file);
    const cv::Mat image = cv::imread(metadata.image_path.string(), cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
      throw std::runtime_error("Cannot read occupancy image: " + metadata.image_path.string());
    }

    const cv::Mat occupied_mask = buildOccupiedMask(image, metadata);

    cv::Mat room_mask = occupied_mask.clone();
    const cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(room_mask, room_mask, cv::MORPH_CLOSE, close_kernel);
    const cv::Rect room_rectangle = inferOuterRoomRectangle(room_mask, metadata.resolution);
    const WorkspaceGeometry workspace = buildWorkspaceGeometry(
      room_rectangle,
      image.rows,
      metadata);

    const std::vector<std::vector<MetricPoint>> obstacle_rectangles =
      extractInteriorObstacleRectangles(occupied_mask, room_rectangle, metadata);
    validateObstacleRectangles(obstacle_rectangles);
    writeObstacleGeometry(output_yaml_file, workspace, obstacle_rectangles);

    std::cout << "There are " << obstacle_rectangles.size() << " obstacles in this map!" << std::endl;
    std::cout << "Generated " << output_yaml_file << " with "
              << obstacle_rectangles.size() << " interior obstacle rectangles, each with 4 vertices."
              << std::endl;
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << "Obstacle-map generation failed: " << exception.what() << std::endl;
    return 1;
  }
}
