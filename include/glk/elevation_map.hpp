#ifndef GLK_ELEVATION_MAP_HPP
#define GLK_ELEVATION_MAP_HPP

#include <GL/gl3w.h>
#include <glk/drawable.hpp>
#include <Eigen/Core>

namespace glk {

struct ElevationMapBuilder {
public:
  ElevationMapBuilder(double resolution, const std::vector<Eigen::Vector2f>& cells_xy) {
    Eigen::Vector2f min_xy = cells_xy[0];
    Eigen::Vector2f max_xy = cells_xy[0];
    for (const auto& cell : cells_xy) {
      min_xy = min_xy.cwiseMin(cell);
      max_xy = max_xy.cwiseMax(cell);
    }

    this->resolution = resolution;
    this->inv_resolution = 1.0 / resolution;
    this->origin = min_xy;
    this->size = ((max_xy - min_xy).array() / resolution).ceil().cast<int>();

    this->cell_indices.resize(cells_xy.size());
    for (size_t i = 0; i < cells_xy.size(); ++i) {
      Eigen::Vector2f cell_pos = cells_xy[i];
      Eigen::Vector2i cell_index = ((cell_pos - origin) * inv_resolution).cast<int>();
      this->cell_indices[i] = static_cast<std::uint64_t>(cell_index.y()) * size.x() + static_cast<std::uint64_t>(cell_index.x());
    }

    positions.resize(size.x() * size.y());
    for (int y = 0; y < size.y(); ++y) {
      for (int x = 0; x < size.x(); ++x) {
        Eigen::Vector2f pos = origin + Eigen::Vector2f(x * resolution, y * resolution);
        positions[y * size.x() + x] = pos;
      }
    }

    elevations.resize(size.x() * size.y(), std::numeric_limits<float>::quiet_NaN());
  }

  ~ElevationMapBuilder() = default;

  void set_elevation(const std::vector<float>& elevations) {
    if (elevations.size() != cell_indices.size()) {
      throw std::runtime_error("Elevations size does not match cell indices size.");
    }

    for (size_t i = 0; i < cell_indices.size(); ++i) {
      std::uint64_t index = cell_indices[i];
      if (index < elevations.size()) {
        this->elevations[index] = elevations[i];
      }
    }
  }

public:
  double resolution;      // Resolution of the map
  double inv_resolution;  // Inverse of the resolution

  Eigen::Vector2i size;                     // Size of the map in number of cells (width, height)
  Eigen::Vector2f origin;                   // Top-left corner of the map in world coordinates
  std::vector<std::uint64_t> cell_indices;  // Indices of the cells in the map

  std::vector<Eigen::Vector2f> positions;  // Positions of the cells in world coordinates
  std::vector<float> elevations;           // Elevation values for each cell
};

class ElevationMap : public glk::Drawable {
public:
  ElevationMap(const ElevationMapBuilder& builder) {
    resolution = builder.resolution;
    size = builder.size;
    origin = builder.origin;

    vao = vbo = ebo = 0;

    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    std::vector<Eigen::Vector3f> positions_with_elevation(builder.positions.size());
    for (size_t i = 0; i < builder.positions.size(); ++i) {
      positions_with_elevation[i] = Eigen::Vector3f(builder.positions[i].x(), builder.positions[i].y(), builder.elevations[i]);
    }

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, positions_with_elevation.size() * sizeof(Eigen::Vector3f), positions_with_elevation.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Eigen::Vector3f), (void*)0);
    glBindVertexArray(0);
  }

  ~ElevationMap() {
    if (ebo) glDeleteBuffers(1, &ebo);
    if (vbo) glDeleteBuffers(1, &vbo);
    if (vao) glDeleteVertexArrays(1, &vao);
  }

  void draw(glk::GLSLShader& shader) const {
    shader.use();
    glBindVertexArray(vao);
    glDrawArrays(GL_POINTS, 0, size.x() * size.y());
    glBindVertexArray(0);
  }

private:
  double resolution;
  Eigen::Vector2i size;
  Eigen::Vector2f origin;

  GLuint vao;
  GLuint vbo;
  GLuint ebo;
};

}  // namespace glk

#endif