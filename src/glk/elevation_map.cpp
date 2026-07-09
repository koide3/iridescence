#include <glk/elevation_map.hpp>

#include <Eigen/Dense>

namespace glk {

ElevationMapBuilder::ElevationMapBuilder(double resolution, const Eigen::Vector2i& size, const Eigen::Vector2f& origin)
: resolution(resolution),
  inv_resolution(1.0 / resolution),
  size(size),
  origin(origin) {
  elevation_map.setConstant(size.y(), size.x(), std::numeric_limits<float>::quiet_NaN());
}

ElevationMapBuilder::~ElevationMapBuilder() {}

template <typename Vector>
std::vector<std::uint64_t> ElevationMapBuilder::calc_cell_indices(const std::vector<Vector>& cells) const {
  const auto fast_floor = [](const Eigen::Array2f& x) -> Eigen::Array2i {
    const Eigen::Array2i ncoord = x.cast<int>();
    return ncoord - (x < ncoord.cast<float>()).cast<int>();
  };

  std::vector<std::uint64_t> cell_indices(cells.size());
  for (size_t i = 0; i < cells.size(); ++i) {
    Eigen::Vector2f cell_pos(cells[i][0], cells[i][1]);
    Eigen::Vector2i cell_index = fast_floor((cell_pos - origin) * inv_resolution);
    if (cell_index.x() < 0 || cell_index.x() >= size.x() || cell_index.y() < 0 || cell_index.y() >= size.y()) {
      cell_indices[i] = std::numeric_limits<std::uint64_t>::max();  // Out of bounds
    } else {
      cell_indices[i] = static_cast<std::uint64_t>(cell_index[1]) * size.x() + static_cast<std::uint64_t>(cell_index[0]);
    }
  }
  return cell_indices;
}

template <typename Vector>
void ElevationMapBuilder::set_elevation(const std::vector<Vector>& cells, int elevation_index, const std::vector<std::uint64_t>& cell_indices, bool average) {
  if (cells.size() != cell_indices.size()) {
    throw std::runtime_error("Cells size does not match cell indices size.");
  }

  if (!average) {
    count_map.resize(0, 0);  // Clear the count map since we are not averaging

    for (size_t i = 0; i < cell_indices.size(); ++i) {
      const std::uint64_t index = cell_indices[i];
      if (index == std::numeric_limits<std::uint64_t>::max()) {
        continue;
      }

      elevation_map.data()[index] = cells[i][elevation_index];  // Assuming the elevation is stored in the specified index
    }
  } else {
    if (count_map.size() != elevation_map.size()) {
      count_map = Eigen::Array<float, -1, -1, Eigen::RowMajor>::Constant(size.y(), size.x(), 0.0f);
      elevation_map = Eigen::Array<float, -1, -1, Eigen::RowMajor>::Constant(size.y(), size.x(), 0.0f);
    } else {
      // Restore the accumulated sum from the previous averages. Unobserved cells hold NaN, so replace them with 0.
      elevation_map = (count_map > 0.5f).select(elevation_map * count_map, 0.0f);
    }

    for (size_t i = 0; i < cell_indices.size(); ++i) {
      const std::uint64_t index = cell_indices[i];
      if (index == std::numeric_limits<std::uint64_t>::max()) {
        continue;
      }

      elevation_map.data()[index] += cells[i][elevation_index];  // Assuming the elevation is stored in the specified index
      count_map.data()[index] += 1.0f;
    }

    elevation_map = (count_map > 0.5f).select(elevation_map / count_map, std::numeric_limits<float>::quiet_NaN()).eval();
  }
}

template <typename Vector>
void ElevationMapBuilder::set_cmap(const std::vector<Vector>& cells, int cmap_index, const std::vector<std::uint64_t>& cell_indices, bool average) {
  if (cells.size() != cell_indices.size()) {
    throw std::runtime_error("Cells size does not match cell indices size.");
  }

  if (color_map.size() != elevation_map.size()) {
    color_map = Eigen::Array<float, -1, -1, Eigen::RowMajor>::Constant(size.y(), size.x(), 0.0f);
  }

  if (!average) {
    for (size_t i = 0; i < cell_indices.size(); ++i) {
      const std::uint64_t index = cell_indices[i];
      if (index == std::numeric_limits<std::uint64_t>::max()) {
        continue;
      }

      color_map.data()[index] = cells[i][cmap_index];  // Assuming the color value is stored in the specified index
    }
  } else {
    Eigen::Array<float, -1, -1, Eigen::RowMajor> color_map = Eigen::Array<float, -1, -1, Eigen::RowMajor>::Constant(size.y(), size.x(), 0.0f);
    Eigen::Array<float, -1, -1, Eigen::RowMajor> counts = Eigen::Array<float, -1, -1, Eigen::RowMajor>::Constant(size.y(), size.x(), 0.0f);

    for (size_t i = 0; i < cell_indices.size(); ++i) {
      const std::uint64_t index = cell_indices[i];
      if (index == std::numeric_limits<std::uint64_t>::max()) {
        continue;
      }

      color_map.data()[index] += cells[i][cmap_index];  // Assuming the color value is stored in the specified index
      counts.data()[index] += 1.0f;
    }

    color_map = (counts > 0.5f).select(color_map / counts, color_map).eval();
  }
}

template <int WindowSize>
void ElevationMapBuilder::filter_elevation_map(bool process_valid_cells) {
  const int shift = WindowSize / 2;
  const bool cmap_exists = (color_map.size() > 0);

  Eigen::Array<float, -1, -1, Eigen::RowMajor> filtered_map = elevation_map;
  Eigen::Array<float, -1, -1, Eigen::RowMajor> filtered_cmap = color_map;

  for (int y = shift; y < size.y() - shift; y++) {
    for (int x = shift; x < size.x() - shift; x++) {
      if (!process_valid_cells && std::isfinite(elevation_map(y, x))) {
        continue;  // Skip valid cells if process_valid_cells is false
      }

      Eigen::Array<float, WindowSize, WindowSize, Eigen::RowMajor> neighbors = elevation_map.block<WindowSize, WindowSize>(y - shift, x - shift);
      const Eigen::Array<bool, WindowSize, WindowSize> valid_mask = neighbors.isFinite();
      const int valid_count = valid_mask.count();
      if (valid_count == 0) {
        continue;
      }

      neighbors = valid_mask.select(neighbors, Eigen::Array<float, WindowSize, WindowSize>::Zero());
      filtered_map(y, x) = neighbors.sum() / static_cast<float>(valid_count);

      if (cmap_exists) {
        Eigen::Array<float, WindowSize, WindowSize, Eigen::RowMajor> cmap_neighbors = color_map.block<WindowSize, WindowSize>(y - shift, x - shift);
        cmap_neighbors = valid_mask.select(cmap_neighbors, Eigen::Array<float, WindowSize, WindowSize>::Zero());
        filtered_cmap(y, x) = cmap_neighbors.sum() / static_cast<float>(valid_count);
      }
    }
  }

  elevation_map = std::move(filtered_map);
  color_map = std::move(filtered_cmap);
}

template <int WindowSize>
void ElevationMapBuilder::calc_cmap_gradient() {
  if (color_map.size() == 0) {
    return;
  }

  const int shift = WindowSize / 2;
  Eigen::Array<float, -1, -1, Eigen::RowMajor> gradient_map = Eigen::Array<float, -1, -1, Eigen::RowMajor>::Constant(size.y(), size.x(), std::numeric_limits<float>::quiet_NaN());

  for (int y = shift; y < size.y() - shift; y++) {
    for (int x = shift; x < size.x() - shift; x++) {
      if (!std::isfinite(color_map(y, x))) {
        continue;  // Skip cells without a valid color value
      }

      // Fit a local plane cmap = a * dx + b * dy + c to the finite neighbors and use the plane slope (a, b) as the gradient.
      Eigen::Matrix3d AtA = Eigen::Matrix3d::Zero();
      Eigen::Vector3d Atb = Eigen::Vector3d::Zero();
      int valid_count = 0;

      for (int dy = -shift; dy <= shift; dy++) {
        for (int dx = -shift; dx <= shift; dx++) {
          const float value = color_map(y + dy, x + dx);
          if (!std::isfinite(value)) {
            continue;  // Ignore NaN cells
          }

          const Eigen::Vector3d a(dx * resolution, dy * resolution, 1.0);
          AtA += a * a.transpose();
          Atb += a * static_cast<double>(value);
          valid_count++;
        }
      }

      if (valid_count < 3) {
        continue;  // Not enough valid neighbors to estimate a gradient
      }

      const Eigen::Vector3d coeffs = AtA.ldlt().solve(Atb);
      gradient_map(y, x) = static_cast<float>(std::sqrt(coeffs.x() * coeffs.x() + coeffs.y() * coeffs.y()));
    }
  }

  color_map = std::move(gradient_map);
}

template std::vector<std::uint64_t> ElevationMapBuilder::calc_cell_indices(const std::vector<Eigen::Matrix<float, 2, 1>>& cells) const;
template std::vector<std::uint64_t> ElevationMapBuilder::calc_cell_indices(const std::vector<Eigen::Matrix<float, 3, 1>>& cells) const;
template std::vector<std::uint64_t> ElevationMapBuilder::calc_cell_indices(const std::vector<Eigen::Matrix<float, 6, 1>>& cells) const;

template void
ElevationMapBuilder::set_elevation(const std::vector<Eigen::Matrix<float, 1, 1>>& cells, int elevation_index, const std::vector<std::uint64_t>& cell_indices, bool average);
template void
ElevationMapBuilder::set_elevation(const std::vector<Eigen::Matrix<float, 3, 1>>& cells, int elevation_index, const std::vector<std::uint64_t>& cell_indices, bool average);
template void
ElevationMapBuilder::set_elevation(const std::vector<Eigen::Matrix<float, 6, 1>>& cells, int elevation_index, const std::vector<std::uint64_t>& cell_indices, bool average);

template void ElevationMapBuilder::set_cmap(const std::vector<Eigen::Matrix<float, 1, 1>>& cells, int cmap_index, const std::vector<std::uint64_t>& cell_indices, bool average);
template void ElevationMapBuilder::set_cmap(const std::vector<Eigen::Matrix<float, 3, 1>>& cells, int cmap_index, const std::vector<std::uint64_t>& cell_indices, bool average);
template void ElevationMapBuilder::set_cmap(const std::vector<Eigen::Matrix<float, 6, 1>>& cells, int cmap_index, const std::vector<std::uint64_t>& cell_indices, bool average);

template void ElevationMapBuilder::filter_elevation_map<3>(bool process_valid_cells);
template void ElevationMapBuilder::filter_elevation_map<5>(bool process_valid_cells);
template void ElevationMapBuilder::filter_elevation_map<7>(bool process_valid_cells);

template void ElevationMapBuilder::calc_cmap_gradient<3>();
template void ElevationMapBuilder::calc_cmap_gradient<5>();
template void ElevationMapBuilder::calc_cmap_gradient<7>();

ElevationMap::ElevationMap(const ElevationMapBuilder& builder, const MeshRenderingOptions& options)
: options(options),
  resolution(builder.resolution),
  size(builder.size),
  origin(builder.origin) {
  //
  vao = vbo = cbo = ebo_faces = ebo_edges = 0;
  num_indices_faces = num_indices_edges = 0;

  num_indices_faces = (size.x() - 1) * (size.y() - 1) * 6;
  num_indices_edges = 2 * ((size.x() - 1) * size.y() + (size.y() - 1) * size.x());

  std::vector<unsigned int> indices(num_indices_faces + num_indices_edges);
  for (int y = 0; y < size.y() - 1; y++) {
    for (int x = 0; x < size.x() - 1; x++) {
      const int v1 = y * size.x() + x;
      const int v2 = y * size.x() + (x + 1);
      const int v3 = (y + 1) * size.x() + x;
      const int v4 = (y + 1) * size.x() + (x + 1);

      const int idx = y * (size.x() - 1) + x;
      indices[idx * 6 + 0] = v1;
      indices[idx * 6 + 1] = v2;
      indices[idx * 6 + 2] = v3;
      indices[idx * 6 + 3] = v2;
      indices[idx * 6 + 4] = v4;
      indices[idx * 6 + 5] = v3;
    }
  }

  // indices of lines for edges (appended after the face indices)
  const int edges_offset = num_indices_faces;
  for (int y = 0; y < size.y(); y++) {
    for (int x = 0; x < size.x() - 1; x++) {
      const int v1 = y * size.x() + x;
      const int v2 = y * size.x() + (x + 1);
      const int idx = y * (size.x() - 1) + x;
      indices[edges_offset + idx * 2 + 0] = v1;
      indices[edges_offset + idx * 2 + 1] = v2;
    }
  }

  for (int y = 0; y < size.y() - 1; y++) {
    for (int x = 0; x < size.x(); x++) {
      const int v1 = y * size.x() + x;
      const int v2 = (y + 1) * size.x() + x;
      const int idx = (size.x() - 1) * size.y() + y * size.x() + x;
      indices[edges_offset + idx * 2 + 0] = v1;
      indices[edges_offset + idx * 2 + 1] = v2;
    }
  }

  std::vector<Eigen::Vector3f> positions(size.x() * size.y());
  for (int y = 0; y < size.y(); y++) {
    for (int x = 0; x < size.x(); x++) {
      const int idx = y * size.x() + x;
      positions[idx] = Eigen::Vector3f(origin.x() + x * resolution, origin.y() + y * resolution, builder.elevation_map(y, x));
    }
  }

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(Eigen::Vector3f), positions.data(), GL_STATIC_DRAW);

  if (builder.color_map.size() > 0) {
    glGenBuffers(1, &cbo);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glBufferData(GL_ARRAY_BUFFER, builder.color_map.size() * sizeof(float), builder.color_map.data(), GL_STATIC_DRAW);
  }

  glGenBuffers(1, &ebo_faces);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_faces);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, num_indices_faces * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

  if (options.draw_edges) {
    glGenBuffers(1, &ebo_edges);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_edges);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, num_indices_edges * sizeof(unsigned int), indices.data() + num_indices_faces, GL_STATIC_DRAW);
  }

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Eigen::Vector3f), (void*)0);
  glBindVertexArray(0);
}

ElevationMap::~ElevationMap() {
  if (ebo_faces) glDeleteBuffers(1, &ebo_faces);
  if (ebo_edges) glDeleteBuffers(1, &ebo_edges);
  if (vbo) glDeleteBuffers(1, &vbo);
  if (cbo) glDeleteBuffers(1, &cbo);
  if (vao) glDeleteVertexArrays(1, &vao);
}

void ElevationMap::draw(glk::GLSLShader& shader) const {
  glBindVertexArray(vao);
  const GLint position_loc = shader.attrib("vert_position");
  glEnableVertexAttribArray(position_loc);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, sizeof(Eigen::Vector3f), (void*)0);

  if (cbo) {
    GLint cmap_loc = shader.attrib("vert_cmap");
    glEnableVertexAttribArray(cmap_loc);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glVertexAttribPointer(cmap_loc, 1, GL_FLOAT, GL_FALSE, sizeof(float), (void*)0);
  }

  if (options.draw_faces) {
    if (options.override_face_color_mode) {
      shader.set_uniform("color_mode", options.face_color_mode);
    }
    if (options.override_face_color) {
      shader.set_uniform("material_color", options.face_color);
    }

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_faces);
    glDrawElements(GL_TRIANGLES, num_indices_faces, GL_UNSIGNED_INT, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  }

  if (options.draw_edges) {
    if (options.override_edge_color_mode) {
      shader.set_uniform("color_mode", options.edge_color_mode);
    }
    if (options.override_edge_color) {
      shader.set_uniform("material_color", options.edge_color);
    }

    glLineWidth(options.edge_line_width);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_edges);
    glDrawElements(GL_LINES, num_indices_edges, GL_UNSIGNED_INT, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glLineWidth(1.0f);
  }

  if (cbo) {
    GLint cmap_loc = shader.attrib("vert_cmap");
    glDisableVertexAttribArray(cmap_loc);
  }

  glBindVertexArray(0);
}

}  // namespace glk
