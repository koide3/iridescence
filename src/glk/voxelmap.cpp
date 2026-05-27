#include <glk/voxelmap.hpp>

namespace glk {

void VoxelMapOptions::set_voxel_alpha(float alpha) {
  override_voxel_color = true;
  voxel_color.w() = alpha;
}

void VoxelMapOptions::set_voxel_color(const Eigen::Vector4f& color) {
  override_voxel_color_mode = true;
  override_voxel_color = true;

  voxel_color_mode = 1;  // flat_color
  voxel_color = color;
}

void VoxelMapOptions::set_edge_alpha(float alpha) {
  override_edge_color = true;
  edge_color.w() = alpha;
}

void VoxelMapOptions::set_edge_color(const Eigen::Vector4f& color) {
  override_edge_color_mode = true;
  override_edge_color = true;

  edge_color_mode = 1;  // flat_color
  edge_color = color;
}

VoxelMap::VoxelMap(const Eigen::Vector3i* voxel_coords, int num_voxels, double resolution, const VoxelMapOptions& options) : options(options) {
  this->num_voxels = num_voxels;
  vao = vbo = ebo_voxels = ebo_edges = 0;

  const float res = static_cast<float>(resolution);

  // Pre-compute vertex offsets with float resolution (avoid repeated double->float conversion)
  const std::array<Eigen::Vector3f, 8> vertex_offsets = {
    Eigen::Vector3f(0.0f, 0.0f, 0.0f),
    Eigen::Vector3f(res, 0.0f, 0.0f),
    Eigen::Vector3f(res, res, 0.0f),
    Eigen::Vector3f(0.0f, res, 0.0f),
    Eigen::Vector3f(0.0f, 0.0f, res),
    Eigen::Vector3f(res, 0.0f, res),
    Eigen::Vector3f(res, res, res),
    Eigen::Vector3f(0.0f, res, res),
  };

  static constexpr std::array<unsigned int, 36> voxel_index_offsets = {
    0, 1, 2, 0, 2, 3,  //
    4, 5, 6, 4, 6, 7,  //
    0, 1, 5, 0, 5, 4,  //
    1, 2, 6, 1, 6, 5,  //
    2, 3, 7, 2, 7, 6,  //
    3, 0, 4, 3, 4, 7,  //
  };

  static constexpr std::array<unsigned int, 24> edge_index_offsets = {
    0, 1, 1, 2, 2, 3, 3, 0,  //
    4, 5, 5, 6, 6, 7, 7, 4,  //
    0, 4, 1, 5, 2, 6, 3, 7,  //
  };

  const GLsizeiptr vertex_buf_size = static_cast<GLsizeiptr>(sizeof(Eigen::Vector3f)) * num_voxels * 8;
  const GLsizeiptr voxel_idx_buf_size = static_cast<GLsizeiptr>(sizeof(unsigned int)) * num_voxels * 36;
  const GLsizeiptr edge_idx_buf_size = static_cast<GLsizeiptr>(sizeof(unsigned int)) * num_voxels * 24;
  constexpr GLbitfield map_flags = GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT | GL_MAP_UNSYNCHRONIZED_BIT;

  // Create all GL objects and allocate buffers upfront
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glGenBuffers(1, &ebo_voxels);
  glGenBuffers(1, &ebo_edges);

  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, vertex_buf_size, nullptr, GL_STATIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_voxels);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, voxel_idx_buf_size, nullptr, GL_STATIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_edges);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, edge_idx_buf_size, nullptr, GL_STATIC_DRAW);

  // Map all three buffers simultaneously
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  auto* vertices = static_cast<Eigen::Vector3f*>(glMapBufferRange(GL_ARRAY_BUFFER, 0, vertex_buf_size, map_flags));

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_voxels);
  auto* voxel_indices = static_cast<unsigned int*>(glMapBufferRange(GL_ELEMENT_ARRAY_BUFFER, 0, voxel_idx_buf_size, map_flags));

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_edges);
  auto* edge_indices = static_cast<unsigned int*>(glMapBufferRange(GL_ELEMENT_ARRAY_BUFFER, 0, edge_idx_buf_size, map_flags));

  // Fill face indices
  for (int i = 0; i < num_voxels; i++) {
    const unsigned int base = i * 8;
    unsigned int* dst = voxel_indices + i * 36;
    for (int j = 0; j < 36; j++) {
      dst[j] = base + voxel_index_offsets[j];
    }
  }

  // Fill edge indices
  for (int i = 0; i < num_voxels; i++) {
    const unsigned int base = i * 8;
    unsigned int* dst = edge_indices + i * 24;
    for (int j = 0; j < 24; j++) {
      dst[j] = base + edge_index_offsets[j];
    }
  }

  // Fill vertices
  for (int i = 0; i < num_voxels; i++) {
    const Eigen::Vector3f origin = voxel_coords[i].cast<float>() * res;
    Eigen::Vector3f* dst = vertices + i * 8;
    for (int j = 0; j < 8; j++) {
      dst[j] = origin + vertex_offsets[j];
    }
  }

  // Unmap all buffers (driver can now DMA the data to VRAM)
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glUnmapBuffer(GL_ARRAY_BUFFER);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_voxels);
  glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_edges);
  glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

VoxelMap::~VoxelMap() {
  if (ebo_edges) {
    glDeleteBuffers(1, &ebo_edges);
  }
  if (ebo_voxels) {
    glDeleteBuffers(1, &ebo_voxels);
  }
  if (vbo) {
    glDeleteBuffers(1, &vbo);
  }
  if (vao) {
    glDeleteVertexArrays(1, &vao);
  }
}

void VoxelMap::draw(glk::GLSLShader& shader) const {
  glBindVertexArray(vao);

  GLint position_loc = shader.attrib("vert_position");
  glEnableVertexAttribArray(position_loc);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);

  // draw voxels
  if (options.draw_voxels) {
    if (options.override_voxel_color_mode) {
      shader.set_uniform("color_mode", options.voxel_color_mode);
    }
    if (options.override_voxel_color) {
      shader.set_uniform("material_color", options.voxel_color);
    }

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_voxels);
    glDrawElements(GL_TRIANGLES, num_voxels * 36, GL_UNSIGNED_INT, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  }

  // draw edges
  if (options.draw_edges) {
    if (options.override_edge_color_mode) {
      shader.set_uniform("color_mode", options.edge_color_mode);
    }
    if (options.override_edge_color) {
      shader.set_uniform("material_color", options.edge_color);
    }

    glLineWidth(options.edge_line_width);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_edges);
    glDrawElements(GL_LINES, num_voxels * 24, GL_UNSIGNED_INT, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  }

  glDisableVertexAttribArray(position_loc);
}

}  // namespace glk