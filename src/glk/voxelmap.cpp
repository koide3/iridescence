#include <glk/voxelmap.hpp>
#include <glk/async_buffer_copy.hpp>

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

  std::vector<Eigen::Vector3f> vertices(num_voxels * 8);     // 8 vertices per voxel
  std::vector<unsigned int> voxel_indices(num_voxels * 36);  // 12 triangles per voxel, 3 vertices per triangle
  std::vector<unsigned int> edge_indices(num_voxels * 24);   // 12 edges per voxel, 2 vertices per edge

  // fill vertices
  const std::array<Eigen::Vector3f, 8> vertex_offsets = {
    Eigen::Vector3f(0.0f, 0.0f, 0.0f),
    Eigen::Vector3f(resolution, 0.0f, 0.0f),
    Eigen::Vector3f(resolution, resolution, 0.0f),
    Eigen::Vector3f(0.0f, resolution, 0.0f),
    Eigen::Vector3f(0.0f, 0.0f, resolution),
    Eigen::Vector3f(resolution, 0.0f, resolution),
    Eigen::Vector3f(resolution, resolution, resolution),
    Eigen::Vector3f(0.0f, resolution, resolution),
  };

  for (int i = 0; i < num_voxels; i++) {
    const Eigen::Vector3f origin = voxel_coords[i].cast<float>() * resolution;
    for (int j = 0; j < 8; j++) {
      vertices[i * 8 + j] = origin + vertex_offsets[j];
    }
  }

  // fill voxel_indices
  const std::array<unsigned int, 36> voxel_index_offsets = {
    0, 1, 2, 0, 2, 3,  //
    4, 5, 6, 4, 6, 7,  //
    0, 1, 5, 0, 5, 4,  //
    1, 2, 6, 1, 6, 5,  //
    2, 3, 7, 2, 7, 6,  //
    3, 0, 4, 3, 4, 7,  //
  };

  for (int i = 0; i < num_voxels; i++) {
    for (int j = 0; j < 36; j++) {
      voxel_indices[i * 36 + j] = i * 8 + voxel_index_offsets[j];
    }
  }

  // fill edge_indices
  const std::array<unsigned int, 24> edge_index_offsets = {
    0, 1, 1, 2, 2, 3, 3, 0,  //
    4, 5, 5, 6, 6, 7, 7, 4,  //
    0, 4, 1, 5, 2, 6, 3, 7,  //
  };

  for (int i = 0; i < num_voxels; i++) {
    for (int j = 0; j < 24; j++) {
      edge_indices[i * 24 + j] = i * 8 + edge_index_offsets[j];
    }
  }

  // create buffers
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(Eigen::Vector3f) * vertices.size(), nullptr, GL_STATIC_DRAW);
  write_buffer_async(GL_ARRAY_BUFFER, sizeof(Eigen::Vector3f) * vertices.size(), vertices.data());

  glGenBuffers(1, &ebo_voxels);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_voxels);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * voxel_indices.size(), nullptr, GL_STATIC_DRAW);
  write_buffer_async(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * voxel_indices.size(), voxel_indices.data());

  glGenBuffers(1, &ebo_edges);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_edges);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * edge_indices.size(), nullptr, GL_STATIC_DRAW);
  write_buffer_async(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * edge_indices.size(), edge_indices.data());

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