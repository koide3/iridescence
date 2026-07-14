#include <glk/voxelmap.hpp>

#include <array>
#include <vector>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <glk/path.hpp>
#include <glk/console_colors.hpp>

namespace glk {

MeshRenderingOptions VoxelMapOptions::to_mesh_rendering_options() const {
  MeshRenderingOptions opts;
  opts.draw_faces = draw_voxels;
  opts.override_face_color_mode = override_voxel_color_mode;
  opts.override_face_color = override_voxel_color;
  opts.face_color_mode = voxel_color_mode;
  opts.face_color = voxel_color;

  opts.draw_edges = draw_edges;
  opts.override_edge_color_mode = override_edge_color_mode;
  opts.override_edge_color = override_edge_color;
  opts.edge_color_mode = edge_color_mode;
  opts.edge_color = edge_color;
  opts.edge_line_width = edge_line_width;
  return opts;
}

void VoxelMapOptions::set_voxel_alpha(float alpha) {
  draw_voxels = true;
  override_voxel_color = true;
  voxel_color.w() = alpha;
}

void VoxelMapOptions::set_voxel_color(const Eigen::Vector4f& color) {
  draw_voxels = true;
  override_voxel_color_mode = true;
  override_voxel_color = true;

  voxel_color_mode = 1;  // flat_color
  voxel_color = color;
}

void VoxelMapOptions::set_edge_alpha(float alpha) {
  draw_edges = true;
  override_edge_color = true;
  edge_color.w() = alpha;
}

void VoxelMapOptions::set_edge_color(const Eigen::Vector4f& color) {
  draw_edges = true;
  override_edge_color_mode = true;
  override_edge_color = true;

  edge_color_mode = 1;  // flat_color
  edge_color = color;
}

// VoxelMap

std::shared_ptr<glk::GLSLShader> VoxelMap::shader = nullptr;

VoxelMap::VoxelMap(const Eigen::Vector3i* voxel_coords, int num_voxels, double resolution, const VoxelMapOptions& options)
: VoxelMap(voxel_coords, num_voxels, resolution, options.to_mesh_rendering_options()) {
  std::cerr << glk::console::yellow << "warning: VoxelMapOptions is deprecated. Use MeshRenderingOptions instead." << glk::console::reset << std::endl;
}

VoxelMap::VoxelMap(const Eigen::Vector3i* voxel_coords, int num_voxels, double resolution, const MeshRenderingOptions& options) : options(options) {
  this->num_voxels = num_voxels;
  this->resolution = resolution;
  vao = cube_vbo = cube_ebo_faces = cube_ebo_edges = coords_vbo = 0;

  if (!shader) {
    std::unordered_set<std::string> includes;
    includes.insert(glk::get_data_path() + "/shader/voxelmap/rainbow_custom.vert");

    shader = std::make_shared<GLSLShader>();
    shader->attach_source(glk::get_data_path() + "/shader/rainbow.vert", includes, "", GL_VERTEX_SHADER);
    shader->attach_source(glk::get_data_path() + "/shader/rainbow.frag", GL_FRAGMENT_SHADER);
    shader->link_program();
  }

  // Pre-compute vertex offsets with float resolution
  const float res = resolution;
  const std::array<Eigen::Vector3f, 8> cube_vertices = {
    Eigen::Vector3f(0.0f, 0.0f, 0.0f),
    Eigen::Vector3f(res, 0.0f, 0.0f),
    Eigen::Vector3f(res, res, 0.0f),
    Eigen::Vector3f(0.0f, res, 0.0f),
    Eigen::Vector3f(0.0f, 0.0f, res),
    Eigen::Vector3f(res, 0.0f, res),
    Eigen::Vector3f(res, res, res),
    Eigen::Vector3f(0.0f, res, res),
  };

  static constexpr std::array<unsigned int, 36> cube_face_indices = {
    0, 1, 2, 0, 2, 3,  //
    4, 5, 6, 4, 6, 7,  //
    0, 1, 5, 0, 5, 4,  //
    1, 2, 6, 1, 6, 5,  //
    2, 3, 7, 2, 7, 6,  //
    3, 0, 4, 3, 4, 7,  //
  };

  static constexpr std::array<unsigned int, 24> cube_edge_indices = {
    0, 1, 1, 2, 2, 3, 3, 0,  //
    4, 5, 5, 6, 6, 7, 7, 4,  //
    0, 4, 1, 5, 2, 6, 3, 7,  //
  };

  // Create all GL objects and allocate buffers upfront
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &cube_vbo);
  glGenBuffers(1, &cube_ebo_faces);
  glGenBuffers(1, &cube_ebo_edges);
  glGenBuffers(1, &coords_vbo);

  glBindBuffer(GL_ARRAY_BUFFER, cube_vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(Eigen::Vector3f) * cube_vertices.size(), cube_vertices.data(), GL_STATIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cube_ebo_faces);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * cube_face_indices.size(), cube_face_indices.data(), GL_STATIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cube_ebo_edges);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * cube_edge_indices.size(), cube_edge_indices.data(), GL_STATIC_DRAW);

  // Per-instance voxel coordinates (Eigen::Vector3i is tightly packed as 3 ints)
  glBindBuffer(GL_ARRAY_BUFFER, coords_vbo);
  glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(sizeof(Eigen::Vector3i)) * num_voxels, voxel_coords, GL_STATIC_DRAW);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

VoxelMap::VoxelMap(const std::vector<Eigen::Vector3i>& voxel_coords, double resolution, const MeshRenderingOptions& options)
: VoxelMap(voxel_coords.data(), static_cast<int>(voxel_coords.size()), resolution, options) {}

VoxelMap::~VoxelMap() {
  if (coords_vbo) {
    glDeleteBuffers(1, &coords_vbo);
  }
  if (cube_ebo_faces) {
    glDeleteBuffers(1, &cube_ebo_faces);
  }
  if (cube_ebo_edges) {
    glDeleteBuffers(1, &cube_ebo_edges);
  }
  if (cube_vbo) {
    glDeleteBuffers(1, &cube_vbo);
  }
  if (vao) {
    glDeleteVertexArrays(1, &vao);
  }
}

void VoxelMap::draw(glk::GLSLShader& shader_) const {
  shader->use();

  shader_.copy_cached_uniforms(*shader);
  shader->set_uniform("voxel_resolution", resolution);

  glBindVertexArray(vao);

  GLint position_loc = shader->attrib("vert_position");
  glEnableVertexAttribArray(position_loc);

  GLint voxel_coord_loc = shader->attrib("voxel_coord");
  glEnableVertexAttribArray(voxel_coord_loc);

  // Per-vertex cube geometry
  glBindBuffer(GL_ARRAY_BUFFER, cube_vbo);
  glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);

  // Per-instance voxel coordinates (advance once per voxel)
  glBindBuffer(GL_ARRAY_BUFFER, coords_vbo);
  glVertexAttribIPointer(voxel_coord_loc, 3, GL_INT, sizeof(Eigen::Vector3i), 0);
  glVertexAttribDivisor(voxel_coord_loc, 1);

  // draw voxels
  if (options.draw_faces) {
    if (options.override_face_color_mode) {
      shader->set_uniform("color_mode", options.face_color_mode);
    }
    if (options.override_face_color) {
      shader->set_uniform("material_color", options.face_color);
    }

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cube_ebo_faces);
    glDrawElementsInstanced(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0, num_voxels);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  }

  // draw edges
  if (options.draw_edges) {
    if (options.override_edge_color_mode) {
      shader->set_uniform("color_mode", options.edge_color_mode);
    }
    if (options.override_edge_color) {
      shader->set_uniform("material_color", options.edge_color);
    }

    glLineWidth(options.edge_line_width);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cube_ebo_edges);
    glDrawElementsInstanced(GL_LINES, 24, GL_UNSIGNED_INT, 0, num_voxels);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  }

  glVertexAttribDivisor(voxel_coord_loc, 0);
  glDisableVertexAttribArray(voxel_coord_loc);
  glDisableVertexAttribArray(position_loc);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  shader_.use();
}

}  // namespace glk