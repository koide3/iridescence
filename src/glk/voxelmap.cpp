#include <glk/voxelmap.hpp>

#include <array>
#include <vector>
#include <cstddef>
#include <unordered_set>

namespace glk {

namespace {

// Hash/equality for using Eigen::Vector3i as an unordered_set key
struct Vector3iHash {
  std::size_t operator()(const Eigen::Vector3i& v) const {
    std::size_t h = std::hash<int>()(v.x());
    h ^= std::hash<int>()(v.y()) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<int>()(v.z()) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};

struct Vector3iEqual {
  bool operator()(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const { return a.x() == b.x() && a.y() == b.y() && a.z() == b.z(); }
};

}  // namespace

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

VoxelMap::VoxelMap(const std::vector<Eigen::Vector3i>& voxel_coords, double resolution, const VoxelMapOptions& options)
: VoxelMap(voxel_coords.data(), static_cast<int>(voxel_coords.size()), resolution, options) {}

VoxelMap::VoxelMap(const Eigen::Vector3i* voxel_coords, int num_voxels, double resolution, const VoxelMapOptions& options) : options(options) {
  this->num_voxels = num_voxels;
  num_voxel_indices = num_edge_indices = 0;
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

  // The 6 cube faces: triangle vertex offsets (2 triangles each) and the neighbor direction
  // a face points towards. A face is "exposed" (needs to be drawn) only if there is no voxel
  // in that neighbor direction.
  static constexpr int face_dirs[6][3] = {
    {0, 0, -1},  // bottom (z = 0)
    {0, 0, 1},   // top    (z = res)
    {0, -1, 0},  // y = 0
    {1, 0, 0},   // x = res
    {0, 1, 0},   // y = res
    {-1, 0, 0},  // x = 0
  };
  static constexpr unsigned int face_tris[6][6] = {
    {0, 2, 1, 0, 3, 2},  // bottom: wound outward (-Z) for consistent backface culling
    {4, 5, 6, 4, 6, 7},  //
    {0, 1, 5, 0, 5, 4},  //
    {1, 2, 6, 1, 6, 5},  //
    {2, 3, 7, 2, 7, 6},  //
    {3, 0, 4, 3, 4, 7},  //
  };

  // The 12 cube edges: vertex pairs and the two faces (indices into face_dirs) each edge borders.
  // An edge is drawn only if at least one of its two adjacent faces is exposed.
  static constexpr unsigned int edge_pairs[12][2] = {
    {0, 1},
    {1, 2},
    {2, 3},
    {3, 0},  //
    {4, 5},
    {5, 6},
    {6, 7},
    {7, 4},  //
    {0, 4},
    {1, 5},
    {2, 6},
    {3, 7},  //
  };
  static constexpr int edge_faces[12][2] = {
    {0, 2},
    {0, 3},
    {0, 4},
    {0, 5},  //
    {1, 2},
    {1, 3},
    {1, 4},
    {1, 5},  //
    {2, 5},
    {2, 3},
    {3, 4},
    {4, 5},  //
  };

  const bool cull = options.cull_hidden_faces;

  // Build a lookup set of occupied voxels for O(1) neighbor queries
  std::unordered_set<Eigen::Vector3i, Vector3iHash, Vector3iEqual> voxel_set;
  if (cull) {
    voxel_set.reserve(num_voxels * 2);
    for (int i = 0; i < num_voxels; i++) {
      voxel_set.insert(voxel_coords[i]);
    }
  }

  // Generate geometry on the CPU. When culling is disabled, every face/edge is emitted, which
  // reproduces the original full-cube geometry.
  std::vector<Eigen::Vector3f> vertices;
  std::vector<unsigned int> voxel_indices;
  std::vector<unsigned int> edge_indices;
  vertices.reserve(static_cast<std::size_t>(num_voxels) * 8);
  voxel_indices.reserve(static_cast<std::size_t>(num_voxels) * 36);
  edge_indices.reserve(static_cast<std::size_t>(num_voxels) * 24);

  for (int i = 0; i < num_voxels; i++) {
    const Eigen::Vector3i& coord = voxel_coords[i];

    bool exposed[6];
    bool any_exposed = false;
    for (int f = 0; f < 6; f++) {
      if (!cull) {
        exposed[f] = true;
      } else {
        const Eigen::Vector3i neighbor(coord.x() + face_dirs[f][0], coord.y() + face_dirs[f][1], coord.z() + face_dirs[f][2]);
        exposed[f] = voxel_set.find(neighbor) == voxel_set.end();
      }
      any_exposed = any_exposed || exposed[f];
    }

    // Fully enclosed voxel: no visible faces or edges, skip entirely
    if (!any_exposed) {
      continue;
    }

    const unsigned int base = static_cast<unsigned int>(vertices.size());
    const Eigen::Vector3f origin = coord.cast<float>() * res;
    for (int j = 0; j < 8; j++) {
      vertices.emplace_back(origin + vertex_offsets[j]);
    }

    for (int f = 0; f < 6; f++) {
      if (!exposed[f]) {
        continue;
      }
      for (int k = 0; k < 6; k++) {
        voxel_indices.push_back(base + face_tris[f][k]);
      }
    }

    for (int e = 0; e < 12; e++) {
      if (!exposed[edge_faces[e][0]] && !exposed[edge_faces[e][1]]) {
        continue;
      }
      edge_indices.push_back(base + edge_pairs[e][0]);
      edge_indices.push_back(base + edge_pairs[e][1]);
    }
  }

  num_voxel_indices = static_cast<int>(voxel_indices.size());
  num_edge_indices = static_cast<int>(edge_indices.size());

  if (vertices.empty()) {
    return;
  }

  // Create GL objects and upload the generated geometry
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(sizeof(Eigen::Vector3f)) * vertices.size(), vertices.data(), GL_STATIC_DRAW);

  if (!voxel_indices.empty()) {
    glGenBuffers(1, &ebo_voxels);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_voxels);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(sizeof(unsigned int)) * voxel_indices.size(), voxel_indices.data(), GL_STATIC_DRAW);
  }

  if (!edge_indices.empty()) {
    glGenBuffers(1, &ebo_edges);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_edges);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(sizeof(unsigned int)) * edge_indices.size(), edge_indices.data(), GL_STATIC_DRAW);
  }

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
  if (!vao) {
    return;
  }

  glBindVertexArray(vao);

  GLint position_loc = shader.attrib("vert_position");
  glEnableVertexAttribArray(position_loc);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);

  // draw voxels
  if (options.draw_voxels && ebo_voxels && num_voxel_indices > 0) {
    if (options.override_voxel_color_mode) {
      shader.set_uniform("color_mode", options.voxel_color_mode);
    }
    if (options.override_voxel_color) {
      shader.set_uniform("material_color", options.voxel_color);
    }

    // Cube faces are wound consistently outward (CCW), so back faces are hidden and can be
    // culled. This roughly halves the rasterized triangles / fragment work, which is especially
    // beneficial for opaque voxels (the opaque render pass does not enable culling globally).
    const GLboolean cull_was_enabled = glIsEnabled(GL_CULL_FACE);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_voxels);
    glDrawElements(GL_TRIANGLES, num_voxel_indices, GL_UNSIGNED_INT, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    if (!cull_was_enabled) {
      glDisable(GL_CULL_FACE);
    }
  }

  // draw edges
  if (options.draw_edges && ebo_edges && num_edge_indices > 0) {
    if (options.override_edge_color_mode) {
      shader.set_uniform("color_mode", options.edge_color_mode);
    }
    if (options.override_edge_color) {
      shader.set_uniform("material_color", options.edge_color);
    }

    glLineWidth(options.edge_line_width);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_edges);
    glDrawElements(GL_LINES, num_edge_indices, GL_UNSIGNED_INT, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  }

  glDisableVertexAttribArray(position_loc);
}

}  // namespace glk