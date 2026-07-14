#ifndef GLK_VOXELMAP_HPP
#define GLK_VOXELMAP_HPP

#include <memory>
#include <vector>
#include <cstring>
#include <iostream>

#include <glk/drawable.hpp>
#include <glk/mesh_rendering_options.hpp>

namespace glk {

struct VoxelMapOptions {
public:
  VoxelMapOptions()
  : draw_voxels(true),
    override_voxel_color_mode(false),
    override_voxel_color(false),
    voxel_color_mode(0),
    voxel_color(1.0f, 1.0f, 1.0f, 1.0f),
    draw_edges(true),
    override_edge_color_mode(false),
    override_edge_color(true),
    edge_color_mode(0),
    edge_color(1.0f, 1.0f, 1.0f, 1.0f),
    edge_line_width(2.0) {
    std::cerr << "warning: VoxelMapOptions is deprecated. Use MeshRenderingOptions instead." << std::endl;
  }

  MeshRenderingOptions to_mesh_rendering_options() const;

public:
  void set_voxel_alpha(float alpha);
  void set_voxel_color(const Eigen::Vector4f& color);

  void set_edge_alpha(float alpha);
  void set_edge_color(const Eigen::Vector4f& color);

public:
  bool draw_voxels;
  bool override_voxel_color_mode;  // if true, use voxel_color_mode instead of drawable's shader settings
  bool override_voxel_color;       // if true, use voxel_color instead of drawable's shader settings
  int voxel_color_mode;            // 0: rainbow, 1: flat_color, 2: vertex_color
  Eigen::Vector4f voxel_color;     // used when voxel_color_mode == 1

  bool draw_edges;
  bool override_edge_color_mode;  // if true, use edge_color_mode instead of drawable's shader settings
  bool override_edge_color;       // if true, use edge_color instead of drawable's shader settings
  int edge_color_mode;            // 0: rainbow, 1: flat_color, 2: vertex_color
  Eigen::Vector4f edge_color;     // used when edge_color_mode == 1

  double edge_line_width;
};

class VoxelMap : public Drawable {
public:
  VoxelMap(const Eigen::Vector3i* voxel_coords, int num_voxels, double resolution, const MeshRenderingOptions& options = MeshRenderingOptions());
  VoxelMap(const std::vector<Eigen::Vector3i>& voxel_coords, double resolution, const MeshRenderingOptions& options = MeshRenderingOptions());

  // Deprecated constructor
  VoxelMap(const Eigen::Vector3i* voxel_coords, int num_voxels, double resolution, const VoxelMapOptions& options);

  virtual ~VoxelMap();

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  VoxelMap(const VoxelMap&);
  VoxelMap& operator=(const VoxelMap&);

private:
  static std::shared_ptr<glk::GLSLShader> shader;

  MeshRenderingOptions options;
  float resolution;
  int num_voxels;

  GLuint vao;
  GLuint cube_vbo;
  GLuint cube_ebo_faces;
  GLuint cube_ebo_edges;
  GLuint coords_vbo;
};

}  // namespace glk

#endif
