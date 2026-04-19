#ifndef GLK_VOXELMAP_HPP
#define GLK_VOXELMAP_HPP

#include <memory>
#include <glk/drawable.hpp>

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
    edge_line_width(2.0) {}

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
  VoxelMap(const Eigen::Vector3i* voxel_coords, int num_voxels, double resolution, const VoxelMapOptions& options = VoxelMapOptions());

  virtual ~VoxelMap();

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  VoxelMap(const VoxelMap&);
  VoxelMap& operator=(const VoxelMap&);

private:
  VoxelMapOptions options;
  int num_voxels;

  GLuint vao;
  GLuint vbo;
  GLuint ebo_voxels;
  GLuint ebo_edges;
};

}  // namespace glk

#endif
