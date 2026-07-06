#ifndef GLK_MESH_RENDERING_OPTIONS_HPP
#define GLK_MESH_RENDERING_OPTIONS_HPP

#include <memory>
#include <glk/drawable.hpp>

namespace glk {

/**
 * @brief Options for rendering meshes.
 */
struct MeshRenderingOptions {
public:
  MeshRenderingOptions()
  : draw_faces(true),
    override_face_color_mode(false),
    override_face_color(false),
    face_color_mode(0),
    face_color(1.0f, 1.0f, 1.0f, 1.0f),
    draw_edges(false),
    override_edge_color_mode(false),
    override_edge_color(true),
    edge_color_mode(0),
    edge_color(1.0f, 1.0f, 1.0f, 1.0f),
    edge_line_width(2.0) {}

public:
  MeshRenderingOptions& set_face_alpha(float alpha);
  MeshRenderingOptions& set_face_color(const Eigen::Vector4f& color);

  MeshRenderingOptions& set_edge_alpha(float alpha);
  MeshRenderingOptions& set_edge_color(const Eigen::Vector4f& color);

public:
  bool draw_faces;
  bool override_face_color_mode;  // if true, use face_color_mode instead of drawable's shader settings
  bool override_face_color;       // if true, use face_color instead of drawable's shader settings
  int face_color_mode;            // 0: rainbow, 1: flat_color, 2: vertex_color
  Eigen::Vector4f face_color;     // used when face_color_mode == 1

  bool draw_edges;
  bool override_edge_color_mode;  // if true, use edge_color_mode instead of drawable's shader settings
  bool override_edge_color;       // if true, use edge_color instead of drawable's shader settings
  int edge_color_mode;            // 0: rainbow, 1: flat_color, 2: vertex_color
  Eigen::Vector4f edge_color;     // used when edge_color_mode == 1

  double edge_line_width;
};

}  // namespace glk

#endif