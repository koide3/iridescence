#include <glk/mesh_rendering_options.hpp>

namespace glk {

MeshRenderingOptions& MeshRenderingOptions::set_face_alpha(float alpha) {
  draw_faces = true;
  override_face_color = true;
  face_color.w() = alpha;

  return *this;
}

MeshRenderingOptions& MeshRenderingOptions::set_face_color(const Eigen::Vector4f& color) {
  draw_faces = true;
  override_face_color_mode = true;
  override_face_color = true;

  face_color_mode = 1;  // flat_color
  face_color = color;

  return *this;
}

MeshRenderingOptions& MeshRenderingOptions::set_edge_alpha(float alpha) {
  draw_edges = true;
  override_edge_color = true;
  edge_color.w() = alpha;

  return *this;
}

MeshRenderingOptions& MeshRenderingOptions::set_edge_color(const Eigen::Vector4f& color) {
  draw_edges = true;
  override_edge_color_mode = true;
  override_edge_color = true;

  edge_color_mode = 1;  // flat_color
  edge_color = color;

  return *this;
}

}  // namespace glk