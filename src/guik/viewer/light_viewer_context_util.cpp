#include <guik/viewer/light_viewer_context.hpp>

#include <glk/thin_lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>

namespace guik {

// PointCloudBuffer
std::shared_ptr<glk::PointCloudBuffer>
LightViewerContext::update_points(const std::string& name, const float* data, int stride, int num_points, const ShaderSetting& shader_setting) {
  auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(data, stride, num_points);
  update_drawable(name, cloud_buffer, shader_setting);
  return cloud_buffer;
}

// ThinLines
void LightViewerContext::update_thin_lines(
  const std::string& name,
  const float* vertices,
  const float* colors,
  int num_vertices,
  const unsigned int* indices,
  int num_indices,
  bool line_strip,
  const ShaderSetting& shader_setting) {
  update_drawable(name, std::make_shared<glk::ThinLines>(vertices, colors, num_vertices, indices, num_indices, line_strip), shader_setting);
}

// Primitives
void LightViewerContext::update_icosahedron(const std::string& name, const ShaderSetting& shader_setting) {
  update_drawable(name, glk::Primitives::icosahedron(), shader_setting);
}
void LightViewerContext::update_sphere(const std::string& name, const ShaderSetting& shader_setting) {
  update_drawable(name, glk::Primitives::sphere(), shader_setting);
}
void LightViewerContext::update_cube(const std::string& name, const ShaderSetting& shader_setting) {
  update_drawable(name, glk::Primitives::cube(), shader_setting);
}
void LightViewerContext::update_cone(const std::string& name, const ShaderSetting& shader_setting) {
  update_drawable(name, glk::Primitives::cone(), shader_setting);
}
void LightViewerContext::update_coord(const std::string& name, const ShaderSetting& shader_setting) {
  update_drawable(name, glk::Primitives::coordinate_system(), shader_setting);
}
void LightViewerContext::update_wire_icosahedron(const std::string& name, const ShaderSetting& shader_setting) {
  update_drawable(name, glk::Primitives::wire_icosahedron(), shader_setting);
}
void LightViewerContext::update_wire_sphere(const std::string& name, const ShaderSetting& shader_setting) {
  update_drawable(name, glk::Primitives::wire_sphere(), shader_setting);
}
void LightViewerContext::update_wire_cube(const std::string& name, const ShaderSetting& shader_setting) {
  update_drawable(name, glk::Primitives::wire_cube(), shader_setting);
}
void LightViewerContext::update_wire_cone(const std::string& name, const ShaderSetting& shader_setting) {
  update_drawable(name, glk::Primitives::wire_cone(), shader_setting);
}
void LightViewerContext::update_wire_frustum(const std::string& name, const ShaderSetting& shader_setting) {
  update_drawable(name, glk::Primitives::wire_frustum(), shader_setting);
}

}  // namespace guik