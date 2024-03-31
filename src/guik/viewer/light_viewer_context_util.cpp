#include <guik/viewer/light_viewer_context.hpp>

#include <glk/thin_lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/normal_distributions.hpp>
#include <glk/primitives/primitives.hpp>

namespace guik {

// PointCloudBuffer
std::shared_ptr<glk::PointCloudBuffer>
LightViewerContext::update_points(const std::string& name, const float* data, int stride, int num_points, const ShaderSetting& shader_setting) {
  auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(data, stride, num_points);
  update_drawable(name, cloud_buffer, shader_setting);
  return cloud_buffer;
}

template <typename Scalar, int Dim>
void LightViewerContext::update_normal_dists(
  const std::string& name,
  const Eigen::Matrix<Scalar, Dim, 1>* points,
  const Eigen::Matrix<Scalar, Dim, Dim>* covs,
  int num_points,
  float scale,
  const ShaderSetting& shader_setting) {
  update_drawable(name, std::make_shared<glk::NormalDistributions>(points, covs, num_points, scale), shader_setting);
}

template void LightViewerContext::update_normal_dists(const std::string&, const Eigen::Matrix<float, 3, 1>*, const Eigen::Matrix<float, 3, 3>*, int, float, const ShaderSetting&);
template void LightViewerContext::update_normal_dists(const std::string&, const Eigen::Matrix<float, 4, 1>*, const Eigen::Matrix<float, 4, 4>*, int, float, const ShaderSetting&);
template void LightViewerContext::update_normal_dists(const std::string&, const Eigen::Matrix<double, 3, 1>*, const Eigen::Matrix<double, 3, 3>*, int, float, const ShaderSetting&);
template void LightViewerContext::update_normal_dists(const std::string&, const Eigen::Matrix<double, 4, 1>*, const Eigen::Matrix<double, 4, 4>*, int, float, const ShaderSetting&);

// ThinLines
std::shared_ptr<glk::ThinLines> LightViewerContext::update_thin_lines(
  const std::string& name,
  const float* vertices,
  const float* colors,
  int num_vertices,
  const unsigned int* indices,
  int num_indices,
  bool line_strip,
  const ShaderSetting& shader_setting) {
  auto thin_lines = std::make_shared<glk::ThinLines>(vertices, colors, num_vertices, indices, num_indices, line_strip);
  update_drawable(name, thin_lines, shader_setting);
  return thin_lines;
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
void LightViewerContext::update_frustum(const std::string& name, const ShaderSetting& shader_setting) {
  update_drawable(name, glk::Primitives::frustum(), shader_setting);
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