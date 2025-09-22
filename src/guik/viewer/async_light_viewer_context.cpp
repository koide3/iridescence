#include <guik/viewer/async_light_viewer_context.hpp>

#include <glk/thin_lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/normal_distributions.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace guik {

AsyncLightViewerContext::AsyncLightViewerContext() : context(nullptr) {}

AsyncLightViewerContext::AsyncLightViewerContext(LightViewerContext* context) : context(context) {}

AsyncLightViewerContext::~AsyncLightViewerContext() {}

void AsyncLightViewerContext::clear() {
  guik::viewer()->invoke([=] { context->clear(); });
}

void AsyncLightViewerContext::clear_text() {
  guik::viewer()->invoke([=] { context->clear_text(); });
}

void AsyncLightViewerContext::append_text(const std::string& text) {
  guik::viewer()->invoke([=] { context->append_text(text); });
}

void AsyncLightViewerContext::register_ui_callback(const std::string& name, const std::function<void()>& callback) {
  guik::viewer()->invoke([=] { context->register_ui_callback(name, callback); });
}

void AsyncLightViewerContext::remove_ui_callback(const std::string& name) {
  register_ui_callback(name, 0);
}

void AsyncLightViewerContext::set_draw_xy_grid(bool draw_xy_grid) {
  guik::viewer()->invoke([=] { context->set_draw_xy_grid(draw_xy_grid); });
}

void AsyncLightViewerContext::set_colormap(glk::COLORMAP colormap) {
  guik::viewer()->invoke([=] { context->set_colormap(colormap); });
}

void AsyncLightViewerContext::set_point_shape(float point_size, bool metric, bool circle) {
  guik::viewer()->invoke([=] { context->set_point_shape(point_size, metric, circle); });
}

void AsyncLightViewerContext::clear_drawables() {
  guik::viewer()->invoke([=] { context->clear_drawables(); });
}
void AsyncLightViewerContext::clear_drawables(const std::function<bool(const std::string&)>& fn) {
  guik::viewer()->invoke([=] { context->clear_drawables(fn); });
}

void AsyncLightViewerContext::remove_drawable(const std::string& name) {
  guik::viewer()->invoke([=] { context->remove_drawable(name); });
}

void AsyncLightViewerContext::remove_drawable(const std::regex& regex) {
  guik::viewer()->invoke([=] { context->remove_drawable(regex); });
}

void AsyncLightViewerContext::save_camera_settings(const std::string& path) {
  guik::viewer()->invoke([=] { context->save_camera_settings(path); });
}

void AsyncLightViewerContext::load_camera_settings(const std::string& path) {
  guik::viewer()->invoke([=] { context->load_camera_settings(path); });
}

void AsyncLightViewerContext::save_color_buffer(const std::string& filename) {
  guik::viewer()->invoke([=] { context->save_color_buffer(filename); });
}

void AsyncLightViewerContext::save_depth_buffer(const std::string& filename, bool real_scale) {
  guik::viewer()->invoke([=] { context->save_depth_buffer(filename, real_scale); });
}

void AsyncLightViewerContext::reset_center() {
  guik::viewer()->invoke([=] { context->reset_center(); });
}

void AsyncLightViewerContext::lookat(const Eigen::Vector3f& pt) {
  guik::viewer()->invoke([=] { context->lookat(pt); });
}

void AsyncLightViewerContext::use_orbit_camera_control(double distance, double theta, double phi) {
  guik::viewer()->invoke([=] { context->use_orbit_camera_control(distance, theta, phi); });
}

void AsyncLightViewerContext::use_orbit_camera_control_xz(double distance, double theta, double phi) {
  guik::viewer()->invoke([=] { context->use_orbit_camera_control_xz(distance, theta, phi); });
}

void AsyncLightViewerContext::use_topdown_camera_control(double distance, double theta) {
  guik::viewer()->invoke([=] { context->use_topdown_camera_control(distance, theta); });
}

void AsyncLightViewerContext::use_arcball_camera_control(double distance, double theta, double phi) {
  guik::viewer()->invoke([=] { context->use_arcball_camera_control(distance, theta, phi); });
}

void AsyncLightViewerContext::use_fps_camera_control(double fovy_deg) {
  guik::viewer()->invoke([=] { context->use_fps_camera_control(fovy_deg); });
}

void AsyncLightViewerContext::update_drawable_setting(const std::string& name, const ShaderSetting& shader_setting) {
  guik::viewer()->invoke([=] {
    auto drawable = context->find_drawable(name);
    if (!drawable.first || !drawable.second) {
      std::cerr << "warning: drawable not found (name=" << name << ")" << std::endl;
      return;
    }
    *drawable.first = shader_setting;
  });
}

// PointCloudBuffer
void AsyncLightViewerContext::update_points(const std::string& name, const float* data, int stride, int num_points, const ShaderSetting& shader_setting) {
  std::vector<float> buffer(data, data + stride / sizeof(float) * num_points);

  guik::viewer()->invoke([=] {
    auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(buffer.data(), stride, num_points);
    context->update_drawable(name, cloud_buffer, shader_setting);
  });
}

void AsyncLightViewerContext::update_points(
  const std::string& name,
  const float* vertices,
  int vertex_stride,
  const float* colors,
  int color_stride,
  int num_points,
  const ShaderSetting& shader_setting) {
  std::vector<float> vertex_buffer(vertices, vertices + vertex_stride / sizeof(float) * num_points);
  std::vector<float> color_buffer(colors, colors + color_stride / sizeof(float) * num_points);

  guik::viewer()->invoke([=] {
    auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(vertex_buffer.data(), vertex_stride, num_points);
    cloud_buffer->add_color(color_buffer.data(), color_stride, num_points);
    context->update_drawable(name, cloud_buffer, shader_setting);
  });
}

template <typename Scalar, int Dim>
void AsyncLightViewerContext::update_normal_dists(
  const std::string& name,
  const Eigen::Matrix<Scalar, Dim, 1>* points,
  const Eigen::Matrix<Scalar, Dim, Dim>* covs,
  int num_points,
  float scale,
  const ShaderSetting& shader_setting) {
  std::vector<Eigen::Matrix<Scalar, Dim, 1>> points_buffer(points, points + num_points);
  std::vector<Eigen::Matrix<Scalar, Dim, Dim>> covs_buffer(covs, covs + num_points);
  guik::viewer()->invoke(
    [=] { context->update_drawable(name, std::make_shared<glk::NormalDistributions>(points_buffer.data(), covs_buffer.data(), num_points, scale), shader_setting); });
}

template void
AsyncLightViewerContext::update_normal_dists(const std::string&, const Eigen::Matrix<float, 3, 1>*, const Eigen::Matrix<float, 3, 3>*, int, float, const ShaderSetting&);
template void
AsyncLightViewerContext::update_normal_dists(const std::string&, const Eigen::Matrix<float, 4, 1>*, const Eigen::Matrix<float, 4, 4>*, int, float, const ShaderSetting&);
template void
AsyncLightViewerContext::update_normal_dists(const std::string&, const Eigen::Matrix<double, 3, 1>*, const Eigen::Matrix<double, 3, 3>*, int, float, const ShaderSetting&);
template void
AsyncLightViewerContext::update_normal_dists(const std::string&, const Eigen::Matrix<double, 4, 1>*, const Eigen::Matrix<double, 4, 4>*, int, float, const ShaderSetting&);

// ThinLines
void AsyncLightViewerContext::update_thin_lines(
  const std::string& name,
  const float* vertices,
  const float* colors,
  int num_vertices,
  const unsigned int* indices,
  int num_indices,
  bool line_strip,
  const ShaderSetting& shader_setting) {
  //
  std::vector<float> vertices_buffer(vertices, vertices + 3 * num_vertices);
  std::vector<float> colors_buffer;
  if (colors) {
    colors_buffer.assign(colors, colors + 4 * num_vertices);
  }
  std::vector<unsigned int> indices_buffer(indices, indices + num_indices);

  guik::viewer()->invoke([=] {
    auto thin_lines = std::make_shared<glk::ThinLines>(vertices_buffer.data(), colors_buffer.data(), num_vertices, indices_buffer.data(), num_indices, line_strip);
    context->update_drawable(name, thin_lines, shader_setting);
  });
}

// Primitives
void AsyncLightViewerContext::update_icosahedron(const std::string& name, const ShaderSetting& shader_setting) {
  guik::viewer()->invoke([=] { context->update_drawable(name, glk::Primitives::icosahedron(), shader_setting); });
}
void AsyncLightViewerContext::update_sphere(const std::string& name, const ShaderSetting& shader_setting) {
  guik::viewer()->invoke([=] { context->update_drawable(name, glk::Primitives::sphere(), shader_setting); });
}
void AsyncLightViewerContext::update_cube(const std::string& name, const ShaderSetting& shader_setting) {
  guik::viewer()->invoke([=] { context->update_drawable(name, glk::Primitives::cube(), shader_setting); });
}
void AsyncLightViewerContext::update_cone(const std::string& name, const ShaderSetting& shader_setting) {
  guik::viewer()->invoke([=] { context->update_drawable(name, glk::Primitives::cone(), shader_setting); });
}
void AsyncLightViewerContext::update_frustum(const std::string& name, const ShaderSetting& shader_setting) {
  guik::viewer()->invoke([=] { context->update_drawable(name, glk::Primitives::frustum(), shader_setting); });
}
void AsyncLightViewerContext::update_coord(const std::string& name, const ShaderSetting& shader_setting) {
  guik::viewer()->invoke([=] { context->update_drawable(name, glk::Primitives::coordinate_system(), shader_setting); });
}
void AsyncLightViewerContext::update_wire_icosahedron(const std::string& name, const ShaderSetting& shader_setting) {
  guik::viewer()->invoke([=] { context->update_drawable(name, glk::Primitives::wire_icosahedron(), shader_setting); });
}
void AsyncLightViewerContext::update_wire_sphere(const std::string& name, const ShaderSetting& shader_setting) {
  guik::viewer()->invoke([=] { context->update_drawable(name, glk::Primitives::wire_sphere(), shader_setting); });
}
void AsyncLightViewerContext::update_wire_cube(const std::string& name, const ShaderSetting& shader_setting) {
  guik::viewer()->invoke([=] { context->update_drawable(name, glk::Primitives::wire_cube(), shader_setting); });
}
void AsyncLightViewerContext::update_wire_cone(const std::string& name, const ShaderSetting& shader_setting) {
  guik::viewer()->invoke([=] { context->update_drawable(name, glk::Primitives::wire_cone(), shader_setting); });
}
void AsyncLightViewerContext::update_wire_frustum(const std::string& name, const ShaderSetting& shader_setting) {
  guik::viewer()->invoke([=] { context->update_drawable(name, glk::Primitives::wire_frustum(), shader_setting); });
}

}  // namespace guik
