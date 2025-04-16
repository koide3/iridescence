#ifndef GUIK_LIGHT_VIEWER_CONTEXT_HPP
#define GUIK_LIGHT_VIEWER_CONTEXT_HPP

#include <mutex>
#include <deque>
#include <regex>
#include <memory>
#include <optional>
#include <unordered_map>

#include <glk/drawable.hpp>
#include <glk/colormap.hpp>
#include <glk/type_conversion.hpp>
#include <guik/gl_canvas.hpp>
#include <guik/camera/camera_control.hpp>
#include <guik/camera/projection_control.hpp>
#include <guik/viewer/shader_setting.hpp>
#include <guik/viewer/anonymous.hpp>

namespace glk {
class ThinLines;
class PointCloudBuffer;
}  // namespace glk

namespace guik {

class AsyncLightViewerContext;
class OrbitCameraControlXY;
class OrbitCameraControlXZ;
class TopDownCameraControl;
class ArcBallCameraControl;
class FPSCameraControl;

class LightViewerContext {
public:
  LightViewerContext(const std::string& context_name);
  virtual ~LightViewerContext();

  void draw_ui();
  void draw_gl();

  bool init_canvas(const Eigen::Vector2i& size);

  void set_size(const Eigen::Vector2i& size);
  void set_clear_color(const Eigen::Vector4f& color);

  void set_pos(const Eigen::Vector2i& pos, ImGuiCond cond = ImGuiCond_FirstUseEver, ImGuiWindowFlags = 0);
  void show();
  void hide();

  virtual void clear();
  virtual void clear_text();
  virtual void append_text(const std::string& text);
  virtual void register_ui_callback(const std::string& name, const std::function<void()>& callback = 0);
  void remove_ui_callback(const std::string& name);

  guik::ShaderSetting& shader_setting() { return global_shader_setting; }
  const guik::ShaderSetting& shader_setting() const { return global_shader_setting; }

  void disable_xy_grid() { set_draw_xy_grid(false); }
  void enable_xy_grid() { set_draw_xy_grid(true); }
  void set_draw_xy_grid(bool draw_xy_grid);
  void set_colormap(glk::COLORMAP colormap);
  void set_screen_effect(const std::shared_ptr<glk::ScreenEffect>& effect);
  const std::shared_ptr<glk::ScreenEffect>& get_screen_effect() const;
  void set_bg_texture(const std::shared_ptr<glk::Texture>& bg_texture);

  void enable_decimal_rendering();
  void enable_normal_buffer();
  void enable_info_buffer();
  void enable_partial_rendering(double clear_thresh = 1e-6);
  void disable_partial_rendering();

  bool normal_buffer_enabled() const;
  bool info_buffer_enabled() const;
  bool partial_rendering_enabled() const;

  const glk::Texture& color_buffer() const;
  const glk::Texture& depth_buffer() const;
  const glk::Texture& normal_buffer() const;
  const glk::Texture& info_buffer() const;
  const glk::Texture& dynamic_flag_buffer() const;

  void clear_drawables();
  void clear_drawables(const std::function<bool(const std::string&)>& fn);

  std::unordered_map<std::string, std::pair<ShaderSetting::Ptr, glk::Drawable::ConstPtr>>& get_drawables();

  std::pair<ShaderSetting::Ptr, glk::Drawable::ConstPtr> find_drawable(const std::string& name);
  void remove_drawable(const std::string& name);
  void remove_drawable(const std::regex& regex);
  void update_drawable(const std::string& name, const glk::Drawable::ConstPtr& drawable, const ShaderSetting& shader_setting = ShaderSetting());

  void clear_drawable_filters();
  void register_drawable_filter(const std::string& filter_name, const std::function<bool(const std::string&)>& filter = 0);
  void remove_drawable_filter(const std::string& filter_name);

  void clear_partial_rendering();

  const std::shared_ptr<CameraControl>& get_camera_control() const;
  const std::shared_ptr<ProjectionControl>& get_projection_control() const;
  void set_camera_control(const std::shared_ptr<CameraControl>& camera_control);
  void set_projection_control(const std::shared_ptr<ProjectionControl>& projection_control);

  void reset_center();
  void lookat(const Eigen::Vector3f& pt);
  template <typename Vector>
  void lookat(const Vector& pt) {
    const auto ptf = pt.eval().template cast<float>();
    lookat(ptf);
  }

  std::shared_ptr<OrbitCameraControlXY> use_orbit_camera_control(double distance = 80.0, double theta = 0.0, double phi = -60.0f * M_PI / 180.0f);
  std::shared_ptr<OrbitCameraControlXZ> use_orbit_camera_control_xz(double distance = 80.0, double theta = 0.0, double phi = 0.0);
  std::shared_ptr<TopDownCameraControl> use_topdown_camera_control(double distance = 80.0, double theta = 0.0);
  std::shared_ptr<ArcBallCameraControl> use_arcball_camera_control(double distance = 80.0, double theta = 0.0, double phi = -60.0f * M_PI / 180.0f);
  std::shared_ptr<FPSCameraControl> use_fps_camera_control(double fovy_deg = 60.0);

  guik::GLCanvas& get_canvas();
  Eigen::Vector2i canvas_tl() const { return canvas_rect_min; }
  Eigen::Vector2i canvas_br() const { return canvas_rect_max; }
  Eigen::Vector2i canvas_size() const { return canvas->size; }
  Eigen::Matrix4f view_matrix() const { return canvas->camera_control->view_matrix(); }
  Eigen::Matrix4f projection_matrix() const { return canvas->projection_control->projection_matrix(); }

  Eigen::Vector4i pick_info(const Eigen::Vector2i& p, int window = 2) const;
  float pick_depth(const Eigen::Vector2i& p, int window = 2) const;
  Eigen::Vector3f unproject(const Eigen::Vector2i& p, float depth) const;
  std::optional<Eigen::Vector3f> pick_point(int button = 0, int window = 2, Eigen::Vector4i* info = nullptr) const;

  // Async
  AsyncLightViewerContext async();

  // Utility methods to directly create and update drawables
  // PointCloudBuffer
  std::shared_ptr<glk::PointCloudBuffer> update_points(const std::string& name, const float* data, int stride, int num_points, const ShaderSetting& shader_setting);
  template <typename Scalar, int Dim>
  std::shared_ptr<glk::PointCloudBuffer> update_points(const std::string& name, const Eigen::Matrix<Scalar, Dim, 1>* points, int num_points, const ShaderSetting& shader_setting);

  template <typename Scalar, int Dim, typename Allocator>
  std::shared_ptr<glk::PointCloudBuffer>
  update_points(const std::string& name, const std::vector<Eigen::Matrix<Scalar, Dim, 1>, Allocator>& points, const ShaderSetting& shader_setting);

  // NormalDistributions
  template <typename Scalar, int Dim>
  void update_normal_dists(
    const std::string& name,
    const Eigen::Matrix<Scalar, Dim, 1>* points,
    const Eigen::Matrix<Scalar, Dim, Dim>* covs,
    int num_points,
    float scale,
    const ShaderSetting& shader_setting);

  template <typename Scalar, int Dim, typename Alloc1, typename Alloc2>
  void update_normal_dists(
    const std::string& name,
    const std::vector<Eigen::Matrix<Scalar, Dim, 1>, Alloc1> points,
    const std::vector<Eigen::Matrix<Scalar, Dim, Dim>, Alloc2> covs,
    float scale,
    const ShaderSetting& shader_setting);

  // ThinLines
  std::shared_ptr<glk::ThinLines> update_thin_lines(
    const std::string& name,
    const float* vertices,
    const float* colors,
    int num_vertices,
    const unsigned int* indices,
    int num_indices,
    bool line_strip,
    const ShaderSetting& shader_setting);

  template <typename Scalar, int Dim>
  std::shared_ptr<glk::ThinLines>
  update_thin_lines(const std::string& name, const Eigen::Matrix<Scalar, Dim, 1>* points, int num_points, bool line_strip, const ShaderSetting& shader_setting);

  template <typename ScalarV, int DimV, typename ScalarC, int DimC>
  std::shared_ptr<glk::ThinLines> update_thin_lines(
    const std::string& name,
    const Eigen::Matrix<ScalarV, DimV, 1>* points,
    const Eigen::Matrix<ScalarC, DimC, 1>* colors,
    int num_points,
    bool line_strip,
    const ShaderSetting& shader_setting);

  template <typename ScalarV, int DimV, typename ScalarC, int DimC>
  std::shared_ptr<glk::ThinLines> update_thin_lines(
    const std::string& name,
    const Eigen::Matrix<ScalarV, DimV, 1>* points,
    const Eigen::Matrix<ScalarC, DimC, 1>* colors,
    int num_points,
    const unsigned int* indices,
    int num_indices,
    bool line_strip,
    const ShaderSetting& shader_setting);

  template <typename Point, typename Alloc>
  std::shared_ptr<glk::ThinLines> update_thin_lines(const std::string& name, const std::vector<Point, Alloc>& points, bool line_strip, const ShaderSetting& shader_setting);

  template <typename Point, typename Alloc>
  std::shared_ptr<glk::ThinLines> update_thin_lines(
    const std::string& name,
    const std::vector<Point, Alloc>& points,
    const std::vector<unsigned int>& indices,
    bool line_strip,
    const ShaderSetting& shader_setting);

  template <typename Point, typename AllocP, typename Color, typename AllocC>
  std::shared_ptr<glk::ThinLines> update_thin_lines(
    const std::string& name,
    const std::vector<Point, AllocP>& points,
    const std::vector<Color, AllocC>& colors,
    bool line_strip,
    const ShaderSetting& shader_setting);

  template <typename Point, typename AllocP, typename Color, typename AllocC>
  std::shared_ptr<glk::ThinLines> update_thin_lines(
    const std::string& name,
    const std::vector<Point, AllocP>& points,
    const std::vector<Color, AllocC>& colors,
    const std::vector<unsigned int>& indices,
    bool line_strip,
    const ShaderSetting& shader_setting);

  // Primitives
  void update_icosahedron(const std::string& name, const ShaderSetting& shader_setting);
  void update_sphere(const std::string& name, const ShaderSetting& shader_setting);
  void update_cube(const std::string& name, const ShaderSetting& shader_setting);
  void update_cone(const std::string& name, const ShaderSetting& shader_setting);
  void update_frustum(const std::string& name, const ShaderSetting& shader_setting);
  void update_coord(const std::string& name, const ShaderSetting& shader_setting);

  void update_wire_icosahedron(const std::string& name, const ShaderSetting& shader_setting);
  void update_wire_sphere(const std::string& name, const ShaderSetting& shader_setting);
  void update_wire_cube(const std::string& name, const ShaderSetting& shader_setting);
  void update_wire_cone(const std::string& name, const ShaderSetting& shader_setting);
  void update_wire_frustum(const std::string& name, const ShaderSetting& shader_setting);

protected:
  std::string context_name;
  Eigen::Vector2i canvas_rect_min;
  Eigen::Vector2i canvas_rect_max;
  std::unique_ptr<guik::GLCanvas> canvas;
  guik::ShaderSetting global_shader_setting;

  bool show_window;
  bool draw_xy_grid;
  bool decimal_rendering;

  Eigen::Matrix4f last_projection_view_matrix;

  std::unordered_map<std::string, std::function<bool(const std::string&)>> drawable_filters;
  std::unordered_map<std::string, std::pair<ShaderSetting::Ptr, glk::Drawable::ConstPtr>> drawables;

  std::mutex sub_texts_mutex;
  std::deque<std::string> sub_texts;
  std::unordered_map<std::string, std::function<void()>> sub_ui_callbacks;
};

// template methods
// PointCloud
template <typename Scalar, int Dim>
std::shared_ptr<glk::PointCloudBuffer>
LightViewerContext::update_points(const std::string& name, const Eigen::Matrix<Scalar, Dim, 1>* points, int num_points, const ShaderSetting& shader_setting) {
  if constexpr (std::is_same<Scalar, float>::value) {
    return update_points(name, reinterpret_cast<const float*>(points), sizeof(float) * Dim, num_points, shader_setting);
  } else {
    const auto points_3f = glk::convert_to_vector<float, 3, 1>(points, num_points);
    return update_points(name, points_3f.data(), num_points, shader_setting);
  }
}

template <typename Scalar, int Dim, typename Allocator>
std::shared_ptr<glk::PointCloudBuffer>
LightViewerContext::update_points(const std::string& name, const std::vector<Eigen::Matrix<Scalar, Dim, 1>, Allocator>& points, const ShaderSetting& shader_setting) {
  return update_points(name, points.data(), points.size(), shader_setting);
}

// NormalDistributions
template <typename Scalar, int Dim, typename Alloc1, typename Alloc2>
void LightViewerContext::update_normal_dists(
  const std::string& name,
  const std::vector<Eigen::Matrix<Scalar, Dim, 1>, Alloc1> points,
  const std::vector<Eigen::Matrix<Scalar, Dim, Dim>, Alloc2> covs,
  float scale,
  const ShaderSetting& shader_setting) {
  update_normal_dists(name, points.data(), covs.data(), points.size(), scale, shader_setting);
}

// ThinLines
template <typename Scalar, int Dim>
std::shared_ptr<glk::ThinLines>
LightViewerContext::update_thin_lines(const std::string& name, const Eigen::Matrix<Scalar, Dim, 1>* points, int num_points, bool line_strip, const ShaderSetting& shader_setting) {
  if constexpr (std::is_same<Scalar, float>::value && Dim == 3) {
    return update_thin_lines(name, reinterpret_cast<const float*>(points), nullptr, num_points, nullptr, 0, line_strip, shader_setting);
  } else {
    const auto points_3f = glk::convert_to_vector<float, 3, 1>(points, num_points);
    return update_thin_lines(name, points_3f.data(), num_points, line_strip, shader_setting);
  }
}

template <typename ScalarV, int DimV, typename ScalarC, int DimC>
std::shared_ptr<glk::ThinLines> LightViewerContext::update_thin_lines(
  const std::string& name,
  const Eigen::Matrix<ScalarV, DimV, 1>* points,
  const Eigen::Matrix<ScalarC, DimC, 1>* colors,
  int num_points,
  bool line_strip,
  const ShaderSetting& shader_setting) {
  if constexpr (std::is_same<ScalarV, float>::value && DimV == 3 && std::is_same<ScalarC, float>::value && DimC == 4) {
    return update_thin_lines(name, reinterpret_cast<const float*>(points), reinterpret_cast<const float*>(colors), num_points, nullptr, 0, line_strip, shader_setting);
  } else {
    const auto points_3f = glk::convert_to_vector<float, 3, 1>(points, num_points);
    const auto colors_4f = glk::convert_to_vector<float, 4, 1>(colors, num_points);
    return update_thin_lines(name, points_3f.data(), colors_4f.data(), num_points, line_strip, shader_setting);
  }
}

template <typename ScalarV, int DimV, typename ScalarC, int DimC>
std::shared_ptr<glk::ThinLines> LightViewerContext::update_thin_lines(
  const std::string& name,
  const Eigen::Matrix<ScalarV, DimV, 1>* points,
  const Eigen::Matrix<ScalarC, DimC, 1>* colors,
  int num_points,
  const unsigned int* indices,
  int num_indices,
  bool line_strip,
  const ShaderSetting& shader_setting) {
  if constexpr (std::is_same<ScalarV, float>::value && DimV == 3 && std::is_same<ScalarC, float>::value && DimC == 4) {
    return update_thin_lines(name, reinterpret_cast<const float*>(points), reinterpret_cast<const float*>(colors), num_points, indices, num_indices, line_strip, shader_setting);
  } else {
    const auto points_3f = glk::convert_to_vector<float, 3, 1>(points, num_points);
    const auto colors_4f = glk::convert_to_vector<float, 4, 1>(colors, num_points);
    return update_thin_lines(name, points_3f.data(), colors_4f.data(), num_points, indices, num_indices, line_strip, shader_setting);
  }
}

template <typename Point, typename Alloc>
std::shared_ptr<glk::ThinLines>
LightViewerContext::update_thin_lines(const std::string& name, const std::vector<Point, Alloc>& points, bool line_strip, const ShaderSetting& shader_setting) {
  return update_thin_lines(name, points.data(), points.size(), line_strip, shader_setting);
}

template <typename Point, typename Alloc>
std::shared_ptr<glk::ThinLines> LightViewerContext::update_thin_lines(
  const std::string& name,
  const std::vector<Point, Alloc>& points,
  const std::vector<unsigned int>& indices,
  bool line_strip,
  const ShaderSetting& shader_setting) {
  return update_thin_lines(name, points.data(), static_cast<const Eigen::Vector4f*>(nullptr), points.size(), indices.data(), indices.size(), line_strip, shader_setting);
}

template <typename Point, typename AllocP, typename Color, typename AllocC>
std::shared_ptr<glk::ThinLines> LightViewerContext::update_thin_lines(
  const std::string& name,
  const std::vector<Point, AllocP>& points,
  const std::vector<Color, AllocC>& colors,
  bool line_strip,
  const ShaderSetting& shader_setting) {
  return update_thin_lines(name, points.data(), colors.data(), points.size(), line_strip, shader_setting);
}

template <typename Point, typename AllocP, typename Color, typename AllocC>
std::shared_ptr<glk::ThinLines> LightViewerContext::update_thin_lines(
  const std::string& name,
  const std::vector<Point, AllocP>& points,
  const std::vector<Color, AllocC>& colors,
  const std::vector<unsigned int>& indices,
  bool line_strip,
  const ShaderSetting& shader_setting) {
  return update_thin_lines(name, points.data(), colors.data(), points.size(), indices.data(), indices.size(), line_strip, shader_setting);
}

}  // namespace guik

#endif