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

/// @brief Viewer context.
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

  /// @brief Clear everything in the viewer (callbacks, filters, drawables, texts).
  virtual void clear();
  /// @brief Clear texts.
  virtual void clear_text();

  /// @brief Append debug text to the viewer. This method is thread-safe.
  virtual void append_text(const std::string& text);

  /// @brief Register a UI callback function.
  /// @param name     Callback name
  /// @param callback Callback function. If null, the callback with the given name is removed.
  virtual void register_ui_callback(const std::string& name, const std::function<void()>& callback = 0);
  /// @brief Remove a UI callback function.
  /// @param name Callback name
  void remove_ui_callback(const std::string& name);

  /// @brief Global shader setting.
  guik::ShaderSetting& shader_setting() { return global_shader_setting; }
  /// @brief Global shader setting.
  const guik::ShaderSetting& shader_setting() const { return global_shader_setting; }

  /// @brief Disable XY grid drawing.
  void disable_xy_grid() { set_draw_xy_grid(false); }
  /// @brief Enable XY grid drawing.
  void enable_xy_grid() { set_draw_xy_grid(true); }
  /// @brief Set enable/disable XY grid drawing.
  void set_draw_xy_grid(bool draw_xy_grid);

  /// @brief Set colormap used for RAINBOW and VERTEX_COLORMAP modes.
  void set_colormap(glk::COLORMAP colormap);

  /// @brief Set screen effect.
  void set_screen_effect(const std::shared_ptr<glk::ScreenEffect>& effect);
  /// @brief Get screen effect.
  const std::shared_ptr<glk::ScreenEffect>& get_screen_effect() const;
  /// @brief Set background texture.
  void set_bg_texture(const std::shared_ptr<glk::Texture>& bg_texture);

  /// @brief Set range of color mapping for RAINBOW mode.
  void set_rainbow_range(const Eigen::Vector2f& minmax_z);
  /// @brief Set axis of color mapping for RAINBOW mode.
  void set_rainbow_axis(const Eigen::Vector3f& axis);

  /// @brief Set point shape properties.
  /// @param point_size   Point size
  /// @param metric       If true, point size is in the metric unit [m], otherwise in pixel unit.
  /// @param circle       If true, points are rendered as circles, otherwise as squares.
  void set_point_shape(float point_size = 1.0f, bool metric = true, bool circle = true);

  /// @brief Enable normal buffer.
  void enable_normal_buffer();
  /// @brief Enable info buffer.
  void enable_info_buffer();

  /// @brief Enable partial rendering (This method will be deprecated).
  void enable_decimal_rendering();
  /// @brief Enable partial rendering.
  /// @param clear_thresh Threshold for clearing the partial rendering buffer.
  void enable_partial_rendering(double clear_thresh = 1e-6);
  /// @brief Disable partial rendering.
  void disable_partial_rendering();

  /// @brief Whether normal buffer is enabled.
  bool normal_buffer_enabled() const;
  /// @brief  Whether info buffer is enabled.
  bool info_buffer_enabled() const;
  /// @brief  Whether partial rendering is enabled.
  bool partial_rendering_enabled() const;

  /// @brief Color buffer.
  const glk::Texture& color_buffer() const;
  /// @brief Depth buffer.
  const glk::Texture& depth_buffer() const;
  /// @brief Normal buffer.
  const glk::Texture& normal_buffer() const;
  /// @brief Info buffer.
  const glk::Texture& info_buffer() const;
  /// @brief Dynamic flag buffer.
  const glk::Texture& dynamic_flag_buffer() const;

  /// @brief Clear all drawables.
  void clear_drawables();
  /// @brief Clear drawables that satisfy the given condition.
  /// @param fn Condition function
  void clear_drawables(const std::function<bool(const std::string&)>& fn);

  /// @brief Get all drawables.
  /// @note  This method may be removed in the future.
  std::unordered_map<std::string, std::pair<ShaderSetting::Ptr, glk::Drawable::ConstPtr>>& get_drawables();

  /// @brief Find a drawable by name.
  /// @param name  Drawable name
  /// @return Drawable and its shader setting pair if found, otherwise an empty pair
  std::pair<ShaderSetting::Ptr, glk::Drawable::ConstPtr> find_drawable(const std::string& name);

  /// @brief Remove a drawable by name.
  void remove_drawable(const std::string& name);
  /// @brief Remove all drawables that match the given regex.
  void remove_drawable(const std::regex& regex);

  /// @brief Update a drawable.
  /// @param name           Drawable name
  /// @param drawable       Drawable
  /// @param shader_setting Shader setting
  void update_drawable(const std::string& name, const glk::Drawable::ConstPtr& drawable, const ShaderSetting& shader_setting = ShaderSetting());

  /// @brief Clear all drawable filters.
  void clear_drawable_filters();
  /// @brief Register a drawable filter.
  /// @param filter_name  Filter name
  /// @param filter       Filter function. If null, the filter with the given name is removed.
  void register_drawable_filter(const std::string& filter_name, const std::function<bool(const std::string&)>& filter = 0);
  /// @brief Remove a drawable filter.
  void remove_drawable_filter(const std::string& filter_name);

  /// @brief Clear partial rendering buffer.
  void clear_partial_rendering();

  /// @brief Get camera control (model matrix control).
  const std::shared_ptr<CameraControl>& get_camera_control() const;
  /// @brief Get projection control (projection matrix control).
  const std::shared_ptr<ProjectionControl>& get_projection_control() const;
  /// @brief Set camera control (model matrix control).
  void set_camera_control(const std::shared_ptr<CameraControl>& camera_control);
  /// @brief Set projection control (projection matrix control).
  void set_projection_control(const std::shared_ptr<ProjectionControl>& projection_control);

  /// @brief Save camera settings to a file.
  bool save_camera_settings(const std::string& path) const;
  /// @brief Load camera settings from a file.
  bool load_camera_settings(const std::string& path);

  /// @brief Reset camera center to the center of the screen.
  void reset_center();
  /// @brief Make the camera look at a point.
  void lookat(const Eigen::Vector3f& pt);
  /// @brief Make the camera look at a point.
  template <typename Vector>
  void lookat(const Vector& pt) {
    const auto ptf = pt.eval().template cast<float>();
    lookat(ptf);
  }

  /// @brief Use orbit camera control (XY plane) useful for systems in which XYZ = front-left-up like LiDAR odometry.
  std::shared_ptr<OrbitCameraControlXY> use_orbit_camera_control(double distance = 80.0, double theta = 0.0, double phi = -60.0f * M_PI / 180.0f);
  /// @brief Use orbit camera control (XZ plane) useful for systems in which XYZ = right-down-forward like visual odometry.
  std::shared_ptr<OrbitCameraControlXZ> use_orbit_camera_control_xz(double distance = 80.0, double theta = 0.0, double phi = 0.0);
  /// @brief Use top-down camera control useful for 2D SLAM.
  std::shared_ptr<TopDownCameraControl> use_topdown_camera_control(double distance = 80.0, double theta = 0.0);
  /// @brief Use arcball camera control.
  std::shared_ptr<ArcBallCameraControl> use_arcball_camera_control(double distance = 80.0, double theta = 0.0, double phi = -60.0f * M_PI / 180.0f);
  /// @brief Use FPS game-like camera control.
  std::shared_ptr<FPSCameraControl> use_fps_camera_control(double fovy_deg = 60.0);

  /// @brief Get underlying GLCanvas.
  guik::GLCanvas& get_canvas();
  // @brief Get top left corner of the canvas rectangle.
  Eigen::Vector2i canvas_tl() const { return canvas_rect_min; }
  /// @brief Get bottom right corner of the canvas rectangle.
  Eigen::Vector2i canvas_br() const { return canvas_rect_max; }
  /// @brief Get canvas size.
  Eigen::Vector2i canvas_size() const { return canvas->size; }
  /// @brief Get view matrix.
  Eigen::Matrix4f view_matrix() const { return canvas->camera_control->view_matrix(); }
  /// @brief Get projection matrix.
  Eigen::Matrix4f projection_matrix() const { return canvas->projection_control->projection_matrix(); }

  /// @brief Pick info from the info buffer.
  /// @param p       Pixel coordinates
  /// @param window  Window size for finding a valid pixel neighboring p
  /// @return        Info vector
  Eigen::Vector4i pick_info(const Eigen::Vector2i& p, int window = 2) const;

  /// @brief Pick depth from the depth buffer.
  /// @param p       Pixel coordinates
  /// @param window  Window size for finding a valid pixel neighboring p
  /// @return        Depth value. Depth > 1.0f indicates background.
  float pick_depth(const Eigen::Vector2i& p, int window = 2) const;

  /// @brief Unproject a pixel to a 3D point.
  /// @param p       Pixel coordinates
  /// @param depth   Depth value
  /// @return 3D point in the world coordinate
  Eigen::Vector3f unproject(const Eigen::Vector2i& p, float depth) const;

  /// @brief Pick a 3D point from the depth buffer.
  /// @param p       Pixel coordinates
  /// @param window  Window size for finding a valid pixel neighboring p
  /// @param info    If not null, the info vector is stored here.
  /// @return        3D point in the world coordinate. If no valid depth is found, std::nullopt is returned.
  std::optional<Eigen::Vector3f> pick_point(int button = 0, int window = 2, Eigen::Vector4i* info = nullptr) const;

  // Buffer read methods
  /// @brief  Read color buffer values.
  /// @return Color buffer values in RGBA format.
  std::vector<unsigned char> read_color_buffer() const;

  /// @brief  Read depth buffer values.
  /// @param  real_scale If true, depth values are scaled to real-world values. Otherwise, raw depth buffer values are returned.
  /// @return Depth buffer values.
  std::vector<float> read_depth_buffer(bool real_scale = true);

  /// @brief Save color buffer to an image file (PNG).
  bool save_color_buffer(const std::string& filename);
  /// @brief Save depth buffer to a file (PNG).
  bool save_depth_buffer(const std::string& filename, bool real_scale = true);

  // Async
  /// @brief Get an asynchronous light viewer context.
  AsyncLightViewerContext async();

  // Utility methods to directly create and update drawables
  // PointCloudBuffer

  /// @brief Register or update a point cloud buffer.
  /// @param name           Drawable name
  /// @param data           Pointer to point data (float array)
  /// @param stride         Stride (in bytes) between points
  /// @param num_points     Number of points
  /// @param shader_setting Shader setting
  /// @return               Created PointCloudBuffer
  std::shared_ptr<glk::PointCloudBuffer> update_points(const std::string& name, const float* data, int stride, int num_points, const ShaderSetting& shader_setting);
  /// @brief Register or update a point cloud buffer.
  template <typename Scalar, int Dim>
  std::shared_ptr<glk::PointCloudBuffer> update_points(const std::string& name, const Eigen::Matrix<Scalar, Dim, 1>* points, int num_points, const ShaderSetting& shader_setting);
  /// @brief Register or update a point cloud buffer.
  template <typename Scalar, int Dim, typename Allocator>
  std::shared_ptr<glk::PointCloudBuffer>
  update_points(const std::string& name, const std::vector<Eigen::Matrix<Scalar, Dim, 1>, Allocator>& points, const ShaderSetting& shader_setting);

  // NormalDistributions
  /// @brief Register or update a normal distribution set.
  /// @param name           Drawable name
  /// @param points         Point mean array
  /// @param covs           Point covariance array
  /// @param num_points     Number of points
  /// @param scale          Scale of the normal distributions
  /// @param shader_setting Shader setting
  template <typename Scalar, int Dim>
  void update_normal_dists(
    const std::string& name,
    const Eigen::Matrix<Scalar, Dim, 1>* points,
    const Eigen::Matrix<Scalar, Dim, Dim>* covs,
    int num_points,
    float scale,
    const ShaderSetting& shader_setting);

  /// @brief Register or update a normal distribution set.
  template <typename Scalar, int Dim, typename Alloc1, typename Alloc2>
  void update_normal_dists(
    const std::string& name,
    const std::vector<Eigen::Matrix<Scalar, Dim, 1>, Alloc1>& points,
    const std::vector<Eigen::Matrix<Scalar, Dim, Dim>, Alloc2>& covs,
    float scale,
    const ShaderSetting& shader_setting);

  // ThinLines
  /// @brief Register or update a thin lines drawable.
  /// @param name           Drawable name
  /// @param vertices       Pointer to vertex data (array of float3)
  /// @param colors         Pointer to color data (array of float4). If null, no vertex colors are assigned..
  /// @param num_vertices   Number of vertices
  /// @param indices        Pointer to index data (array of unsigned int). If null, non-indexed drawing is used.
  /// @param num_indices    Number of indices
  /// @param line_strip     If true, line strip mode is used. Otherwise, line list mode is used.
  /// @param shader_setting Shader setting
  std::shared_ptr<glk::ThinLines> update_thin_lines(
    const std::string& name,
    const float* vertices,
    const float* colors,
    int num_vertices,
    const unsigned int* indices,
    int num_indices,
    bool line_strip,
    const ShaderSetting& shader_setting);

  /// @brief Register or update a thin lines drawable.
  template <typename Scalar, int Dim>
  std::shared_ptr<glk::ThinLines>
  update_thin_lines(const std::string& name, const Eigen::Matrix<Scalar, Dim, 1>* points, int num_points, bool line_strip, const ShaderSetting& shader_setting);

  /// @brief Register or update a thin lines drawable.
  template <typename ScalarV, int DimV, typename ScalarC, int DimC>
  std::shared_ptr<glk::ThinLines> update_thin_lines(
    const std::string& name,
    const Eigen::Matrix<ScalarV, DimV, 1>* points,
    const Eigen::Matrix<ScalarC, DimC, 1>* colors,
    int num_points,
    bool line_strip,
    const ShaderSetting& shader_setting);

  /// @brief Register or update a thin lines drawable.
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

  /// @brief Register or update a thin lines drawable.
  template <typename Point, typename Alloc>
  std::shared_ptr<glk::ThinLines> update_thin_lines(const std::string& name, const std::vector<Point, Alloc>& points, bool line_strip, const ShaderSetting& shader_setting);

  /// @brief Register or update a thin lines drawable.
  template <typename Point, typename Alloc>
  std::shared_ptr<glk::ThinLines> update_thin_lines(
    const std::string& name,
    const std::vector<Point, Alloc>& points,
    const std::vector<unsigned int>& indices,
    bool line_strip,
    const ShaderSetting& shader_setting);

  /// @brief Register or update a thin lines drawable.
  template <typename Point, typename AllocP, typename Color, typename AllocC>
  std::shared_ptr<glk::ThinLines> update_thin_lines(
    const std::string& name,
    const std::vector<Point, AllocP>& points,
    const std::vector<Color, AllocC>& colors,
    bool line_strip,
    const ShaderSetting& shader_setting);

  /// @brief Register or update a thin lines drawable.
  template <typename Point, typename AllocP, typename Color, typename AllocC>
  std::shared_ptr<glk::ThinLines> update_thin_lines(
    const std::string& name,
    const std::vector<Point, AllocP>& points,
    const std::vector<Color, AllocC>& colors,
    const std::vector<unsigned int>& indices,
    bool line_strip,
    const ShaderSetting& shader_setting);

  // Primitives
  /// @brief Register or update an icosahedron primitive.
  void update_icosahedron(const std::string& name, const ShaderSetting& shader_setting);
  /// @brief Register or update a sphere primitive.
  void update_sphere(const std::string& name, const ShaderSetting& shader_setting);
  /// @brief Register or update a cube primitive.
  void update_cube(const std::string& name, const ShaderSetting& shader_setting);
  /// @brief Register or update a cone primitive.
  void update_cone(const std::string& name, const ShaderSetting& shader_setting);
  /// @brief Register or update a frustum primitive.
  void update_frustum(const std::string& name, const ShaderSetting& shader_setting);
  /// @brief Register or update a coordinate axes primitive.
  void update_coord(const std::string& name, const ShaderSetting& shader_setting);

  /// @brief Register or update a wireframe icosahedron primitive.
  void update_wire_icosahedron(const std::string& name, const ShaderSetting& shader_setting);
  /// @brief Register or update a wireframe sphere primitive.
  void update_wire_sphere(const std::string& name, const ShaderSetting& shader_setting);
  /// @brief Register or update a wireframe cube primitive.
  void update_wire_cube(const std::string& name, const ShaderSetting& shader_setting);
  /// @brief Register or update a wireframe cone primitive.
  void update_wire_cone(const std::string& name, const ShaderSetting& shader_setting);
  /// @brief Register or update a wireframe frustum primitive.
  void update_wire_frustum(const std::string& name, const ShaderSetting& shader_setting);

protected:
  std::string context_name;                  ///< Viewer context name
  Eigen::Vector2i canvas_rect_min;           ///< Top-left corner of the canvas rectangle
  Eigen::Vector2i canvas_rect_max;           ///< Bottom-right corner of the canvas rectangle
  std::unique_ptr<guik::GLCanvas> canvas;    ///< GL canvas
  guik::ShaderSetting global_shader_setting; ///< Global shader setting

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
  const std::vector<Eigen::Matrix<Scalar, Dim, 1>, Alloc1>& points,
  const std::vector<Eigen::Matrix<Scalar, Dim, Dim>, Alloc2>& covs,
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