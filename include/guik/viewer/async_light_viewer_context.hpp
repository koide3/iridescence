#ifndef GUIK_ASYNC_LIGHT_VIEWER_CONTEXT_HPP
#define GUIK_ASYNC_LIGHT_VIEWER_CONTEXT_HPP

#include <guik/viewer/light_viewer_context.hpp>

namespace guik {

/**
 * @brief Async interface to manipulate LightViewerContext.
 *        All OpenGL operations will be called in the viewer thread through `invoke` method.
 *        Note that async operations come at a cost of data copy.
 */
class AsyncLightViewerContext {
public:
  AsyncLightViewerContext();  // context must be set manually
  AsyncLightViewerContext(LightViewerContext* context);
  ~AsyncLightViewerContext();

  /// @brief Clear everything in the viewer (callbacks, filters, drawables, texts).
  void clear();
  /// @brief Clear texts.
  void clear_text();
  /// @brief Append debug text to the viewer. This method is thread-safe.
  void append_text(const std::string& text);
  /// @brief Register a UI callback function.
  /// @param name     Callback name
  /// @param callback Callback function. If null, the callback with the given name is removed.
  void register_ui_callback(const std::string& name, const std::function<void()>& callback = 0);
  /// @brief Remove a UI callback function.
  /// @param name Callback name
  void remove_ui_callback(const std::string& name);

  /// @brief Disable XY grid drawing.
  void disable_xy_grid() { set_draw_xy_grid(false); }
  /// @brief Enable XY grid drawing.
  void enable_xy_grid() { set_draw_xy_grid(true); }
  /// @brief Set enable/disable XY grid drawing.
  void set_draw_xy_grid(bool draw_xy_grid);
  /// @brief Set colormap used for RAINBOW and VERTEX_COLORMAP modes.
  void set_colormap(glk::COLORMAP colormap);

  /// @brief Set point shape properties.
  /// @param point_size   Point size
  /// @param metric       If true, point size is in the metric unit [m], otherwise in pixel unit.
  /// @param circle       If true, points are rendered as circles, otherwise as squares.
  void set_point_shape(float point_size = 1.0f, bool metric = true, bool circle = true);

  /// @brief Clear all drawables.
  void clear_drawables();
  /// @brief Clear drawables that satisfy the given condition.
  /// @param fn Condition function
  void clear_drawables(const std::function<bool(const std::string&)>& fn);
  /// @brief Remove a drawable by name.
  void remove_drawable(const std::string& name);
  /// @brief Remove all drawables that match the given regex.
  void remove_drawable(const std::regex& regex);

  /// @brief Save camera settings to a file.
  void save_camera_settings(const std::string& path);
  /// @brief Load camera settings from a file.
  void load_camera_settings(const std::string& path);
  /// @brief Save color buffer to an image file (PNG).
  void save_color_buffer(const std::string& filename);
  /// @brief Save depth buffer to a file (PNG).
  void save_depth_buffer(const std::string& filename, bool real_scale = true);

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
  void use_orbit_camera_control(double distance = 80.0, double theta = 0.0, double phi = -60.0f * M_PI / 180.0f);
  /// @brief Use orbit camera control (XZ plane) useful for systems in which XYZ = right-down-forward like visual odometry.
  void use_orbit_camera_control_xz(double distance = 80.0, double theta = 0.0, double phi = 0.0);
  /// @brief Use top-down camera control useful for 2D SLAM.
  void use_topdown_camera_control(double distance = 80.0, double theta = 0.0);
  /// @brief Use arcball camera control.
  void use_arcball_camera_control(double distance = 80.0, double theta = 0.0, double phi = -60.0f * M_PI / 180.0f);
  /// @brief Use FPS game-like camera control.
  void use_fps_camera_control(double fovy_deg = 60.0);

  /// @brief Update a drawable's shader setting.
  void update_drawable_setting(const std::string& name, const ShaderSetting& shader_setting);

  // utility methods to directly create and update drawables
  // PointCloudBuffer
  /// @brief Register or update a point cloud buffer.
  /// @param name           Drawable name
  /// @param data           Pointer to point data (float array)
  /// @param stride         Stride (in bytes) between points
  /// @param num_points     Number of points
  /// @param shader_setting Shader setting
  void update_points(const std::string& name, const float* data, int stride, int num_points, const ShaderSetting& shader_setting);
  /// @brief Register or update a point cloud buffer.
  template <typename Scalar, int Dim>
  void update_points(const std::string& name, const Eigen::Matrix<Scalar, Dim, 1>* points, int num_points, const ShaderSetting& shader_setting);
  /// @brief Register or update a point cloud buffer.
  template <typename Scalar, int Dim, typename Allocator>
  void update_points(const std::string& name, const std::vector<Eigen::Matrix<Scalar, Dim, 1>, Allocator>& points, const ShaderSetting& shader_setting);

  /// @brief Register or update a point cloud buffer with colors.
  void update_points(const std::string& name, const float* vertices, int vertex_stride, const float* colors, int color_stride, int num_points, const ShaderSetting& shader_setting);
  /// @brief Register or update a point cloud buffer with colors.
  template <typename ScalarV, int DimV, typename ScalarC, int DimC>
  void update_points(
    const std::string& name,
    const Eigen::Matrix<ScalarV, DimV, 1>* points,
    const Eigen::Matrix<ScalarC, DimC, 1>* colors,
    int num_points,
    const ShaderSetting& shader_setting);
  /// @brief Register or update a point cloud buffer with colors.
  template <typename ScalarV, int DimV, typename AllocatorV, typename ScalarC, int DimC, typename AllocatorC>
  void update_points(
    const std::string& name,
    const std::vector<Eigen::Matrix<ScalarV, DimV, 1>, AllocatorV>& points,
    const std::vector<Eigen::Matrix<ScalarC, DimC, 1>, AllocatorC>& colors,
    const ShaderSetting& shader_setting);

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
  /// @param colors         Pointer to color data (array of float4). If null, no vertex colors are assigned.
  /// @param num_vertices   Number of vertices
  /// @param indices        Pointer to index data (array of unsigned int). If null, non-indexed drawing is used.
  /// @param num_indices    Number of indices
  /// @param line_strip     If true, line strip mode is used. Otherwise, line list mode is used.
  /// @param shader_setting Shader setting
  void update_thin_lines(
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
  void update_thin_lines(const std::string& name, const Eigen::Matrix<Scalar, Dim, 1>* points, int num_points, bool line_strip, const ShaderSetting& shader_setting);

  /// @brief Register or update a thin lines drawable.
  template <typename ScalarV, int DimV, typename ScalarC, int DimC>
  void update_thin_lines(
    const std::string& name,
    const Eigen::Matrix<ScalarV, DimV, 1>* points,
    const Eigen::Matrix<ScalarC, DimC, 1>* colors,
    int num_points,
    bool line_strip,
    const ShaderSetting& shader_setting);

  /// @brief Register or update a thin lines drawable.
  template <typename ScalarV, int DimV, typename ScalarC, int DimC>
  void update_thin_lines(
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
  void update_thin_lines(const std::string& name, const std::vector<Point, Alloc>& points, bool line_strip, const ShaderSetting& shader_setting);

  /// @brief Register or update a thin lines drawable.
  template <typename Point, typename Alloc>
  void update_thin_lines(
    const std::string& name,
    const std::vector<Point, Alloc>& points,
    const std::vector<unsigned int>& indices,
    bool line_strip,
    const ShaderSetting& shader_setting);

  /// @brief Register or update a thin lines drawable.
  template <typename Point, typename AllocP, typename Color, typename AllocC>
  void update_thin_lines(
    const std::string& name,
    const std::vector<Point, AllocP>& points,
    const std::vector<Color, AllocC>& colors,
    bool line_strip,
    const ShaderSetting& shader_setting);

  /// @brief Register or update a thin lines drawable.
  template <typename Point, typename AllocP, typename Color, typename AllocC>
  void update_thin_lines(
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
  LightViewerContext* context;
};

// template methods
// PointCloud
template <typename Scalar, int Dim>
void AsyncLightViewerContext::update_points(const std::string& name, const Eigen::Matrix<Scalar, Dim, 1>* points, int num_points, const ShaderSetting& shader_setting) {
  if constexpr (std::is_same<Scalar, float>::value) {
    return update_points(name, reinterpret_cast<const float*>(points), sizeof(float) * Dim, num_points, shader_setting);
  } else {
    const auto points_3f = glk::convert_to_vector<float, 3, 1>(points, num_points);
    return update_points(name, points_3f.data(), num_points, shader_setting);
  }
}

template <typename Scalar, int Dim, typename Allocator>
void AsyncLightViewerContext::update_points(const std::string& name, const std::vector<Eigen::Matrix<Scalar, Dim, 1>, Allocator>& points, const ShaderSetting& shader_setting) {
  return update_points(name, points.data(), points.size(), shader_setting);
}

template <typename ScalarV, int DimV, typename ScalarC, int DimC>
void AsyncLightViewerContext::update_points(
  const std::string& name,
  const Eigen::Matrix<ScalarV, DimV, 1>* points,
  const Eigen::Matrix<ScalarC, DimC, 1>* colors,
  int num_points,
  const ShaderSetting& shader_setting) {
  if constexpr (std::is_same<ScalarV, float>::value && std::is_same<ScalarC, float>::value) {
    return update_points(
      name,
      reinterpret_cast<const float*>(points),
      sizeof(float) * DimV,
      reinterpret_cast<const float*>(colors),
      sizeof(float) * DimC,
      num_points,
      shader_setting);
  } else {
    const auto points_3f = glk::convert_to_vector<float, 3, 1>(points, num_points);
    const auto colors_4f = glk::convert_to_vector<float, 4, 1>(colors, num_points);
    return update_points(name, points_3f.data(), colors_4f.data(), num_points, shader_setting);
  }
}

template <typename ScalarV, int DimV, typename AllocatorV, typename ScalarC, int DimC, typename AllocatorC>
void AsyncLightViewerContext::update_points(
  const std::string& name,
  const std::vector<Eigen::Matrix<ScalarV, DimV, 1>, AllocatorV>& points,
  const std::vector<Eigen::Matrix<ScalarC, DimC, 1>, AllocatorC>& colors,
  const ShaderSetting& shader_setting) {
  return update_points(name, points.data(), colors.data(), points.size(), shader_setting);
}

// NormalDistributions
template <typename Scalar, int Dim, typename Alloc1, typename Alloc2>
void AsyncLightViewerContext::update_normal_dists(
  const std::string& name,
  const std::vector<Eigen::Matrix<Scalar, Dim, 1>, Alloc1>& points,
  const std::vector<Eigen::Matrix<Scalar, Dim, Dim>, Alloc2>& covs,
  float scale,
  const ShaderSetting& shader_setting) {
  update_normal_dists(name, points.data(), covs.data(), points.size(), scale, shader_setting);
}

// ThinLines
template <typename Scalar, int Dim>
void AsyncLightViewerContext::update_thin_lines(
  const std::string& name,
  const Eigen::Matrix<Scalar, Dim, 1>* points,
  int num_points,
  bool line_strip,
  const ShaderSetting& shader_setting) {
  if constexpr (std::is_same<Scalar, float>::value && Dim == 3) {
    return update_thin_lines(name, reinterpret_cast<const float*>(points), nullptr, num_points, nullptr, 0, line_strip, shader_setting);
  } else {
    const auto points_3f = glk::convert_to_vector<float, 3, 1>(points, num_points);
    return update_thin_lines(name, points_3f.data(), num_points, line_strip, shader_setting);
  }
}

template <typename ScalarV, int DimV, typename ScalarC, int DimC>
void AsyncLightViewerContext::update_thin_lines(
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
void AsyncLightViewerContext::update_thin_lines(
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
void AsyncLightViewerContext::update_thin_lines(const std::string& name, const std::vector<Point, Alloc>& points, bool line_strip, const ShaderSetting& shader_setting) {
  return update_thin_lines(name, points.data(), points.size(), line_strip, shader_setting);
}

template <typename Point, typename Alloc>
void AsyncLightViewerContext::update_thin_lines(
  const std::string& name,
  const std::vector<Point, Alloc>& points,
  const std::vector<unsigned int>& indices,
  bool line_strip,
  const ShaderSetting& shader_setting) {
  return update_thin_lines(name, points.data(), static_cast<const Eigen::Vector4f*>(nullptr), points.size(), indices.data(), indices.size(), line_strip, shader_setting);
}

template <typename Point, typename AllocP, typename Color, typename AllocC>
void AsyncLightViewerContext::update_thin_lines(
  const std::string& name,
  const std::vector<Point, AllocP>& points,
  const std::vector<Color, AllocC>& colors,
  bool line_strip,
  const ShaderSetting& shader_setting) {
  return update_thin_lines(name, points.data(), colors.data(), points.size(), line_strip, shader_setting);
}

template <typename Point, typename AllocP, typename Color, typename AllocC>
void AsyncLightViewerContext::update_thin_lines(
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