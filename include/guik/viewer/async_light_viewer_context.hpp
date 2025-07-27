#ifndef GUIK_ASYNC_LIGHT_VIEWER_CONTEXT_HPP
#define GUIK_ASYNC_LIGHT_VIEWER_CONTEXT_HPP

#include <guik/viewer/light_viewer_context.hpp>

namespace guik {

/**
 * @brief Async interface to manipulate LightViewerContext.
 *        All OpenGL operations will be done in the viewer thread through `invoke` method.
 *        Note that async operations come at a cost of data copy.
 */
class AsyncLightViewerContext {
public:
  AsyncLightViewerContext();  // context must be set manually
  AsyncLightViewerContext(LightViewerContext* context);
  ~AsyncLightViewerContext();

  void clear();
  void clear_text();
  void append_text(const std::string& text);
  void register_ui_callback(const std::string& name, const std::function<void()>& callback = 0);
  void remove_ui_callback(const std::string& name);

  void disable_xy_grid() { set_draw_xy_grid(false); }
  void enable_xy_grid() { set_draw_xy_grid(true); }
  void set_draw_xy_grid(bool draw_xy_grid);
  void set_colormap(glk::COLORMAP colormap);

  void clear_drawables();
  void clear_drawables(const std::function<bool(const std::string&)>& fn);
  void remove_drawable(const std::string& name);
  void remove_drawable(const std::regex& regex);

  void reset_center();
  void lookat(const Eigen::Vector3f& pt);
  template <typename Vector>
  void lookat(const Vector& pt) {
    const auto ptf = pt.eval().template cast<float>();
    lookat(ptf);
  }
  void use_orbit_camera_control(double distance = 80.0, double theta = 0.0, double phi = -60.0f * M_PI / 180.0f);
  void use_orbit_camera_control_xz(double distance = 80.0, double theta = 0.0, double phi = 0.0);
  void use_topdown_camera_control(double distance = 80.0, double theta = 0.0);
  void use_arcball_camera_control(double distance = 80.0, double theta = 0.0, double phi = -60.0f * M_PI / 180.0f);
  void use_fps_camera_control(double fovy_deg = 60.0);

  void update_drawable_setting(const std::string& name, const ShaderSetting& shader_setting);

  // utility methods to directly create and update drawables
  // PointCloudBuffer
  void update_points(const std::string& name, const float* data, int stride, int num_points, const ShaderSetting& shader_setting);
  template <typename Scalar, int Dim>
  void update_points(const std::string& name, const Eigen::Matrix<Scalar, Dim, 1>* points, int num_points, const ShaderSetting& shader_setting);

  template <typename Scalar, int Dim, typename Allocator>
  void update_points(const std::string& name, const std::vector<Eigen::Matrix<Scalar, Dim, 1>, Allocator>& points, const ShaderSetting& shader_setting);

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
  void update_thin_lines(
    const std::string& name,
    const float* vertices,
    const float* colors,
    int num_vertices,
    const unsigned int* indices,
    int num_indices,
    bool line_strip,
    const ShaderSetting& shader_setting);

  template <typename Scalar, int Dim>
  void update_thin_lines(const std::string& name, const Eigen::Matrix<Scalar, Dim, 1>* points, int num_points, bool line_strip, const ShaderSetting& shader_setting);

  template <typename ScalarV, int DimV, typename ScalarC, int DimC>
  void update_thin_lines(
    const std::string& name,
    const Eigen::Matrix<ScalarV, DimV, 1>* points,
    const Eigen::Matrix<ScalarC, DimC, 1>* colors,
    int num_points,
    bool line_strip,
    const ShaderSetting& shader_setting);

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

  template <typename Point, typename Alloc>
  void update_thin_lines(const std::string& name, const std::vector<Point, Alloc>& points, bool line_strip, const ShaderSetting& shader_setting);

  template <typename Point, typename Alloc>
  void update_thin_lines(
    const std::string& name,
    const std::vector<Point, Alloc>& points,
    const std::vector<unsigned int>& indices,
    bool line_strip,
    const ShaderSetting& shader_setting);

  template <typename Point, typename AllocP, typename Color, typename AllocC>
  void update_thin_lines(
    const std::string& name,
    const std::vector<Point, AllocP>& points,
    const std::vector<Color, AllocC>& colors,
    bool line_strip,
    const ShaderSetting& shader_setting);

  template <typename Point, typename AllocP, typename Color, typename AllocC>
  void update_thin_lines(
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

// NormalDistributions
template <typename Scalar, int Dim, typename Alloc1, typename Alloc2>
void AsyncLightViewerContext::update_normal_dists(
  const std::string& name,
  const std::vector<Eigen::Matrix<Scalar, Dim, 1>, Alloc1> points,
  const std::vector<Eigen::Matrix<Scalar, Dim, Dim>, Alloc2> covs,
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