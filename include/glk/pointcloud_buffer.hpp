#ifndef GLK_POINTCLOUD_BUFFER_HPP
#define GLK_POINTCLOUD_BUFFER_HPP

#include <atomic>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <glk/drawable.hpp>
#include <glk/colormap.hpp>

namespace glk {

struct AuxBufferData {
  std::string attribute_name;
  int dim;
  int stride;
  GLuint buffer;
};

class PointCloudBuffer : public glk::Drawable {
public:
  using Ptr = std::shared_ptr<PointCloudBuffer>;

  PointCloudBuffer(int stride, int num_points);
  PointCloudBuffer(const float* data, int stride, int num_points);

  // Eigen utility constructors
  PointCloudBuffer(const Eigen::Matrix<float, 3, -1>& points);
  PointCloudBuffer(const Eigen::Matrix<double, 3, -1>& points);

  template <typename Scalar, int Dim, typename Allocator>
  PointCloudBuffer(const std::vector<Eigen::Matrix<Scalar, Dim, 1>, Allocator>& points);

  PointCloudBuffer(const Eigen::Vector3f* points, int num_points);
  PointCloudBuffer(const Eigen::Vector4f* points, int num_points);
  PointCloudBuffer(const Eigen::Vector3d* points, int num_points);
  PointCloudBuffer(const Eigen::Vector4d* points, int num_points);

  virtual ~PointCloudBuffer() override;

  template <int N, typename Allocator>
  void add_normals(const std::vector<Eigen::Matrix<float, N, 1>, Allocator>& normals);
  template <int N, typename Allocator>
  void add_normals(const std::vector<Eigen::Matrix<double, N, 1>, Allocator>& normals);
  template <int N>
  void add_normals(const Eigen::Matrix<float, N, 1>* normals, int num_points);
  template <int N>
  void add_normals(const Eigen::Matrix<double, N, 1>* normals, int num_points);

  template <typename Allocator>
  void add_color(const std::vector<Eigen::Vector4f, Allocator>& colors);
  template <typename Allocator>
  void add_color(const std::vector<Eigen::Vector4d, Allocator> colors);
  void add_color(const Eigen::Vector4f* colors, int num_points);
  void add_color(const Eigen::Vector4d* colors, int num_points);

  void add_intensity(glk::COLORMAP colormap, const std::vector<float>& intensities, float scale = 1.0f);
  void add_intensity(glk::COLORMAP colormap, const std::vector<double>& intensities, float scale = 1.0f);
  void add_intensity(glk::COLORMAP colormap, const float* intensities, const int num_points, float scale = 1.0f);
  void add_intensity(glk::COLORMAP colormap, const double* intensities, const int num_points, float scale = 1.0f);

  void add_normals(const float* data, int stride, int num_points);
  void add_color(const float* data, int stride, int num_points);
  void add_intensity(glk::COLORMAP colormap, const float* data, int stride, int num_points, float scale = 1.0f);
  void add_buffer(const std::string& attribute_name, int dim, const float* data, int stride, int num_points);

  void enable_partial_rendering(int points_budget = 8192 * 5);
  void disable_partial_rendering();

  void bind(glk::GLSLShader& shader) const;
  void unbind(glk::GLSLShader& shader) const;

  virtual void draw(glk::GLSLShader& shader) const override;

  GLuint vba_id() const;
  GLuint vbo_id() const;
  GLuint ebo_id() const;

  int get_aux_size() const;
  const AuxBufferData& get_aux_buffer(int i) const;

  int size() const { return num_points; }

private:
  mutable std::atomic_uint rendering_count;
  int points_rendering_budget;

  GLuint vao;
  GLuint vbo;
  GLuint ebo;
  int stride;
  int num_points;

  std::vector<AuxBufferData> aux_buffers;
};

// template methods
template <typename Scalar, int Dim, typename Allocator>
PointCloudBuffer::PointCloudBuffer(const std::vector<Eigen::Matrix<Scalar, Dim, 1>, Allocator>& points)
: PointCloudBuffer(points.data(), points.size()) {}

template <int N, typename Allocator>
void PointCloudBuffer::add_normals(const std::vector<Eigen::Matrix<float, N, 1>, Allocator>& normals) {
  add_normals(normals[0].data(), sizeof(float) * N, normals.size());
}

template <int N, typename Allocator>
void PointCloudBuffer::add_normals(const std::vector<Eigen::Matrix<double, N, 1>, Allocator>& normals) {
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normals_f(normals.size());
  std::transform(normals.begin(), normals.end(), normals_f.begin(), [](const Eigen::Matrix<double, N, 1>& n) { return n.template head<3>().template cast<float>(); });
  add_normals(normals_f[0].data(), sizeof(Eigen::Vector3f), normals_f.size());
}

template <int N>
void PointCloudBuffer::add_normals(const Eigen::Matrix<float, N, 1>* normals, int num_points) {
  add_normals(normals->data(), sizeof(float) * N, num_points);
}

template <int N>
void PointCloudBuffer::add_normals(const Eigen::Matrix<double, N, 1>* normals, int num_points) {
  std::vector<Eigen::Matrix<float, N, 1>, Eigen::aligned_allocator<Eigen::Matrix<float, N, 1>>> normals_f(num_points);
  std::transform(normals, normals + num_points, normals_f.begin(), [](const Eigen::Matrix<double, N, 1>& p) { return p.template cast<float>(); });
  add_normals(normals_f);
}

template <typename Allocator>
void PointCloudBuffer::add_color(const std::vector<Eigen::Vector4f, Allocator>& colors) {
  add_color(colors[0].data(), sizeof(Eigen::Vector4f), colors.size());
}

template <typename Allocator>
void PointCloudBuffer::add_color(const std::vector<Eigen::Vector4d, Allocator> colors) {
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> colors_f(colors.size());
  std::transform(colors.begin(), colors.end(), colors_f.begin(), [](const Eigen::Vector4d& p) { return p.cast<float>(); });
  add_color(colors_f[0].data(), sizeof(Eigen::Vector4f), colors_f.size());
}

}  // namespace glk

#endif