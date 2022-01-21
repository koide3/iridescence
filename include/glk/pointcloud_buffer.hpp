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

  template <typename Allocator>
  void add_normals(const std::vector<Eigen::Vector3f, Allocator>& normals);
  template <typename Allocator>
  void add_normals(const std::vector<Eigen::Vector3d, Allocator>& normals);
  template <typename Allocator>
  void add_color(const std::vector<Eigen::Vector4f, Allocator>& colors);
  template <typename Allocator>
  void add_color(const std::vector<Eigen::Vector4d, Allocator> colors);

  void add_intensity(glk::COLORMAP colormap, const std::vector<float>& intensities, float scale = 1.0f);
  void add_intensity(glk::COLORMAP colormap, const std::vector<double>& intensities, float scale = 1.0f);

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

// template members
template <typename Scalar, int Dim, typename Allocator>
PointCloudBuffer::PointCloudBuffer(const std::vector<Eigen::Matrix<Scalar, Dim, 1>, Allocator>& points)
: PointCloudBuffer(Eigen::Matrix<Scalar, 3, -1>(Eigen::Map<const Eigen::Matrix<Scalar, Dim, -1>>(points.front().data(), Dim, points.size()).template topRows<3>())) {}

template <typename Allocator>
void PointCloudBuffer::add_normals(const std::vector<Eigen::Vector3f, Allocator>& normals) {
  add_normals(normals[0].data(), sizeof(Eigen::Vector3f), normals.size());
}

template <typename Allocator>
void PointCloudBuffer::add_normals(const std::vector<Eigen::Vector3d, Allocator>& normals) {
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normals_f(normals.size());
  std::transform(normals.begin(), normals.end(), normals_f.begin(), [](const Eigen::Vector3d& n) { return n.cast<float>(); });
  add_normals(normals_f[0].data(), sizeof(Eigen::Vector3f), normals_f.size());
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