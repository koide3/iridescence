#ifndef GLK_POINTCLOUD_BUFFER_HPP
#define GLK_POINTCLOUD_BUFFER_HPP

#include <atomic>
#include <memory>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <glk/drawable.hpp>
#include <glk/colormap.hpp>
#include <glk/api_export.hpp>

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

  // Add point attribute methods
  template <int N, typename Allocator>
  void add_normals(const std::vector<Eigen::Matrix<float, N, 1>, Allocator>& normals);
  template <int N, typename Allocator>
  void add_normals(const std::vector<Eigen::Matrix<double, N, 1>, Allocator>& normals);
  template <int N>
  void add_normals(const Eigen::Matrix<float, N, 1>* normals, int num_points);
  template <int N>
  void add_normals(const Eigen::Matrix<double, N, 1>* normals, int num_points);

  template <typename Scalar, typename Allocator>
  void add_color(const std::vector<Eigen::Matrix<Scalar, 4, 1>, Allocator>& colors);
  void add_color(const Eigen::Vector4f* colors, int num_points);
  void add_color(const Eigen::Vector4d* colors, int num_points);

  void add_intensity(glk::COLORMAP colormap, const std::vector<float>& intensities, float scale = 1.0f);
  void add_intensity(glk::COLORMAP colormap, const std::vector<double>& intensities, float scale = 1.0f);
  void add_intensity(glk::COLORMAP colormap, const float* intensities, const int num_points, float scale = 1.0f);
  void add_intensity(glk::COLORMAP colormap, const double* intensities, const int num_points, float scale = 1.0f);

  void add_normals(const float* data, int stride, int num_points);
  void add_color(const float* data, int stride, int num_points);
  void add_intensity(glk::COLORMAP colormap, const float* data, int stride, int num_points, float scale = 1.0f);

  void set_colormap_buffer(const std::string& attribute_name);
  void add_colormap(std::vector<float>& cmap, float scale = 1.0f);
  void add_colormap(std::vector<double>& cmap, float scale = 1.0);
  void add_colormap(const float* data, int stride, int num_points, float scale = 1.0f);

  void add_buffer(const std::string& attribute_name, const std::vector<float>& data);
  void add_buffer(const std::string& attribute_name, const std::vector<double>& data);
  template <typename Scalar, int D, typename Allocator>
  void add_buffer(const std::string& attribute_name, const std::vector<Eigen::Matrix<Scalar, D, 1>, Allocator>& data);
  void add_buffer(const std::string& attribute_name, int dim, const float* data, int stride, int num_points);

  // Partial attribute update methods (User must ensure that stride and dim are matched with existing attribute)
  void update_buffer_with_indices(GLuint buffer, const float* data, int stride, const unsigned int* indices, int num_indices);
  void update_buffer_with_indices(const std::string& attribute_name, int dim, const float* data, int stride, const unsigned int* indices, int num_indices);

  template <typename Scalar, int D, typename Allocator>
  void update_points_with_indices(const std::vector<Eigen::Matrix<Scalar, D, 1>, Allocator>& points, const std::vector<unsigned int>& indices);
  void update_points_with_indices(const Eigen::Vector3f* points, const unsigned int* indices, int num_indices);
  void update_points_with_indices(const Eigen::Vector4f* points, const unsigned int* indices, int num_indices);
  void update_points_with_indices(const Eigen::Vector3d* points, const unsigned int* indices, int num_indices);
  void update_points_with_indices(const Eigen::Vector4d* points, const unsigned int* indices, int num_indices);
  void update_points_with_indices(const float* data, int stride, const unsigned int* indices, int num_indices);

  template <typename Scalar, typename Allocator>
  void update_color_with_indices(const std::vector<Eigen::Matrix<Scalar, 4, 1>, Allocator>& colors, const std::vector<unsigned int>& indices);
  void update_color_with_indices(const Eigen::Vector4f* colors, const unsigned int* indices, int num_indices);
  void update_color_with_indices(const Eigen::Vector4d* colors, const unsigned int* indices, int num_indices);

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

  size_t memory_usage() const;

private:
  mutable std::atomic_uint rendering_count;
  int points_rendering_budget;

  GLuint vao;
  GLuint vbo;
  GLuint ebo;
  int stride;
  int num_points;

  GLuint cmap_bo;  // buffer object for colormap attribute

  std::vector<AuxBufferData> aux_buffers;
};

// template methods
template <typename Scalar, int Dim, typename Allocator>
PointCloudBuffer::PointCloudBuffer(const std::vector<Eigen::Matrix<Scalar, Dim, 1>, Allocator>& points) : PointCloudBuffer(points.data(), points.size()) {}

template <int N, typename Allocator>
void PointCloudBuffer::add_normals(const std::vector<Eigen::Matrix<float, N, 1>, Allocator>& normals) {
  add_normals(normals[0].data(), sizeof(float) * N, normals.size());
}

template <int N, typename Allocator>
void PointCloudBuffer::add_normals(const std::vector<Eigen::Matrix<double, N, 1>, Allocator>& normals) {
  std::vector<Eigen::Vector3f> normals_f(normals.size());
  std::transform(normals.begin(), normals.end(), normals_f.begin(), [](const Eigen::Matrix<double, N, 1>& n) { return n.template head<3>().template cast<float>(); });
  add_normals(normals_f[0].data(), sizeof(Eigen::Vector3f), normals_f.size());
}

template <int N>
void PointCloudBuffer::add_normals(const Eigen::Matrix<float, N, 1>* normals, int num_points) {
  add_normals(normals->data(), sizeof(float) * N, num_points);
}

template <int N>
void PointCloudBuffer::add_normals(const Eigen::Matrix<double, N, 1>* normals, int num_points) {
  std::vector<Eigen::Matrix<float, N, 1>> normals_f(num_points);
  std::transform(normals, normals + num_points, normals_f.begin(), [](const Eigen::Matrix<double, N, 1>& p) { return p.template cast<float>(); });
  add_normals(normals_f);
}

template <typename Scalar, typename Allocator>
void PointCloudBuffer::add_color(const std::vector<Eigen::Matrix<Scalar, 4, 1>, Allocator>& colors) {
  add_color(colors.data(), colors.size());
}

template <typename Scalar, int D, typename Allocator>
void PointCloudBuffer::add_buffer(const std::string& attribute_name, const std::vector<Eigen::Matrix<Scalar, D, 1>, Allocator>& data) {
  if constexpr (std::is_same<Scalar, float>::value) {
    add_buffer(attribute_name, D, data[0].data(), sizeof(float) * D, data.size());
  } else {
    std::vector<Eigen::Matrix<float, D, 1>> data_f(data.size());
    std::transform(data.begin(), data.end(), data_f.begin(), [](const Eigen::Matrix<double, D, 1>& p) { return p.template cast<float>(); });
    add_buffer(attribute_name, D, data_f.data(), sizeof(float) * D, data_f.size());
  }
}

template <typename Scalar, int D, typename Allocator>
void PointCloudBuffer::update_points_with_indices(const std::vector<Eigen::Matrix<Scalar, D, 1>, Allocator>& points, const std::vector<unsigned int>& indices) {
  if (points.size() != indices.size()) {
    std::cerr << "error: points.size() != indices.size() (" << points.size() << " vs " << indices.size() << ")" << std::endl;
  }
  update_points_with_indices(points.data(), indices.data(), indices.size());
}

template <typename Scalar, typename Allocator>
void PointCloudBuffer::update_color_with_indices(const std::vector<Eigen::Matrix<Scalar, 4, 1>, Allocator>& colors, const std::vector<unsigned int>& indices) {
  if (colors.size() != indices.size()) {
    std::cerr << "error: colors.size() != indices.size() (" << colors.size() << " vs " << indices.size() << ")" << std::endl;
  }
  update_color_with_indices(colors.data(), indices.data(), indices.size());
}

}  // namespace glk

#endif