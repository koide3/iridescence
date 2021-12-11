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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<PointCloudBuffer>;

  PointCloudBuffer(int stride, int num_points);
  PointCloudBuffer(const float* data, int stride, int num_points);

  // Eigen utility constructors
  PointCloudBuffer(const Eigen::Matrix<float, 3, -1>& points);
  PointCloudBuffer(const Eigen::Matrix<double, 3, -1>& points);
  PointCloudBuffer(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& points);
  PointCloudBuffer(const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>& points);
  PointCloudBuffer(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& points);
  PointCloudBuffer(const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& points);

  PointCloudBuffer(const Eigen::Vector3f* points, int num_points);
  PointCloudBuffer(const Eigen::Vector4f* points, int num_points);
  PointCloudBuffer(const Eigen::Vector3d* points, int num_points);
  PointCloudBuffer(const Eigen::Vector4d* points, int num_points);

  virtual ~PointCloudBuffer() override;

  void add_normals(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& normals);
  void add_color(const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>& colors);
  void add_color(const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& colors);
  void add_intensity(glk::COLORMAP colormap, const std::vector<float>& intensities, float scale = 1.0f);
  void add_intensity(glk::COLORMAP colormap, const std::vector<double>& intensities, float scale = 1.0f);

  void add_normals(const float* data, int stride, int num_points);
  void add_color(const float* data, int stride, int num_points);
  void add_intensity(glk::COLORMAP colormap, const float* data, int stride, int num_points, float scale = 1.0f);
  void add_buffer(const std::string& attribute_name, int dim, const float* data, int stride, int num_points);

  void enable_decimal_rendering(int points_budget = 8192 * 5);

  virtual void draw(glk::GLSLShader& shader) const override;

  GLuint vba_id() const;
  GLuint vbo_id() const;
  GLuint ebo_id() const;

  int get_aux_size() const;
  const AuxBufferData& get_aux_buffer(int i) const;

private:
  mutable std::atomic_uint rendering_count;
  int points_budget;

  GLuint vao;
  GLuint vbo;
  GLuint ebo;
  int stride;
  int num_points;

  std::vector<AuxBufferData> aux_buffers;
};

}  // namespace glk

#endif