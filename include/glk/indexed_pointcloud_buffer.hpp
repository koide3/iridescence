#ifndef GLK_INDEXED_POINTCLOUD_BUFFER_HPP
#define GLK_INDEXED_POINTCLOUD_BUFFER_HPP

#include <glk/pointcloud_buffer.hpp>

namespace glk {

/// @brief IndexedPointCloudBuffer is a drawable that represents a point cloud with an index buffer.
///        It is useful for rendering a subset of points without duplicating the point cloud data.
class IndexedPointCloudBuffer : public glk::Drawable {
public:
  /// @brief Constructor
  /// @param cloud_buffer  Shared pointer to the point cloud buffer.
  /// @param indices       Pointer to the index buffer.
  /// @param num_indices   Number of indices in the index buffer.
  IndexedPointCloudBuffer(const std::shared_ptr<const glk::PointCloudBuffer>& cloud_buffer, const unsigned int* indices, int num_indices);

  /// @brief Constructor
  /// @param cloud_buffer  Shared pointer to the point cloud buffer.
  /// @param indices       Vector of indices.
  IndexedPointCloudBuffer(const std::shared_ptr<const glk::PointCloudBuffer>& cloud_buffer, const std::vector<unsigned int>& indices);

  template <typename Integral>
  IndexedPointCloudBuffer(const std::shared_ptr<const glk::PointCloudBuffer>& cloud_buffer, const Integral* indices, int num_indices);

  template <typename Integral>
  IndexedPointCloudBuffer(const std::shared_ptr<const glk::PointCloudBuffer>& cloud_buffer, const std::vector<Integral>& indices);

  virtual ~IndexedPointCloudBuffer() override;

  virtual void draw(glk::GLSLShader& shader) const override;

  GLuint ebo_id() const;

  int size() const { return num_indices; }

private:
  std::shared_ptr<const glk::PointCloudBuffer> cloud_buffer;

  int num_indices;
  GLuint ebo;
};
}  // namespace glk

#endif