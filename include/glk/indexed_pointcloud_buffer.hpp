#ifndef GLK_INDEXED_POINTCLOUD_BUFFER_HPP
#define GLK_INDEXED_POINTCLOUD_BUFFER_HPP

#include <glk/pointcloud_buffer.hpp>

namespace glk {

class IndexedPointCloudBuffer : public glk::Drawable {
public:
  IndexedPointCloudBuffer(const std::shared_ptr<const glk::PointCloudBuffer>& cloud_buffer, const unsigned int* indices, int num_indices);
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