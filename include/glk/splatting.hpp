#ifndef GLK_SPLATTING_HPP
#define GLK_SPLATTING_HPP

#include <glk/drawable.hpp>

namespace glk {

class Texture;
class PointCloudBuffer;

std::shared_ptr<glk::GLSLShader> create_splatting_shader();

class Splatting : public glk::Drawable {
public:
  Splatting(const std::shared_ptr<glk::GLSLShader>& shader = nullptr);
  virtual ~Splatting();

  void enable_vertex_radius();
  void disable_vertex_radius();
  void set_point_radius(float r);
  void set_texture(const std::shared_ptr<glk::Texture>& texture);
  void set_cloud_buffer(const std::shared_ptr<glk::PointCloudBuffer>& cloud_buffer);

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  int vert_radius_enabled;
  float point_radius;

  std::shared_ptr<glk::GLSLShader> shader;
  std::shared_ptr<glk::Texture> texture;
  std::shared_ptr<glk::PointCloudBuffer> cloud_buffer;
};

}  // namespace glk

#endif