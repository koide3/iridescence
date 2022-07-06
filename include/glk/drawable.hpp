#ifndef GLK_DRAWABLE_HPP
#define GLK_DRAWABLE_HPP

#include <vector>
#include <Eigen/Core>

#include <GL/gl3w.h>
#include <glk/glsl_shader.hpp>

namespace glk {

class Drawable {
public:
  using Ptr = std::shared_ptr<Drawable>;
  using ConstPtr = std::shared_ptr<const Drawable>;

  virtual ~Drawable() {}

  virtual void draw(glk::GLSLShader& shader) const {}
};

}  // namespace glk

#endif