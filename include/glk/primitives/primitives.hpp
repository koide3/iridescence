#ifndef GLK_PRIMITIVES_HPP
#define GLK_PRIMITIVES_HPP

#include <glk/drawble.hpp>

namespace glk {

class Primitives {
private:
  Primitives() { meshes.resize(NUM_PRIMITIVES, nullptr); }

public:
  enum PrimitiveType { ICOSAHEDRON = 0, SPHERE, CUBE, CONE, GRID, COORDINATE_SYSTEM, BUNNY, WIRE_ICOSAHEDRON, WIRE_SPHERE, WIRE_CUBE, WIRE_CONE, WIRE_BUNNY, NUM_PRIMITIVES };

  static Primitives *instance() {
    if (instance_ == nullptr) {
      instance_ = new Primitives();
    }
    return instance_;
  }

  static const glk::Drawable& primitive(PrimitiveType type) {
    return Primitives::instance()->create_primitive(type);
  }

  static std::shared_ptr<glk::Drawable> primitive_ptr(PrimitiveType type) {
    return Primitives::instance()->create_primitive_ptr(type);
  }

private:
  const glk::Drawable& create_primitive(PrimitiveType type);
  std::shared_ptr<glk::Drawable> create_primitive_ptr(PrimitiveType type);

private:
  static Primitives *instance_;

  std::vector<std::shared_ptr<glk::Drawable>> meshes;
};
}  // namespace glk

#endif