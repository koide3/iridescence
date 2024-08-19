#ifndef GLK_PRIMITIVES_HPP
#define GLK_PRIMITIVES_HPP

#include <glk/drawable.hpp>
#include <glk/api_export.hpp>

namespace glk {

class Primitives {
private:
  Primitives() { meshes.resize(NUM_PRIMITIVES, nullptr); }

public:
  enum PrimitiveType {
    ICOSAHEDRON = 0,
    SPHERE,
    CUBE,
    CONE,
    GRID,
    COORDINATE_SYSTEM,
    SOLID_COORDINATE_SYSTEM,
    BUNNY,
    FRUSTUM,
    WIRE_ICOSAHEDRON,
    WIRE_SPHERE,
    WIRE_CUBE,
    WIRE_CONE,
    WIRE_BUNNY,
    WIRE_FRUSTUM,
    NUM_PRIMITIVES
  };

  static Primitives* instance() {
    if (instance_ == nullptr) {
      instance_ = new Primitives();
    }
    return instance_;
  }

  static const glk::Drawable& primitive(PrimitiveType type) { return Primitives::instance()->create_primitive(type); }

  static glk::Drawable::ConstPtr primitive_ptr(PrimitiveType type) { return Primitives::instance()->create_primitive_ptr(type); }

  static glk::Drawable::ConstPtr icosahedron() { return primitive_ptr(ICOSAHEDRON); }

  static glk::Drawable::ConstPtr sphere() { return primitive_ptr(SPHERE); }

  static glk::Drawable::ConstPtr cube() { return primitive_ptr(CUBE); }

  static glk::Drawable::ConstPtr cone() { return primitive_ptr(CONE); }

  static glk::Drawable::ConstPtr frustum() { return primitive_ptr(FRUSTUM); }

  static glk::Drawable::ConstPtr coordinate_system() { return primitive_ptr(COORDINATE_SYSTEM); }

  static glk::Drawable::ConstPtr solid_coordinate_system() { return primitive_ptr(SOLID_COORDINATE_SYSTEM); }

  static glk::Drawable::ConstPtr bunny() { return primitive_ptr(BUNNY); }

  static glk::Drawable::ConstPtr wire_icosahedron() { return primitive_ptr(WIRE_ICOSAHEDRON); }

  static glk::Drawable::ConstPtr wire_sphere() { return primitive_ptr(WIRE_SPHERE); }

  static glk::Drawable::ConstPtr wire_cube() { return primitive_ptr(WIRE_CUBE); }

  static glk::Drawable::ConstPtr wire_cone() { return primitive_ptr(WIRE_CONE); }

  static glk::Drawable::ConstPtr wire_bunny() { return primitive_ptr(WIRE_BUNNY); }

  static glk::Drawable::ConstPtr wire_frustum() { return primitive_ptr(WIRE_FRUSTUM); }

private:
  const glk::Drawable& create_primitive(PrimitiveType type);
  std::shared_ptr<glk::Drawable> create_primitive_ptr(PrimitiveType type);

private:
  GLK_API static Primitives* instance_;

  std::vector<std::shared_ptr<glk::Drawable>> meshes;
};
}  // namespace glk

#endif