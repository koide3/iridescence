#include <glk/primitives/primitives.hpp>

#include <iostream>
#include <glk/path.hpp>
#include <glk/mesh.hpp>
#include <glk/lines.hpp>
#include <glk/thin_lines.hpp>
#include <glk/mesh_utils.hpp>
#include <glk/console_colors.hpp>

#include <glk/primitives/grid.hpp>
#include <glk/primitives/cube.hpp>
#include <glk/primitives/cone.hpp>
#include <glk/primitives/icosahedron.hpp>
#include <glk/primitives/coordinate_system.hpp>
#include <glk/primitives/frustum.hpp>
#include <glk/io/ply_io.hpp>

namespace glk {

using namespace glk::console;

GLK_API Primitives* Primitives::instance_ = nullptr;

const glk::Drawable& Primitives::create_primitive(PrimitiveType type) {
  if (meshes[type] == nullptr) {
    bool wireframe = int(type) >= WIRE_ICOSAHEDRON;

    switch (type) {
      default:
        std::cerr << bold_red << "error : unknown primitive type " << type << reset << std::endl;
        break;
      case ICOSAHEDRON:
      case WIRE_ICOSAHEDRON: {
        glk::Icosahedron icosahedron;
        glk::Flatize flat(icosahedron.vertices, icosahedron.indices);
        meshes[type].reset(new glk::Mesh(flat.vertices, flat.normals, flat.indices, wireframe));
      } break;
      case SPHERE:
      case WIRE_SPHERE: {
        glk::Icosahedron icosahedron;
        icosahedron.subdivide();
        icosahedron.subdivide();
        icosahedron.spherize();
        meshes[type].reset(new glk::Mesh(icosahedron.vertices, icosahedron.normals, icosahedron.indices, wireframe));
      } break;
      case CUBE:
      case WIRE_CUBE: {
        glk::Cube cube;
        glk::Flatize flat(cube.vertices, cube.indices);
        meshes[type].reset(new glk::Mesh(flat.vertices, flat.normals, flat.indices, wireframe));
      } break;
      case CONE:
      case WIRE_CONE: {
        glk::Cone cone;
        glk::Flatize flat(cone.vertices, cone.indices);
        meshes[type].reset(new glk::Mesh(flat.vertices, flat.normals, flat.indices, wireframe));
      } break;
      case GRID: {
        glk::Grid grid;
        meshes[type].reset(new glk::Lines(0.01f, grid.vertices));
      } break;
      case BUNNY:
      case WIRE_BUNNY: {
        auto ply = load_ply(get_data_path() + "/models/bunny.ply");
        meshes[type].reset(new glk::Mesh(ply->vertices, ply->normals, ply->indices, wireframe));
      } break;
      case COORDINATE_SYSTEM: {
        glk::CoordinateSystem coord;
        auto lines = std::make_shared<glk::ThinLines>(coord.vertices, coord.colors);
        lines->set_line_width(2.5f);
        meshes[type] = lines;
      } break;
      case SOLID_COORDINATE_SYSTEM: {
        glk::CoordinateSystem coord;
        meshes[type].reset(new glk::Lines(0.01f, coord.vertices, coord.colors));
      } break;
      case FRUSTUM:
      case WIRE_FRUSTUM: {
        glk::Frustum frustum(0.6f, 0.4f, 0.5f, !wireframe);
        meshes[type].reset(new glk::Mesh(frustum.vertices, frustum.normals, frustum.indices, wireframe));
      } break;
    }
  }

  return *meshes[type];
}

class PrimitiveWrapper : public glk::Drawable {
public:
  PrimitiveWrapper(const glk::Drawable& primitive) : primitive(primitive) {}

  virtual void draw(glk::GLSLShader& shader) const { primitive.draw(shader); }

private:
  const glk::Drawable& primitive;
};

std::shared_ptr<glk::Drawable> Primitives::create_primitive_ptr(PrimitiveType type) {
  return std::make_shared<PrimitiveWrapper>(primitive(type));
}

}  // namespace glk