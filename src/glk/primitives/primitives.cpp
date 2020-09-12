#include <glk/primitives/primitives.hpp>

#include <iostream>
#include <glk/mesh.hpp>
#include <glk/lines.hpp>
#include <glk/mesh_utils.hpp>

#include <glk/primitives/grid.hpp>
#include <glk/primitives/cube.hpp>
#include <glk/primitives/cone.hpp>
#include <glk/primitives/icosahedron.hpp>
#include <glk/primitives/coordinate_system.hpp>
#include <glk/loaders/ply_loader.hpp>

namespace glk {

Primitives* Primitives::instance_ = nullptr;

const glk::Drawable& Primitives::create_primitive(PrimitiveType type) {
  if(meshes[type] == nullptr) {
    bool wireframe = int(type) >= WIRE_ICOSAHEDRON;

    switch(type) {
      default:
        std::cerr << "error : unknown primitive type " << type << std::endl;
        break;
      case ICOSAHEDRON:
      case WIRE_ICOSAHEDRON:
      {
        glk::Icosahedron icosahedron;
        glk::Flatize flat(icosahedron.vertices, icosahedron.indices);
        meshes[type].reset(new glk::Mesh(flat.vertices, flat.normals, flat.indices, wireframe));
      } break;
      case SPHERE:
      case WIRE_SPHERE:
      {
        glk::Icosahedron icosahedron;
        icosahedron.subdivide();
        icosahedron.subdivide();
        icosahedron.spherize();
        meshes[type].reset(new glk::Mesh(icosahedron.vertices, icosahedron.normals, icosahedron.indices, wireframe));
      } break;
      case CUBE:
      case WIRE_CUBE:
      {
        glk::Cube cube;
        glk::Flatize flat(cube.vertices, cube.indices);
        meshes[type].reset(new glk::Mesh(flat.vertices, flat.normals, flat.indices, wireframe));
      } break;
      case CONE:
      case WIRE_CONE:
      {
        glk::Cone cone;
        glk::Flatize flat(cone.vertices, cone.indices);
        meshes[type].reset(new glk::Mesh(flat.vertices, flat.normals, flat.indices, wireframe));
      } break;
      case GRID: {
        glk::Grid grid;
        meshes[type].reset(new glk::Lines(0.01f, grid.vertices));
      } break;
      case BUNNY:
      case WIRE_BUNNY:
      {
        glk::PLYLoader ply("data/models/bunny.ply");
        meshes[type].reset(new glk::Mesh(ply.vertices, ply.normals, ply.indices, wireframe));
      } break;
      case COORDINATE_SYSTEM: {
        glk::CoordinateSystem coord;
        meshes[type].reset(new glk::Lines(0.01f, coord.vertices, coord.colors));
      } break;
    }
  }

  return *meshes[type];
}

class PrimitiveWrapper : public glk::Drawable {
public:
  PrimitiveWrapper(const glk::Drawable& primitive)
  : primitive(primitive)
  {}

  virtual void draw(glk::GLSLShader& shader) const {
    primitive.draw(shader);
  }

private:
  const glk::Drawable& primitive;
};

std::shared_ptr<glk::Drawable> Primitives::create_primitive_ptr(PrimitiveType type) {
  return std::make_shared<PrimitiveWrapper>(primitive(type));
}

}  // namespace glk