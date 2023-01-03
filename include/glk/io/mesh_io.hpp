#ifndef GLK_MESH_IO_HPP
#define GLK_MESH_IO_HPP

#include <glk/mesh_model.hpp>

namespace glk {

std::shared_ptr<MeshModel> load_mesh_model(const std::string& path);

}  // namespace glk

#endif