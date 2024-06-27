#include <glk/io/mesh_io.hpp>

#include <filesystem>
#include <glk/mesh.hpp>
#include <glk/mesh_model.hpp>
#include <glk/io/image_io.hpp>
#include <glk/texture.hpp>
#include <guik/viewer/shader_setting.hpp>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

namespace glk {

std::shared_ptr<MeshModel> load_mesh_model(const std::string& path) {
  Assimp::Importer importer;
  const auto scene =
    importer.ReadFile(path, aiProcess_CalcTangentSpace | aiProcess_GenSmoothNormals | aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);
  if (!scene) {
    std::cerr << "error: failed to open " << path << std::endl;
    std::cerr << "     : " << importer.GetErrorString() << std::endl;
    return nullptr;
  }

  if (!scene->HasMeshes()) {
    std::cerr << "error: no meshes in " << path << std::endl;
    return nullptr;
  }

  auto model = std::make_shared<MeshModel>();

  for (int material_index = 0; material_index < scene->mNumMaterials; material_index++) {
    auto shader_setting = guik::FlatColor(0.0f, 0.0f, 0.0f, 1.0f);
    std::shared_ptr<glk::Texture> texture;

    const auto& material = scene->mMaterials[material_index][0];
    aiColor3D color(0.0, 0.0, 0.0);
    if (material.Get(AI_MATKEY_COLOR_DIFFUSE, color) == AI_SUCCESS) {
      shader_setting.add("material_color", Eigen::Vector4f(color.r, color.g, color.b, 1.0f));
    }

    aiString tex_path;
    if (material.GetTexture(aiTextureType_DIFFUSE, 0, &tex_path) == AI_SUCCESS) {
      const auto ends_with = [](const std::string& s, const std::string& suffix) {
        return s.size() < suffix.size() ? false : std::equal(std::rbegin(suffix), std::rend(suffix), std::rbegin(s));
      };

      const std::string filename = std::filesystem::path(path).parent_path().string() + "/" + std::string(tex_path.C_Str());

      int width, height;
      std::vector<unsigned char> pixels;
      if (glk::load_image(filename, width, height, pixels)) {
        shader_setting.add("color_mode", guik::ColorMode::TEXTURE_COLOR);
        texture = std::make_shared<glk::Texture>(Eigen::Vector2i(width, height), GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
      }
    }

    model->push_material(shader_setting, texture);
  }

  for (int mesh_index = 0; mesh_index < scene->mNumMeshes; mesh_index++) {
    const auto& mesh = scene->mMeshes[mesh_index];

    std::vector<unsigned int> faces;
    faces.reserve(mesh->mNumFaces * 3);
    for (int i = 0; i < mesh->mNumFaces; i++) {
      const auto& face = mesh->mFaces[i];
      faces.emplace_back(face.mIndices[0]);
      faces.emplace_back(face.mIndices[1]);
      faces.emplace_back(face.mIndices[2]);
    }

    std::vector<Eigen::Vector3f> vertices;
    std::vector<Eigen::Vector3f> normals;
    std::vector<Eigen::Vector4f> colors;
    std::vector<Eigen::Vector2f> tex_coords;

    if (mesh->mVertices) {
      vertices.reserve(mesh->mNumVertices);
      for (int i = 0; i < mesh->mNumVertices; i++) {
        const auto& v = mesh->mVertices[i];
        vertices.emplace_back(v.x, v.y, v.z);
      }
    }
    if (mesh->mNormals) {
      normals.reserve(mesh->mNumVertices);
      for (int i = 0; i < mesh->mNumVertices; i++) {
        const auto& n = mesh->mNormals[i];
        normals.emplace_back(n.x, n.y, n.z);
      }
    }
    if (mesh->mColors[0]) {
      colors.reserve(mesh->mNumVertices);
      for (int i = 0; i < mesh->mNumVertices; i++) {
        const auto& c = mesh->mColors[0][i];
        colors.emplace_back(c.r, c.g, c.b, c.a);
      }
    }
    if (mesh->mTextureCoords[0]) {
      tex_coords.reserve(mesh->mNumVertices);
      for (int i = 0; i < mesh->mNumVertices; i++) {
        const auto& t = mesh->mTextureCoords[0][i];
        tex_coords.emplace_back(t.x, 1.0 - t.y);
      }
    }

    model->push_mesh(
      mesh->mMaterialIndex,
      std::make_shared<glk::Mesh>(
        vertices.data(),
        sizeof(float) * 3,
        normals.data(),
        sizeof(float) * 3,
        colors.data(),
        sizeof(float) * 4,
        tex_coords.data(),
        sizeof(float) * 2,
        vertices.size(),
        faces.data(),
        faces.size()));
  }

  return model;
}
}  // namespace glk
