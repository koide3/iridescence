#include <glk/mesh_model.hpp>

#include <glk/mesh.hpp>
#include <guik/viewer/shader_setting.hpp>

namespace glk {

void MeshModel::push_mesh(const int material_id, const std::shared_ptr<glk::Mesh>& mesh) {
  material_ids.emplace_back(material_id);
  meshes.emplace_back(mesh);
}

void MeshModel::push_material(const guik::ShaderSetting& setting, const std::shared_ptr<glk::Texture>& texture) {
  settings.emplace_back(setting.clone());
  textures.emplace_back(texture);
}

void MeshModel::override_material(const guik::ShaderSetting& setting, const std::shared_ptr<glk::Texture>& texture) {
  for (auto& s : this->settings) {
    s = setting.clone();
  }
  for (auto& tex : this->textures) {
    tex = texture;
  }
}

void MeshModel::draw(glk::GLSLShader& shader) const {
  for (int i = 0; i < meshes.size(); i++) {
    const auto material_id = material_ids[i];
    if (material_id < settings.size()) {
      if (textures[material_id]) {
        textures[material_id]->bind(GL_TEXTURE1);
      }

      for (int i = 0; i < settings[material_id].params.size(); i++) {
        if (i == 1 || i == 2) {
          // Ignore point_scale and model_matrix
          continue;
        }

        settings[material_id].params[i].set(shader);
      }
    }

    meshes[i]->draw(shader);
  }
}

}  // namespace glk
