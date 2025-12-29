#ifndef GLK_MESH_MODEL_HPP
#define GLK_MESH_MODEL_HPP

#include <glk/drawable.hpp>

namespace guik {
struct ShaderSetting;
}  // namespace guik

namespace glk {

class Mesh;
class Texture;

class MeshModel : public glk::Drawable {
public:
  void push_mesh(const int material_id, const std::shared_ptr<glk::Mesh>& mesh);
  void push_material(const guik::ShaderSetting& setting, const std::shared_ptr<glk::Texture>& texture);
  void override_material(const guik::ShaderSetting& setting, const std::shared_ptr<glk::Texture>& texture);

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  std::vector<int> material_ids;
  std::vector<std::shared_ptr<glk::Mesh>> meshes;

  std::vector<guik::ShaderSetting> settings;
  std::vector<std::shared_ptr<glk::Texture>> textures;
};

}  // namespace glk

#endif