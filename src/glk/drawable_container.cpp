#include <glk/drawable_container.hpp>

#include <guik/viewer/shader_setting.hpp>

namespace glk {

DrawableContainer::DrawableContainer(bool skip_model_matrix_setting) : skip_model_matrix(skip_model_matrix_setting) {}

DrawableContainer::DrawableContainer(std::initializer_list<glk::Drawable::ConstPtr> init, bool skip_model_matrix_setting) : skip_model_matrix(skip_model_matrix_setting) {
  drawables.resize(init.size());
  std::transform(init.begin(), init.end(), drawables.begin(), [](const glk::Drawable::ConstPtr& drawable) { return std::make_pair(std::nullopt, drawable); });
}

DrawableContainer ::~DrawableContainer() {}

size_t DrawableContainer::size() const {
  return drawables.size();
}

void DrawableContainer::clear() {
  drawables.clear();
}

void DrawableContainer::push_back(const glk::Drawable::ConstPtr& drawable) {
  drawables.emplace_back(std::make_pair(std::nullopt, drawable));
}

void DrawableContainer::push_back(const glk::Drawable::ConstPtr& drawable, const guik::ShaderSetting& shader_setting) {
  drawables.emplace_back(std::make_pair(shader_setting, drawable));
}

void DrawableContainer::draw(glk::GLSLShader& shader) const {
  for (const auto& drawable : drawables) {
    if (drawable.first) {
      const auto& params = drawable.first->params;
      for (int i = 0; i < params.size(); i++) {
        if (skip_model_matrix && i == 2) {
          continue;
        }

        params[i].set(shader);
      }
    }

    drawable.second->draw(shader);
  }
}

}  // namespace glk
