#ifndef GLK_DRAWABLE_CONTAINER_HPP
#define GLK_DRAWABLE_CONTAINER_HPP

#include <vector>
#include <Eigen/Core>

#include <glk/drawable.hpp>

namespace guik {
struct ShaderSetting;
}

namespace glk {

class DrawableContainer : public glk::Drawable {
public:
  using Ptr = std::shared_ptr<Drawable>;
  using ConstPtr = std::shared_ptr<const Drawable>;

  DrawableContainer(bool skip_model_matrix_setting = true);

  DrawableContainer(std::initializer_list<glk::Drawable::ConstPtr> init, bool skip_model_matrix_setting = true);

  virtual ~DrawableContainer();

  size_t size() const;
  void clear();
  void push_back(const glk::Drawable::ConstPtr& drawable);
  void push_back(const glk::Drawable::ConstPtr& drawable, const guik::ShaderSetting& shader_setting);

  virtual void draw(glk::GLSLShader& shader) const override;

public:
  bool skip_model_matrix;
  std::vector<std::pair<std::optional<guik::ShaderSetting>, glk::Drawable::ConstPtr>> drawables;
};

}  // namespace glk
#endif