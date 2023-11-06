#ifndef GLK_DRAWABLE_CONTAINER_HPP
#define GLK_DRAWABLE_CONTAINER_HPP

#include <vector>
#include <Eigen/Core>

#include <glk/drawable.hpp>

namespace guik {
class ShaderSetting;
}

namespace glk {

class DrawableContainer : public glk::Drawable {
public:
  using Ptr = std::shared_ptr<Drawable>;
  using ConstPtr = std::shared_ptr<const Drawable>;

  DrawableContainer();

  DrawableContainer(std::initializer_list<glk::Drawable::ConstPtr> init);

  template <typename ConstIterator>
  DrawableContainer(ConstIterator first, ConstIterator last) {
    drawables.reserve(std::distance(first, last));
    std::transform(first, last, std::back_inserter(drawables), [](const glk::Drawable::ConstPtr& drawable) { return std::make_pair(std::nullopt, drawable); });
  }

  virtual ~DrawableContainer();

  size_t size() const;
  void clear();
  void push_back(const glk::Drawable::ConstPtr& drawable);
  void push_back(const glk::Drawable::ConstPtr& drawable, const guik::ShaderSetting& shader_setting);

  virtual void draw(glk::GLSLShader& shader) const override;

public:
  std::vector<std::pair<std::optional<guik::ShaderSetting>, glk::Drawable::ConstPtr>> drawables;
};

}  // namespace glk
#endif