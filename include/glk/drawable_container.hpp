#ifndef GLK_DRAWABLE_CONTAINER_HPP
#define GLK_DRAWABLE_CONTAINER_HPP

#include <vector>
#include <Eigen/Core>

#include <glk/drawable.hpp>

namespace guik {
struct ShaderSetting;
}

namespace glk {

/// @brief Container to store multiple drawables and draw them in order.
///        It can also store a shader setting for each drawable.
class DrawableContainer : public glk::Drawable {
public:
  using Ptr = std::shared_ptr<Drawable>;
  using ConstPtr = std::shared_ptr<const Drawable>;

  /// @brief Constructor
  /// @param skip_model_matrix_setting  If true, the model matrix setting will be skipped when drawing the drawables.
  DrawableContainer(bool skip_model_matrix_setting = true);

  /// @brief Constructor
  /// @param init                      Initializer list of drawables to be added to the container.
  /// @param skip_model_matrix_setting If true, the model matrix setting will be skipped when drawing the drawables.
  DrawableContainer(std::initializer_list<glk::Drawable::ConstPtr> init, bool skip_model_matrix_setting = true);

  virtual ~DrawableContainer();

  /// @brief Number of drawables in the container.
  size_t size() const;

  /// @brief  Clear all drawables in the container.
  void clear();

  /// @brief Add a drawable to the container.
  /// @param drawable Drawable to be added.
  void push_back(const glk::Drawable::ConstPtr& drawable);

  /// @brief Add a drawable to the container with a shader setting.
  /// @param drawable  Drawable to be added.
  /// @param shader_setting Shader setting to be used when drawing the drawable.
  void push_back(const glk::Drawable::ConstPtr& drawable, const guik::ShaderSetting& shader_setting);

  virtual void draw(glk::GLSLShader& shader) const override;

public:
  bool skip_model_matrix;
  std::vector<std::pair<std::optional<guik::ShaderSetting>, glk::Drawable::ConstPtr>> drawables;
};

}  // namespace glk
#endif