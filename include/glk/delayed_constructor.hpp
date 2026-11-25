#ifndef GLK_DELAYED_CONSTRUCTOR_HPP
#define GLK_DELAYED_CONSTRUCTOR_HPP

#include <memory>
#include <iostream>
#include <functional>
#include <glk/drawable.hpp>
#include <glk/console_colors.hpp>

namespace glk {

/// @brief Drawable that constructs the actual drawable on the first draw() call.
///        This allows creating a drawable in a thread without an OpenGL context (e.g., to pass it to AsyncLightViewer),
///        while the OpenGL resources are created in the rendering thread.
/// @note  The factory is called at most once. If it throws, an error is printed and nothing is drawn afterward.
class DelayedConstructor : public glk::Drawable {
public:
  using Ptr = std::shared_ptr<DelayedConstructor>;
  using ConstPtr = std::shared_ptr<const DelayedConstructor>;
  using Factory = std::function<glk::Drawable::ConstPtr()>;

  /// @brief Constructor
  /// @param factory  Function to construct the actual drawable. Called in the rendering thread upon the first draw().
  explicit DelayedConstructor(const Factory& factory) : factory(factory) {}

  /// @brief Create a DelayedConstructor that constructs T(args...) upon the first draw().
  /// @note  Arguments are copied into the factory.
  template <typename T, typename... Args>
  static Ptr create(Args&&... args) {
    return std::make_shared<DelayedConstructor>([=]() -> glk::Drawable::ConstPtr { return std::make_shared<T>(args...); });
  }

  virtual ~DelayedConstructor() override {}

  /// @brief Construct the drawable if not yet constructed, and draw it.
  virtual void draw(glk::GLSLShader& shader) const override {
    construct();
    if (instance) {
      instance->draw(shader);
    }
  }

  /// @brief Construct the drawable if not yet constructed. Must be called in the thread with the OpenGL context.
  void construct() const {
    if (instance || !factory) {
      return;
    }

    // Release the factory before calling it so that it is never called again even if it throws
    Factory f;
    std::swap(f, factory);

    try {
      instance = f();
    } catch (const std::exception& e) {
      std::cerr << console::bold_red << "error: failed to construct a delayed drawable (" << e.what() << ")" << console::reset << std::endl;
    }
  }

  /// @brief Check if the drawable has been constructed.
  bool constructed() const { return instance != nullptr; }

  /// @brief Get the constructed drawable (nullptr if not constructed yet).
  const glk::Drawable::ConstPtr& get() const { return instance; }

private:
  mutable Factory factory;                   ///< Factory to construct the drawable (reset after the first call)
  mutable glk::Drawable::ConstPtr instance;  ///< Constructed drawable
};

}  // namespace glk

#endif
