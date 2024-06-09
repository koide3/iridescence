#ifndef GUIK_DRAWABLE_TREE_HPP
#define GUIK_DRAWABLE_TREE_HPP

#include <glk/drawable.hpp>
#include <guik/viewer/shader_setting.hpp>

namespace guik {

/// @brief Hierarchical drawable tree
class DrawableTree : public glk::Drawable {
public:
  using Ptr = std::shared_ptr<DrawableTree>;
  using ConstPtr = std::shared_ptr<const DrawableTree>;

  DrawableTree();

  void update_drawable(const std::string& name, glk::Drawable::ConstPtr drawable, const guik::ShaderSetting& setting);
  void update_setting(const std::string& name, const guik::ShaderSetting& setting);

  template <typename T>
  void update_tree_setting(const std::string& name, const std::string& param_name, const T& val) {
    const auto names = split_path(name);
    update_tree_setting(names.data(), names.data() + names.size(), param_name, val);
  }

  void update_tree_color_mode(const std::string& name, guik::ColorMode::MODE color_mode);
  void update_tree_color(const std::string& name, const Eigen::Vector4f& color);

  bool remove(const std::string& name);

  void print_tree(int depth = 0, const Eigen::Matrix4f& parent_model_matrix = Eigen::Matrix4f::Identity()) const;

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  DrawableTree(const std::string& name);

  void update_drawable(const std::string* first, const std::string* last, glk::Drawable::ConstPtr drawable, const guik::ShaderSetting& setting);

  bool remove(const std::string* first, const std::string* last);

  template <typename T>
  void update_tree_setting(const std::string* first, const std::string* last, const std::string& param_name, const T& val) {
    if (first == last) {
      setting.add<T>(param_name, val);
      for (auto child : children) {
        child.second->update_tree_setting<T>(first, last, param_name, val);
      }
    } else {
      auto found = children.find(*first);
      if (found == children.end()) {
        return;
      }

      found->second->update_tree_setting<T>(first + 1, last, param_name, val);
    }
  }

  void draw(glk::GLSLShader& shader, const Eigen::Matrix4f& parent_model_matrix) const;

private:
  std::vector<std::string> split_path(const std::string& path);

private:
  const std::string name;
  guik::ShaderSetting setting;

  glk::Drawable::ConstPtr drawable;
  std::unordered_map<std::string, DrawableTree::Ptr> children;
};

}  // namespace  guik

#endif