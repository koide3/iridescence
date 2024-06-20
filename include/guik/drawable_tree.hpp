#ifndef GUIK_DRAWABLE_TREE_HPP
#define GUIK_DRAWABLE_TREE_HPP

#include <glk/drawable.hpp>
#include <guik/viewer/shader_setting.hpp>

namespace guik {

/**
@brief  Hierarchical drawable tree.
        Drawables are managed with slash-separated path names (e.g., "tree/child/grandchild/obj1").
        The model matrix of a node is hierarchically applied to its children.
        For example, for a tree "A/B/C", the model matrix of "C" becomes "A * B * C".
@example
        auto tree = std::make_shared<DrawableTree>();
        // Register drawables to the tree.
        tree->update_drawable("parent/child/node1", drawable1, guik::VertexColor(node1_matrix));
        tree->update_drawable("parent/child/node2", drawable2, guik::VertexColor(node2_matrix));
        // Update shader settings. This matrix is applied to both "node1" and "node2".
        tree->update_setting("parent/child", guik::ShaderSetting(child_matrix));

        // Overwrite the color settings of all children of "parent/child".
        tree->update_tree_color_mode("parent/child", guik::ColorMode::FLAT_COLOR);
        tree->update_tree_color("parent/child", Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f));

        // Overwrite the point scale of all children of "parent/child".
        tree->update_tree_setting("parent/child", "point_scale", 2.0f);

        // Output the tree structure to stdout for debugging.
        tree->print_tree();

        viewer->update_drawable("tree", tree);
*/
class DrawableTree : public glk::Drawable {
public:
  using Ptr = std::shared_ptr<DrawableTree>;
  using ConstPtr = std::shared_ptr<const DrawableTree>;

  DrawableTree();

  /// @brief Update drawable at a tree node.
  /// @param name       Slash-separated path name (e.g., "parent/child/node1")
  /// @param drawable   Drawable object
  /// @param setting    Shader setting
  void update_drawable(const std::string& name, glk::Drawable::ConstPtr drawable, const guik::ShaderSetting& setting);

  /// @brief Update shader setting of a node.
  void update_setting(const std::string& name, const guik::ShaderSetting& setting);

  /// @brief Update shader setting of all children of a node.
  template <typename T>
  void update_tree_setting(const std::string& name, const std::string& param_name, const T& val) {
    const auto names = split_path(name);
    update_tree_setting(names.data(), names.data() + names.size(), param_name, val);
  }

  /// @brief Update color mode of all children of a node.
  void update_tree_color_mode(const std::string& name, guik::ColorMode::MODE color_mode);

  /// @brief Update color of all children of a node.
  void update_tree_color(const std::string& name, const Eigen::Vector4f& color);

  /// @brief Remove a subtree.
  bool remove(const std::string& name);

  /// @brief Print the tree structure to stdout.
  void print_tree(int depth = 0, const Eigen::Matrix4f& parent_model_matrix = Eigen::Matrix4f::Identity()) const;

  /// @brief Draw the tree.
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
  const std::string name;                                       // Node name
  guik::ShaderSetting setting;                                  // Shader setting associated with the node
  glk::Drawable::ConstPtr drawable;                             // Drawable object associated with the node
  std::unordered_map<std::string, DrawableTree::Ptr> children;  // Child nodes
};

}  // namespace  guik

#endif