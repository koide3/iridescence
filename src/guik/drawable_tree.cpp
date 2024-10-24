#include <guik/drawable_tree.hpp>

namespace guik {

DrawableTree::DrawableTree() : name("root") {}

DrawableTree::DrawableTree(const std::string& name) : name(name) {}

void DrawableTree::update_drawable(const std::string& name, glk::Drawable::ConstPtr drawable, const guik::ShaderSetting& setting) {
  const auto names = split_path(name);
  update_drawable(names.data(), names.data() + names.size(), drawable, setting);
}

void DrawableTree::update_setting(const std::string& name, const guik::ShaderSetting& setting) {
  const auto names = split_path(name);
  update_drawable(names.data(), names.data() + names.size(), nullptr, setting);
}

void DrawableTree::update_tree_color_mode(const std::string& name, guik::ColorMode::MODE color_mode) {
  update_tree_setting(name, "color_mode", color_mode);
}
void DrawableTree::update_tree_color(const std::string& name, const Eigen::Vector4f& color) {
  update_tree_setting(name, "material_color", color);
}

bool DrawableTree::remove(const std::string& name) {
  const auto names = split_path(name);
  if (!remove(names.data(), names.data() + names.size())) {
    std::cerr << "warning: failed to remove subtree : " << name << std::endl;
    return false;
  }
  return true;
}

void DrawableTree::draw(glk::GLSLShader& shader) const {
  draw(shader, Eigen::Matrix4f::Identity());
}

void DrawableTree::print_tree(int depth, const Eigen::Matrix4f& parent_model_matrix) const {
  Eigen::Matrix4f model_matrix = setting.has_model_matrix() ? parent_model_matrix * setting.model_matrix() : parent_model_matrix;

  for (int i = 0; i < depth; i++) {
    std::cout << "  ";
  }

  Eigen::Quaternionf quat(model_matrix.block<3, 3>(0, 0));
  Eigen::Vector3f trans(model_matrix.block<3, 1>(0, 3));

  std::cout << name << " : trans=(" << trans.x() << "," << trans.y() << "," << trans.z() << ") quat=(" << quat.x() << "," << quat.y() << "," << quat.z() << "," << quat.w() << ")"
            << (drawable ? " D" : "") << std::endl;

  for (const auto& child : children) {
    child.second->print_tree(depth + 1, model_matrix);
  }
}

void DrawableTree::update_drawable(const std::string* first, const std::string* last, glk::Drawable::ConstPtr drawable, const guik::ShaderSetting& setting) {
  if (first == last) {
    this->setting = setting;
    if (drawable) {
      this->drawable = drawable;
    }
    return;
  }

  auto found = children.find(*first);
  if (found == children.end()) {
    auto child = DrawableTree::Ptr(new DrawableTree(*first));
    found = children.emplace_hint(found, *first, child);
  }

  found->second->update_drawable(first + 1, last, drawable, setting);
}

bool DrawableTree::remove(const std::string* first, const std::string* last) {
  if (first == last) {
    return false;
  }

  auto found = children.find(*first);
  if (found == children.end()) {
    return false;
  }

  if (first + 1 == last) {
    children.erase(found);
    return true;
  }

  return found->second->remove(first + 1, last);
}

void DrawableTree::draw(glk::GLSLShader& shader, const Eigen::Matrix4f& parent_model_matrix) const {
  Eigen::Matrix4f model_matrix = parent_model_matrix;
  if (setting.has_model_matrix()) {
    model_matrix = model_matrix * setting.model_matrix();
  }

  if (drawable) {
    setting.set(shader);
    shader.set_uniform("model_matrix", model_matrix);
    drawable->draw(shader);
  }

  for (const auto& child : children) {
    setting.set(shader);
    child.second->draw(shader, model_matrix);
  }
}

std::vector<std::string> DrawableTree::split_path(const std::string& path) {
  if (path.empty()) {
    std::cerr << "warning: empty path" << std::endl;
    return {};
  }

  std::vector<std::string> names;
  int cursor = path.find_first_not_of('/');
  while (cursor < path.size()) {
    int next = path.find('/', cursor);
    if (next == cursor) {
      std::cout << "warning: skip empty name : " << path.substr(0, cursor) << "()" << path.substr(cursor) << std::endl;
      cursor++;
      continue;
    }

    if (next == std::string::npos) {
      names.push_back(path.substr(cursor));
      break;
    }
    names.push_back(path.substr(cursor, next - cursor));
    cursor = next + 1;
  }

  return names;
}
}  // namespace guik
