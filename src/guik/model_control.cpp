#include <guik/model_control.hpp>

#include <vector>
#include <sstream>

#include <imgui.h>
#include <ImGuizmo.h>

#include <Eigen/Core>

#include <guik/viewer/light_viewer.hpp>

namespace guik {

ModelControl::ModelControl(const std::string& name, const Eigen::Matrix4f& init_model_matrix)
: name(name),
  pose(init_model_matrix),
  gizmo_enabled(true),
  gizmo_operation(ImGuizmo::OPERATION::TRANSLATE | ImGuizmo::OPERATION::ROTATE),
  gizmo_mode(ImGuizmo::MODE::LOCAL),
  gizmo_clip_space(0.1f) {}

void ModelControl::draw_ui() {
  ImGui::Begin(name.c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize);

  draw_gizmo_ui();

  float value = 0.0f;
  ImGui::SetNextItemWidth(50);
  if (ImGui::DragFloat("##PX", &value, 0.01f, 0.0f, 0.0f, "PX")) {
    pose = pose * Eigen::Translation3f(Eigen::Vector3f::UnitX() * value);
  }

  value = 0.0f;
  ImGui::SameLine();
  ImGui::SetNextItemWidth(50);
  if (ImGui::DragFloat("##PY", &value, 0.01f, 0.0f, 0.0f, "PY")) {
    pose = pose * Eigen::Translation3f(Eigen::Vector3f::UnitY() * value);
  }

  value = 0.0f;
  ImGui::SameLine();
  ImGui::SetNextItemWidth(50);
  if (ImGui::DragFloat("##PZ", &value, 0.01f, 0.0f, 0.0f, "PZ")) {
    pose = pose * Eigen::Translation3f(Eigen::Vector3f::UnitZ() * value);
  }

  value = 0.0f;
  ImGui::SetNextItemWidth(50);
  if (ImGui::DragFloat("##RX", &value, 0.01f, 0.0f, 0.0f, "RX")) {
    pose = pose * Eigen::AngleAxisf(value, Eigen::Vector3f::UnitX());
  }

  value = 0.0f;
  ImGui::SameLine();
  ImGui::SetNextItemWidth(50);
  if (ImGui::DragFloat("##RY", &value, 0.01f, 0.0f, 0.0f, "RY")) {
    pose = pose * Eigen::AngleAxisf(value, Eigen::Vector3f::UnitY());
  }

  value = 0.0f;
  ImGui::SameLine();
  ImGui::SetNextItemWidth(50);
  if (ImGui::DragFloat("##RZ", &value, 0.01f, 0.0f, 0.0f, "RZ")) {
    pose = pose * Eigen::AngleAxisf(value, Eigen::Vector3f::UnitZ());
  }

  value = 0.0f;
  ImGui::SetNextItemWidth(100);
  if (ImGui::DragFloat("##SCALE", &value, 0.01f, 0.0f, 0.0f, "SCALE")) {
    pose = pose * Eigen::UniformScaling<float>(1.0f + value);
  }

  std::stringstream sst;
  sst << "trans:" << pose.translation().transpose() << "\n"
      << "quat :" << Eigen::Quaternionf(pose.rotation()).coeffs().transpose() << "\n"
      << "scale:" << pose.linear().colwise().norm() << std::endl;
  ImGui::Text("%s", sst.str().c_str());

  ImGui::End();
}

void ModelControl::draw_gizmo_ui() {
  bool trans = gizmo_operation & ImGuizmo::TRANSLATE;
  bool rotate = gizmo_operation & ImGuizmo::ROTATE;
  bool scale = gizmo_operation & ImGuizmo::SCALEU;

  if (ImGui::Checkbox("Translate", &trans)) {
    if (trans) {
      gizmo_operation |= ImGuizmo::TRANSLATE;
    } else {
      gizmo_operation &= ~ImGuizmo::TRANSLATE;
    }
  }

  ImGui::SameLine();
  if (ImGui::Checkbox("Rotate", &rotate)) {
    if (rotate) {
      gizmo_operation |= ImGuizmo::ROTATE;
    } else {
      gizmo_operation &= ~ImGuizmo::ROTATE;
    }
  }

  ImGui::SameLine();
  if (ImGui::Checkbox("Scale", &scale)) {
    if (scale) {
      gizmo_operation |= ImGuizmo::SCALEU;
    } else {
      gizmo_operation &= ~ImGuizmo::SCALEU;
    }
  }
}

void ModelControl::draw_gizmo() {
  auto viewer = guik::LightViewer::instance();
  auto size = viewer->canvas_size();

  draw_gizmo(0, 0, size[0], size[1], viewer->view_matrix(), viewer->projection_matrix(), false);
}

void ModelControl::draw_gizmo(int win_x, int win_y, int win_w, int win_h, const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection, bool on_window) {
  ImGuizmo::Enable(gizmo_enabled);
  if(on_window) {
    ImGuizmo::SetDrawlist();
  }
  ImGuizmo::SetRect(win_x, win_y, win_w, win_h);

  Eigen::Matrix4f model = pose.matrix();
  Eigen::Matrix4f delta = Eigen::Matrix4f::Identity();
  ImGuizmo::Manipulate(view.data(), projection.data(), static_cast<ImGuizmo::OPERATION>(gizmo_operation), static_cast<ImGuizmo::MODE>(gizmo_mode), model.data(), delta.data());

  pose = Eigen::Affine3f(model);
}

bool ModelControl::is_guizmo_using() const {
  return ImGuizmo::IsUsing();
}

const std::string& ModelControl::model_name() const { return name; }

Eigen::Matrix4f ModelControl::model_matrix() const { return pose.matrix(); }

void ModelControl::set_gizmo_enabled(bool enabled) {
  gizmo_enabled = enabled;
}

void ModelControl::enable_gizmo() {
  set_gizmo_enabled(true);
}

void ModelControl::disable_gizmo() {
  set_gizmo_enabled(false);
}

void ModelControl::set_gizmo_operation(const std::string& operation) {
  if(operation == "TRANSLATE") {
    gizmo_operation = ImGuizmo::OPERATION::TRANSLATE;
  } else if (operation == "ROTATE") {
    gizmo_operation = ImGuizmo::OPERATION::ROTATE;
  } else if (operation == "SCALE") {
    gizmo_operation = ImGuizmo::OPERATION::SCALE;
  } else if (operation == "SCALEU") {
    gizmo_operation = ImGuizmo::OPERATION::SCALEU;
  } else if (operation == "UNIVERSAL") {
    gizmo_operation = ImGuizmo::OPERATION::UNIVERSAL;
  } else {
    std::cerr << "warning: invalid gizmo operation " << operation << std::endl;
  }
}

void ModelControl::set_gizmo_operation(int operation) {
  gizmo_operation = operation;
}

void ModelControl::set_gizmo_mode(int mode) {
  if (mode < 0 || mode > ImGuizmo::MODE::WORLD) {
    std::cerr << "warning: invalid gizmo mode " << mode << std::endl;
    return;
  }

  gizmo_mode = mode;
}

void ModelControl::set_gizmo_clip_scale(float space) {
  ImGuizmo::SetGizmoSizeClipSpace(space);
}

}  // namespace guik
