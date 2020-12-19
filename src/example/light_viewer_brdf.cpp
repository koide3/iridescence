#include <glk/primitives/primitives.hpp>
#include <glk/effects/screen_space_lighting.hpp>

#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();
  viewer->show_info_window();

  auto effect = std::make_shared<glk::ScreenSpaceLighting>(viewer->canvas_size());
  viewer->set_screen_effect(effect);

  std::vector<const char*> diffuse_models = {"ZERO", "ONE", "LAMBERT", "DISNEY", "NORMALIZED_DISNEY", "OPEN_NAYAR"};
  std::vector<const char*> specular_models = {"ZERO", "PHONG", "BLINN_PHONG", "COOK_TORRANCE"};
  std::vector<const char*> occlusion_models = {"ZERO", "AMBIENT_OCCLUSION"};
  int diffuse_model = 5;
  int specular_model = 3;
  int occlusion_model = 1;
  effect->set_diffuse_model(static_cast<glk::ScreenSpaceLighting::DIFFUSE_MODEL>(diffuse_model));
  effect->set_specular_model(static_cast<glk::ScreenSpaceLighting::SPECULAR_MODEL>(specular_model));
  effect->set_occlusion_model(static_cast<glk::ScreenSpaceLighting::OCCLUSION_MODEL>(occlusion_model));

  float roughness = 0.5f;
  float albedo = 1.0f;
  effect->set_albedo(albedo);
  effect->set_roughness(roughness);

  viewer->register_ui_callback("ui", [&] {
    if(ImGui::Combo("diffuse", &diffuse_model, diffuse_models.data(), diffuse_models.size())) {
      effect->set_diffuse_model(static_cast<glk::ScreenSpaceLighting::DIFFUSE_MODEL>(diffuse_model));
    }

    if(ImGui::Combo("specular", &specular_model, specular_models.data(), specular_models.size())) {
      effect->set_specular_model(static_cast<glk::ScreenSpaceLighting::SPECULAR_MODEL>(specular_model));
    }

    if(ImGui::Combo("occlusion", &occlusion_model, occlusion_models.data(), occlusion_models.size())) {
      effect->set_occlusion_model(static_cast<glk::ScreenSpaceLighting::OCCLUSION_MODEL>(occlusion_model));
    }

    if(ImGui::DragFloat("roughness", &roughness, 0.01f, 0.0f, 10.0f)) {
      effect->set_roughness(roughness);
    }
    if(ImGui::DragFloat("albedo", &albedo, 0.01f, 0.0f, 10.0f)) {
      effect->set_albedo(albedo);
    }
  });

  viewer->update_drawable("floor", glk::Primitives::primitive_ptr(glk::Primitives::CUBE), guik::FlatColor(Eigen::Vector4f::Ones(), (Eigen::Scaling<float>(25.0f, 25.0f, 0.1f) * Eigen::Isometry3f::Identity()).matrix()));
  Eigen::Matrix4f transform = (Eigen::Translation3f(0.0f, 0.0f, -0.5f) * Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX()) * Eigen::UniformScaling<float>(15.0f) * Eigen::Isometry3f::Identity()).matrix();
  viewer->update_drawable("bunny", glk::Primitives::primitive_ptr(glk::Primitives::BUNNY), guik::FlatColor(Eigen::Vector4f::Ones(), transform));

  const int num_lights = 8;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> light_colors(num_lights);
  for(int i = 0; i < num_lights; i++) {
    light_colors[i] = glk::colormap_categoricalf(glk::COLORMAP::TURBO, i, num_lights);
  }

  while(viewer->spin_once()) {
    const double t = ImGui::GetTime();
    for(int i = 0; i < num_lights; i++) {
      double theta = t + i * 2.0 * M_PI / num_lights;
      Eigen::Vector3f light_pos (10.0 * std::cos(theta), 10.0 * std::sin(theta), 2.0f);
      effect->set_light(i, light_pos, light_colors[i]);

      Eigen::Affine3f model_matrix = Eigen::Translation3f(light_pos) * Eigen::UniformScaling<float>(0.25f) * Eigen::Isometry3f::Identity();
      viewer->update_drawable("light_" + std::to_string(i), glk::Primitives::sphere(), guik::FlatColor(light_colors[i], model_matrix));
    }
    ImGui::GetTime();
  }
  return 0;
}