#include <glk/path.hpp>
#include <glk/texture.hpp>
#include <glk/io/png_io.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/effects/screen_space_lighting.hpp>

#include <glk/io/png_io.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();
  viewer->enable_normal_buffer();

  auto effect = std::make_shared<glk::ScreenSpaceLighting>(viewer->canvas_size());
  viewer->set_screen_effect(effect);

  std::vector<const char*> diffuse_models = {"ZERO", "ONE", "LAMBERT", "DISNEY", "NORMALIZED_DISNEY", "OREN_NAYAR"};
  std::vector<const char*> specular_models = {"ZERO", "PHONG", "BLINN_PHONG", "COOK_TORRANCE"};
  std::vector<const char*> occlusion_models = {"ZERO", "AMBIENT_OCCLUSION"};
  std::vector<const char*> iridescence_models = {"ZERO", "IRIDESCENCE1", "IRIDESCENCE2", "IRIDESCENCE3"};
  int diffuse_model = 5;
  int specular_model = 3;
  int occlusion_model = 1;
  int iridescence_model = 1;
  effect->set_diffuse_model(static_cast<glk::ScreenSpaceLighting::DIFFUSE_MODEL>(diffuse_model));
  effect->set_specular_model(static_cast<glk::ScreenSpaceLighting::SPECULAR_MODEL>(specular_model));
  effect->set_occlusion_model(static_cast<glk::ScreenSpaceLighting::OCCLUSION_MODEL>(occlusion_model));
  effect->set_iridescence_model(static_cast<glk::ScreenSpaceLighting::IRIDESCENCE_MODEL>(iridescence_model));

  float roughness = 0.1f;
  float albedo = 1.0f;
  effect->set_albedo(albedo);
  effect->set_roughness(roughness);

  bool rotate_lights = true;
  const int num_lights = 8;
  std::vector<Eigen::Vector4f> light_colors(num_lights);
  for (int i = 0; i < num_lights; i++) {
    light_colors[i] = glk::colormap_categoricalf(glk::COLORMAP::TURBO, i, num_lights);
  }

  viewer->register_ui_callback("ui", [&] {
    if (ImGui::Combo("diffuse", &diffuse_model, diffuse_models.data(), diffuse_models.size())) {
      effect->set_diffuse_model(static_cast<glk::ScreenSpaceLighting::DIFFUSE_MODEL>(diffuse_model));
    }

    if (ImGui::Combo("specular", &specular_model, specular_models.data(), specular_models.size())) {
      effect->set_specular_model(static_cast<glk::ScreenSpaceLighting::SPECULAR_MODEL>(specular_model));
    }

    if (ImGui::Combo("occlusion", &occlusion_model, occlusion_models.data(), occlusion_models.size())) {
      effect->set_occlusion_model(static_cast<glk::ScreenSpaceLighting::OCCLUSION_MODEL>(occlusion_model));
    }

    if (ImGui::Combo("iridescence", &iridescence_model, iridescence_models.data(), iridescence_models.size())) {
      effect->set_iridescence_model(static_cast<glk::ScreenSpaceLighting::IRIDESCENCE_MODEL>(iridescence_model));
    }

    if (ImGui::DragFloat("roughness", &roughness, 0.01f, 0.01f, 10.0f)) {
      effect->set_roughness(roughness);
    }
    if (ImGui::DragFloat("albedo", &albedo, 0.01f, 0.0f, 10.0f)) {
      effect->set_albedo(albedo);
    }

    ImGui::Separator();
    ImGui::Checkbox("rotate", &rotate_lights);

    ImGui::SameLine();
    if (ImGui::Button("rainbow")) {
      for (int i = 0; i < num_lights; i++) {
        light_colors[i] = glk::colormap_categoricalf(glk::COLORMAP::TURBO, i, num_lights);
      }
    }

    ImGui::SameLine();
    if (ImGui::Button("flat")) {
      for (int i = 0; i < num_lights; i++) {
        light_colors[i].setConstant(0.8f);
      }
    }
  });

  viewer->update_drawable("floor", glk::Primitives::cube(), guik::FlatColor(Eigen::Vector4f::Ones()).scale(25.0f, 25.0f, 0.1f));
  viewer->update_drawable(
    "bunny",
    glk::Primitives::bunny(),
    guik::FlatColor(Eigen::Vector4f::Ones(), Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX()) * Eigen::UniformScaling<float>(15.0f)));

  double t = 0.0;
  while (viewer->spin_once()) {
    t += rotate_lights ? ImGui::GetIO().DeltaTime : 0.0f;
    for (int i = 0; i < num_lights; i++) {
      double theta = t + i * 2.0 * M_PI / num_lights;
      Eigen::Vector3f light_pos(10.0 * std::cos(theta), 10.0 * std::sin(theta), 2.0f);
      Eigen::Vector2f attenuation(0.0f, 0.001f);
      float max_distance = 100.0f;
      effect->set_light(i, light_pos, light_colors[i], attenuation, max_distance);

      Eigen::Affine3f model_matrix = Eigen::Translation3f(light_pos) * Eigen::UniformScaling<float>(0.25f) * Eigen::Isometry3f::Identity();
      viewer->update_drawable("light_" + std::to_string(i), glk::Primitives::sphere(), guik::FlatColor(light_colors[i], model_matrix));
    }
  }
  return 0;
}