#include <random>
#include <iostream>
#include <glk/path.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/effects/screen_effect.hpp>

#include <glk/effects/screen_space_lighting.hpp>
#include <glk/effects/screen_scape_attribute_estimation.hpp>

namespace glk {

ScreenSpaceLighting::ScreenSpaceLighting(const Eigen::Vector2i& size) {
  ssae.reset(new ScreenSpaceAttributeEstimation(size));

  diffuse_model = DIFFUSE_MODEL::LAMBERT;
  specular_model = SPECULAR_MODEL::PHONG;
  occlusion_model = OCCLUSION_MODEL::ZERO;

  if(!load_shader()) {
    return;
  }

  add_light(Eigen::Vector3f(-10.0f, -10.0f, 200.0f), Eigen::Vector4f(0.9f, 0.9f, 1.0f, 1.0f));

  lighting_shader.use();
  lighting_shader.set_uniform("albedo", 1.0f);
  lighting_shader.set_uniform("roughness", 0.2f);
  lighting_shader.set_uniform("ambient_light_color", Eigen::Vector4f::Zero().eval());
}
ScreenSpaceLighting::~ScreenSpaceLighting() {}

void ScreenSpaceLighting::set_diffuse_model(DIFFUSE_MODEL model) {
  diffuse_model = model;
  load_shader();
}

void ScreenSpaceLighting::set_specular_model(SPECULAR_MODEL model) {
  specular_model = model;
  load_shader();
}

void ScreenSpaceLighting::set_occlusion_model(OCCLUSION_MODEL model) {
  occlusion_model = model;
  load_shader();
}

bool ScreenSpaceLighting::load_shader() {
  light_updated = true;
  std::vector<std::string> vertex_shaders = {get_data_path() + "/shader/texture.vert"};
  std::vector<std::string> fragment_shaders = {get_data_path() + "/shader/ssli.frag", get_data_path() + "/shader/brdf/schlick_fresnel.frag"};

  switch(diffuse_model) {
    default:
    case DIFFUSE_MODEL::ZERO:
      fragment_shaders.push_back(get_data_path() + "/shader/brdf/diffuse_zero.frag");
      break;
    case DIFFUSE_MODEL::ONE:
      fragment_shaders.push_back(get_data_path() + "/shader/brdf/diffuse_one.frag");
      break;
    case DIFFUSE_MODEL::LAMBERT:
      fragment_shaders.push_back(get_data_path() + "/shader/brdf/diffuse_lambert.frag");
      break;
    case DIFFUSE_MODEL::DISNEY:
      fragment_shaders.push_back(get_data_path() + "/shader/brdf/diffuse_disney.frag");
      break;
    case DIFFUSE_MODEL::NORMALIZED_DISNEY:
      fragment_shaders.push_back(get_data_path() + "/shader/brdf/diffuse_disney_normalized.frag");
      break;
    case DIFFUSE_MODEL::OREN_NAYAR:
      fragment_shaders.push_back(get_data_path() + "/shader/brdf/diffuse_oren_nayar.frag");
      break;
  }

  switch(specular_model) {
    default:
    case SPECULAR_MODEL::ZERO:
      fragment_shaders.push_back(get_data_path() + "/shader/brdf/specular_zero.frag");
      break;
    case SPECULAR_MODEL::PHONG:
      fragment_shaders.push_back(get_data_path() + "/shader/brdf/specular_phong.frag");
      break;
    case SPECULAR_MODEL::BLINN_PHONG:
      fragment_shaders.push_back(get_data_path() + "/shader/brdf/specular_blinn_phong.frag");
      break;
    case SPECULAR_MODEL::COOK_TORRANCE:
      fragment_shaders.push_back(get_data_path() + "/shader/brdf/specular_cook_torrance.frag");
      break;
  }

  switch(occlusion_model) {
    default:
    case OCCLUSION_MODEL::ZERO:
      fragment_shaders.push_back(get_data_path() + "/shader/brdf/occlusion_zero.frag");
      break;
    case OCCLUSION_MODEL::AMBIENT_OCCLUSION:
      fragment_shaders.push_back(get_data_path() + "/shader/brdf/occlusion_ambient.frag");
      break;
  }

  return lighting_shader.init(vertex_shaders, fragment_shaders);
}

void ScreenSpaceLighting::set_albedo(float albedo) {
  this->albedo = albedo;
}

void ScreenSpaceLighting::set_roughness(float roughness) {
  this->roughness = roughness;
}

int ScreenSpaceLighting::num_lights() const {
  return light_pos.size();
}

void ScreenSpaceLighting::set_light(int i, const Eigen::Vector3f& pos, const Eigen::Vector4f& color) {
  while(i > light_pos.size() - 1) {
    light_pos.push_back(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    light_color.push_back(Eigen::Vector4f(0.0f, 0.0f, 0.0f, 0.0f));
  }

  light_updated = true;
  light_pos[i] = pos;
  light_color[i] = color;
}

void ScreenSpaceLighting::add_light(const Eigen::Vector3f& pos, const Eigen::Vector4f& color) {
  light_updated = true;
  light_pos.push_back(pos);
  light_color.push_back(color);
}

void ScreenSpaceLighting::set_size(const Eigen::Vector2i& size) {
  ssae->set_size(size);
}

void ScreenSpaceLighting::draw(const TextureRenderer& renderer, const glk::Texture& color_texture, const glk::Texture& depth_texture, const TextureRendererInput::Ptr& input) {
  ssae->draw(renderer, color_texture, depth_texture, input);

  auto view_matrix = input->get<Eigen::Matrix4f>("view_matrix");
  if(!view_matrix) {
    std::cerr << "view and projection matrices must be set" << std::endl;
    return;
  }
  Eigen::Vector3f view_point = view_matrix->inverse().block<3, 1>(0, 3);

  glEnable(GL_TEXTURE_2D);
  glDisable(GL_DEPTH_TEST);

  lighting_shader.use();
  lighting_shader.set_uniform("color_sampler", 0);
  lighting_shader.set_uniform("position_sampler", 1);
  lighting_shader.set_uniform("normal_sampler", 2);
  lighting_shader.set_uniform("occlusion_sampler", 3);

  lighting_shader.set_uniform("view_point", view_point);

  lighting_shader.set_uniform("albedo", albedo);
  lighting_shader.set_uniform("roughness", roughness);

  if(light_updated) {
    lighting_shader.set_uniform("num_lights", static_cast<int>(light_pos.size()));
    lighting_shader.set_uniform("light_pos", light_pos);
    lighting_shader.set_uniform("light_color", light_color);
  }

  color_texture.bind(GL_TEXTURE0);
  ssae->position().bind(GL_TEXTURE1);
  ssae->normal().bind(GL_TEXTURE2);
  ssae->occlusion().bind(GL_TEXTURE3);

  renderer.draw_plain(lighting_shader);

  glDisable(GL_TEXTURE_2D);
  glEnable(GL_DEPTH_TEST);
}

}  // namespace glk
