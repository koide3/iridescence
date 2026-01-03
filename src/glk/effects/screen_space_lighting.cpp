#include <random>
#include <iostream>
#include <glk/path.hpp>
#include <glk/console_colors.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/io/png_io.hpp>
#include <glk/effects/screen_effect.hpp>

#include <glk/effects/screen_space_lighting.hpp>
#include <glk/effects/screen_space_splatting.hpp>
#include <glk/effects/screen_scape_attribute_estimation.hpp>

#include <guik/viewer/light_viewer.hpp>

namespace glk {

ScreenSpaceLighting::ScreenSpaceLighting(const Eigen::Vector2i& size, bool use_splatting) {
  if (use_splatting) {
    splatting.reset(new ScreenSpaceSplatting(size));
  } else {
    ssae.reset(new ScreenSpaceAttributeEstimation(size));
  }

  int width, height;
  std::vector<unsigned char> bytes;

  if (!load_png(get_data_path() + "/texture/iridescence1.png", width, height, bytes)) {
    return;
  }
  iridescence_texture.reset(new glk::Texture(Eigen::Vector2i(width, height), GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, bytes.data()));

  diffuse_model = DIFFUSE_MODEL::OREN_NAYAR;
  specular_model = SPECULAR_MODEL::COOK_TORRANCE;
  occlusion_model = OCCLUSION_MODEL::AMBIENT_OCCLUSION;
  iridescence_model = IRIDESCENCE_MODEL::IRIDESCENCE1;

  if (!load_shader()) {
    return;
  }

  set_directional_light(0, Eigen::Vector3f(0.4f, 0.1f, -1.0f), Eigen::Vector4f(2.0f, 2.0f, 2.0f, 1.0f));
  // set_light(0, Eigen::Vector3f(0.0f, 0.0f, 50.0f), Eigen::Vector4f(2.0f, 2.0f, 2.0f, 1.0f), Eigen::Vector2f(0.0f, 0.0f), 100.0f);

  albedo = 1.0f;
  roughness = 0.2f;

  lighting_shader.use();
  lighting_shader.set_uniform("albedo", 1.0f);
  lighting_shader.set_uniform("roughness", 0.2f);
  lighting_shader.set_uniform("ambient_light_color", Eigen::Vector4f(0.1f, 0.1f, 0.1f, 1.0f));
}
ScreenSpaceLighting::~ScreenSpaceLighting() {}

const glk::Texture& ScreenSpaceLighting::position() const {
  return ssae->position();
}

const glk::Texture& ScreenSpaceLighting::normal() const {
  return ssae->normal();
}

const glk::Texture& ScreenSpaceLighting::occlusion() const {
  return ssae->occlusion();
}

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

void ScreenSpaceLighting::set_iridescence_model(IRIDESCENCE_MODEL model) {
  iridescence_model = model;
  load_shader();
}

bool ScreenSpaceLighting::load_shader() {
  light_updated = true;
  std::vector<std::string> vertex_shaders = {get_data_path() + "/shader/texture.vert"};
  std::vector<std::string> fragment_shaders = {get_data_path() + "/shader/ssli.frag", get_data_path() + "/shader/brdf/schlick_fresnel.frag"};

  switch (diffuse_model) {
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

  switch (specular_model) {
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

  switch (occlusion_model) {
    default:
    case OCCLUSION_MODEL::ZERO:
      fragment_shaders.push_back(get_data_path() + "/shader/brdf/occlusion_zero.frag");
      break;
    case OCCLUSION_MODEL::AMBIENT_OCCLUSION:
      fragment_shaders.push_back(get_data_path() + "/shader/brdf/occlusion_ambient.frag");
      break;
  }

  std::string iridescence_texture_path;
  switch (iridescence_model) {
    case IRIDESCENCE_MODEL::ZERO:
      break;
    case IRIDESCENCE_MODEL::IRIDESCENCE1:
      iridescence_texture_path = get_data_path() + "/texture/iridescence1.png";
      break;
    case IRIDESCENCE_MODEL::IRIDESCENCE2:
      iridescence_texture_path = get_data_path() + "/texture/iridescence2.png";
      break;
    case IRIDESCENCE_MODEL::IRIDESCENCE3:
      iridescence_texture_path = get_data_path() + "/texture/iridescence3.png";
      break;
  }

  if (iridescence_model == IRIDESCENCE_MODEL::ZERO) {
    fragment_shaders.push_back(get_data_path() + "/shader/brdf/iridescence_zero.frag");
  } else {
    int width, height;
    std::vector<unsigned char> bytes;

    if (!load_png(iridescence_texture_path, width, height, bytes)) {
      return false;
    }
    fragment_shaders.push_back(get_data_path() + "/shader/brdf/iridescence.frag");
    iridescence_texture.reset(new glk::Texture(Eigen::Vector2i(width, height), GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, bytes.data()));
  }

  return lighting_shader.init(vertex_shaders, fragment_shaders);
}

float ScreenSpaceLighting::get_albedo() const {
  return albedo;
}

float ScreenSpaceLighting::get_roughness() const {
  return roughness;
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

bool ScreenSpaceLighting::is_light_directional(int i) const {
  return light_directional[i];
}

float ScreenSpaceLighting::get_light_range(int i) const {
  return light_range[i];
}

const Eigen::Vector2f& ScreenSpaceLighting::get_light_attenuation(int i) const {
  return light_attenuation[i];
}

const Eigen::Vector3f& ScreenSpaceLighting::get_light_pos(int i) const {
  return light_pos[i];
}

const Eigen::Vector3f& ScreenSpaceLighting::get_light_dir(int i) const {
  return light_pos[i];
}

const Eigen::Vector4f& ScreenSpaceLighting::get_light_color(int i) const {
  return light_color[i];
}

void ScreenSpaceLighting::set_light_directional(int i, bool directional) {
  light_updated = true;
  light_directional[i] = directional;
}

void ScreenSpaceLighting::set_light_range(int i, float range) {
  light_updated = true;
  light_range[i] = range;
}

void ScreenSpaceLighting::set_light_attenuation(int i, const Eigen::Vector2f& attenuation) {
  light_updated = true;
  light_attenuation[i] = attenuation;
}

void ScreenSpaceLighting::set_light_pos(int i, const Eigen::Vector3f& pos) {
  light_updated = true;
  light_pos[i] = pos;
}

void ScreenSpaceLighting::set_light_dir(int i, const Eigen::Vector3f& dir) {
  light_updated = true;
  light_pos[i] = dir;
}

void ScreenSpaceLighting::set_light_color(int i, const Eigen::Vector4f& color) {
  light_updated = true;
  light_color[i] = color;
}

void ScreenSpaceLighting::set_light(int i, const Eigen::Vector3f& pos, const Eigen::Vector4f& color) {
  set_light(i, pos, color, Eigen::Vector2f::Zero(), 1e6f);
}

void ScreenSpaceLighting::set_light(int i, const Eigen::Vector3f& pos, const Eigen::Vector4f& color, const Eigen::Vector2f& attenuation, float max_range) {
  light_updated = true;

  while (i >= light_pos.size()) {
    light_directional.push_back(false);
    light_range.push_back(1000.0f);
    light_attenuation.push_back(Eigen::Vector2f(0.0f, 0.0f));
    light_pos.push_back(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    light_color.push_back(Eigen::Vector4f(0.0f, 0.0f, 0.0f, 0.0f));
  }

  light_directional[i] = false;
  light_range[i] = max_range;
  light_attenuation[i] = attenuation;
  light_pos[i] = pos;
  light_color[i] = color;
}

void ScreenSpaceLighting::set_directional_light(int i, const Eigen::Vector3f& direction, const Eigen::Vector4f& color) {
  light_updated = true;

  while (i >= light_pos.size()) {
    light_directional.push_back(false);
    light_range.push_back(1000.0f);
    light_attenuation.push_back(Eigen::Vector2f(0.0f, 0.0f));
    light_pos.push_back(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    light_color.push_back(Eigen::Vector4f(0.0f, 0.0f, 0.0f, 0.0f));
  }

  light_directional[i] = true;
  light_range[i] = 0.0f;
  light_pos[i] = direction.normalized();
  light_color[i] = color;
}

void ScreenSpaceLighting::set_size(const Eigen::Vector2i& size) {
  if (splatting) {
    splatting->set_size(size);
  }

  if (ssae) {
    ssae->set_size(size);
  }
}

void ScreenSpaceLighting::draw(
  const TextureRenderer& renderer,
  const glk::Texture& color_texture,
  const glk::Texture& depth_texture,
  const TextureRendererInput::Ptr& input,
  glk::FrameBuffer* frame_buffer) {
  using namespace glk::console;

  if (splatting) {
    splatting->draw(renderer, color_texture, depth_texture, input);
  }
  if (ssae) {
    ssae->draw(renderer, color_texture, depth_texture, input);
  }

  if (frame_buffer) {
    frame_buffer->bind();
  }

  auto view_matrix = input->get<Eigen::Matrix4f>("view_matrix");
  if (!view_matrix) {
    std::cerr << bold_red << "error: view and projection matrices must be set" << reset << std::endl;
    return;
  }
  Eigen::Vector3f view_point = view_matrix->inverse().block<3, 1>(0, 3);

  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);

  lighting_shader.use();
  lighting_shader.set_uniform("color_sampler", 0);
  lighting_shader.set_uniform("position_sampler", 1);
  lighting_shader.set_uniform("normal_sampler", 2);
  lighting_shader.set_uniform("occlusion_sampler", 3);
  lighting_shader.set_uniform("iridescence_sampler", 4);

  lighting_shader.set_uniform("view_point", view_point);

  lighting_shader.set_uniform("albedo", albedo);
  lighting_shader.set_uniform("roughness", roughness);

  if (light_updated) {
    lighting_shader.set_uniform("num_lights", static_cast<int>(light_pos.size()));
    lighting_shader.set_uniform("light_directional", light_directional);
    lighting_shader.set_uniform("light_range", light_range);
    lighting_shader.set_uniform("light_attenuation", light_attenuation);
    lighting_shader.set_uniform("light_pos", light_pos);
    lighting_shader.set_uniform("light_color", light_color);
  }

  if (splatting) {
    splatting->color().bind(GL_TEXTURE0);
    splatting->position().bind(GL_TEXTURE1);
    splatting->normal().bind(GL_TEXTURE2);
  } else {
    color_texture.bind(GL_TEXTURE0);
    ssae->position().bind(GL_TEXTURE1);
    ssae->normal().bind(GL_TEXTURE2);
    ssae->occlusion().bind(GL_TEXTURE3);
  }

  iridescence_texture->bind(GL_TEXTURE4);

  renderer.draw_plain(lighting_shader);

  glEnable(GL_DEPTH_TEST);

  /*
  guik::LightViewer::instance()->register_ui_callback("texture", [this] {
    ImGui::Begin("texture", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    if(splatting) {
      ImGui::Image((void*)splatting->normal().id(), ImVec2(512, 512), ImVec2(0, 1), ImVec2(1, 0));
    } else {
      ImGui::Image((void*)ssae->normal().id(), ImVec2(512, 512), ImVec2(0, 1), ImVec2(1, 0));
    }

    ImGui::End();
  });
  */

  if (frame_buffer) {
    frame_buffer->unbind();
  }
}

}  // namespace glk
