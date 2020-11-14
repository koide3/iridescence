#ifndef GLK_SCREEN_SPACE_LIGHTING_HPP
#define GLK_SCREEN_SPACE_LIGHTING_HPP

#include <random>
#include <iostream>
#include <glk/path.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/effects/screen_effect.hpp>

#include <glk/effects/screen_scape_attribute_estimation.hpp>

namespace glk {

class ScreenSpaceLighting : public ScreenEffect {
public:
  ScreenSpaceLighting(const Eigen::Vector2i& size) {
    ssae.reset(new ScreenSpaceAttributeEstimation(size));

    if(!lighting_shader.init(get_data_path() + "/shader/texture.vert", get_data_path() + "/shader/ssli.frag")) {
      return;
    }

    add_light(Eigen::Vector3f(0.0f, 0.0f, 200.0f), Eigen::Vector4f(0.9f, 0.9f, 1.0f, 1.0f));
  }
  virtual ~ScreenSpaceLighting() override {}

  int num_lights() const {
    return light_pos.size();
  }

  void set_light(int i, const Eigen::Vector3f& pos, const Eigen::Vector4f& color) {
    light_updated = true;
    light_pos[i] = pos;
    light_color[i] = color;
  }

  void add_light(const Eigen::Vector3f& pos, const Eigen::Vector4f& color) {
    light_updated = true;
    light_pos.push_back(pos);
    light_color.push_back(color);
  }

  virtual void set_size(const Eigen::Vector2i& size) override {
    ssae->set_size(size);
  }

  virtual void draw(const TextureRenderer& renderer, const glk::Texture& color_texture, const glk::Texture& depth_texture, const TextureRendererInput::Ptr& input) override {
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
    lighting_shader.set_uniform("material_shininess", 10.0f);
    lighting_shader.set_uniform("ambient_light_color", Eigen::Vector4f(0.1f, 0.1f, 0.1f, 0.0f));

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

private:
  std::unique_ptr<glk::ScreenSpaceAttributeEstimation> ssae;

  glk::GLSLShader lighting_shader;

  bool light_updated;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> light_pos;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> light_color;
};

}  // namespace glk

#endif