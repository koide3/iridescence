#ifndef GLK_SCREEN_SPACE_AMBIENT_OCCLUSION_HPP
#define GLK_SCREEN_SPACE_AMBIENT_OCCLUSION_HPP

#include <random>
#include <iostream>
#include <glk/path.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/effects/screen_effect.hpp>

#include <glk/effects/screen_scape_attribute_estimation.hpp>

namespace glk {

class ScreenSpaceAmbientOcclusion : public ScreenEffect {
public:
  ScreenSpaceAmbientOcclusion(const Eigen::Vector2i& size) {
    ssae.reset(new ScreenSpaceAttributeEstimation(size));

    if(!ssao_shader.init(get_data_path() + "/shader/texture.vert", get_data_path() + "/shader/ssao.frag")) {
      return;
    }
  }
  virtual ~ScreenSpaceAmbientOcclusion() override {}

  virtual void set_size(const Eigen::Vector2i& size) override {
    ssae->set_size(size);
  }

  virtual void draw(const TextureRenderer& renderer, const glk::Texture& color_texture, const glk::Texture& depth_texture, const TextureRendererInput::Ptr& input) override {
    ssae->draw(renderer, color_texture, depth_texture, input);

    glEnable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);

    ssao_shader.use();
    ssao_shader.set_uniform("color_sampler", 0);
    ssao_shader.set_uniform("position_sampler", 1);
    ssao_shader.set_uniform("normal_sampler", 2);
    ssao_shader.set_uniform("occlusion_sampler", 3);

    color_texture.bind(GL_TEXTURE0);
    ssae->position().bind(GL_TEXTURE1);
    ssae->normal().bind(GL_TEXTURE2);
    ssae->occlusion().bind(GL_TEXTURE3);

    renderer.draw_plain(ssao_shader);

    glDisable(GL_TEXTURE_2D);
    glEnable(GL_DEPTH_TEST);
  }

private:
  std::unique_ptr<glk::ScreenSpaceAttributeEstimation> ssae;

  glk::GLSLShader ssao_shader;
};

}  // namespace glk

#endif