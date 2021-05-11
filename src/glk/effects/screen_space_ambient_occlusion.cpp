#include <random>
#include <iostream>
#include <glk/path.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/effects/screen_effect.hpp>

#include <glk/effects/screen_scape_attribute_estimation.hpp>
#include <glk/effects/screen_space_ambient_occlusion.hpp>

namespace glk {

ScreenSpaceAmbientOcclusion::ScreenSpaceAmbientOcclusion(const Eigen::Vector2i& size) {
  ssae.reset(new ScreenSpaceAttributeEstimation(size));

  if(!ssao_shader.init(get_data_path() + "/shader/texture.vert", get_data_path() + "/shader/ssao.frag")) {
    return;
  }
}
ScreenSpaceAmbientOcclusion::~ScreenSpaceAmbientOcclusion() {}

void ScreenSpaceAmbientOcclusion::set_size(const Eigen::Vector2i& size) {
  ssae->set_size(size);
}

void ScreenSpaceAmbientOcclusion::draw(const TextureRenderer& renderer, const glk::Texture& color_texture, const glk::Texture& depth_texture, const TextureRendererInput::Ptr& input, glk::FrameBuffer* frame_buffer) {
  ssae->draw(renderer, color_texture, depth_texture, input, frame_buffer);

  if(frame_buffer) {
    frame_buffer->bind();
  }

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

  glEnable(GL_DEPTH_TEST);

  if(frame_buffer) {
    frame_buffer->unbind();
  }
}

}  // namespace glk
