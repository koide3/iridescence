#ifndef GLK_SCREEN_SPACE_AMBIENT_OCCLUSION_HPP
#define GLK_SCREEN_SPACE_AMBIENT_OCCLUSION_HPP

#include <random>
#include <iostream>
#include <glk/path.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/effects/screen_effect.hpp>

namespace glk {

class ScreenSpaceAmbientOcclusion : public ScreenEffect {
public:
  ScreenSpaceAmbientOcclusion() {
    if(!ssao_shader.init(get_data_path() + "/shader/texture.vert", get_data_path() + "/shader/ssao.frag")) {
      return;
    }
    std::mt19937 mt;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> random_vectors(32);
    for(auto& vec : random_vectors) {
      double u = std::uniform_real_distribution<>(0.0, 1.0)(mt);
      double v = std::uniform_real_distribution<>(-1.0, 1.0)(mt);
      double w = std::uniform_real_distribution<>(0.0, 1.0)(mt);

      double r = w * 1.0;

      vec.x() = r * std::sqrt(1 - v * v) * std::cos(2.0 * M_PI * u);
      vec.y() = r * std::sqrt(1 - v * v) * std::sin(2.0 * M_PI * u);
      vec.z() = r * v;
    }

    ssao_shader.use();
    ssao_shader.set_uniform("ao_radius", 0.1f);
    ssao_shader.set_uniform("random_vectors", random_vectors);
  }
  virtual ~ScreenSpaceAmbientOcclusion() override {}

  virtual void draw(const TextureRenderer& renderer, const glk::Texture& color_texture, const glk::Texture& depth_texture, const TextureRendererInput::Ptr& input) override {
    ssao_shader.use();
    ssao_shader.set_uniform("color_sampler", 0);
    ssao_shader.set_uniform("depth_sampler", 1);

    auto view_matrix = input->get<Eigen::Matrix4f>("view_matrix");
    auto projection_matrix = input->get<Eigen::Matrix4f>("projection_matrix");
    if(!view_matrix || !projection_matrix) {
      std::cerr << "view and projection matrices must be set" << std::endl;
      return;
    }
    Eigen::Matrix4f projection_view_matrix = (*projection_matrix) * (*view_matrix);
    ssao_shader.set_uniform("projection_view_matrix", projection_view_matrix);
    ssao_shader.set_uniform("inv_projection_view_matrix", projection_view_matrix.inverse().eval());

    glEnable(GL_TEXTURE_2D);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, color_texture.id());
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, depth_texture.id());
    glActiveTexture(GL_TEXTURE0);

    renderer.draw_plain(ssao_shader);

    glDisable(GL_TEXTURE_2D);
  }

private:
  glk::GLSLShader ssao_shader;
};

}  // namespace glk

#endif