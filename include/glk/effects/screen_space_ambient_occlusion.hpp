#ifndef GLK_SCREEN_SPACE_AMBIENT_OCCLUSION_HPP
#define GLK_SCREEN_SPACE_AMBIENT_OCCLUSION_HPP

#include <random>
#include <glk/frame_buffer.hpp>
#include <glk/effects/screen_effect.hpp>

namespace glk {

class ScreenSpaceAmbientOcclusion : public ScreenEffect {
public:
  ScreenSpaceAmbientOcclusion(const std::string& data_directory) {
    if(!ssao_shader.init(data_directory + "/shader/texture.vert", data_directory + "/shader/ssao.frag")) {
      return;
    }
    std::mt19937 mt;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> random_vectors(512);
    for(auto& vec : random_vectors) {
      double u = std::uniform_real_distribution<>(0.0, 1.0)(mt);
      double v = std::uniform_real_distribution<>(-1.0, 1.0)(mt);
      double w = std::uniform_real_distribution<>(0.0, 1.0)(mt);

      double r = w * 1.0;

      vec.x() = r * std::sqrt(1 - v * v) * std::cos(2.0 * M_PI * u);
      vec.y() = r * std::sqrt(1 - v * v) * std::sin(2.0 * M_PI * u);
      vec.z() = std::abs(r * v);
    }

    ssao_shader.use();
    ssao_shader.set_uniform("random_vectors", random_vectors);
  }
  virtual ~ScreenSpaceAmbientOcclusion() override {}

  virtual PASS_TYPE pass() const override {
    return ONE;
  }

  virtual void use(const glk::Texture& color_texture, const glk::Texture& depth_texture, const glk::FrameBuffer* first_pass_result, PASS_TYPE pass) override {
    ssao_shader.use();

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, color_texture.id());
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, depth_texture.id());
    glActiveTexture(GL_TEXTURE2);
    glActiveTexture(GL_TEXTURE0);

    ssao_shader.set_uniform("color_sampler", 0);
    ssao_shader.set_uniform("depth_sampler", 1);
  }

  virtual glk::GLSLShader& shader(PASS_TYPE pass) override {
    return ssao_shader;
  }

private:
  glk::GLSLShader ssao_shader;
};

}  // namespace glk

#endif