#include <random>
#include <iostream>
#include <glk/path.hpp>
#include <glk/console_colors.hpp>
#include <glk/effects/screen_effect.hpp>
#include <glk/effects/naive_screen_space_ambient_occlusion.hpp>

namespace glk {

NaiveScreenSpaceAmbientOcclusion::NaiveScreenSpaceAmbientOcclusion() {
  if (!ssao_shader.init(get_data_path() + "/shader/texture.vert", get_data_path() + "/shader/naive_ssao.frag")) {
    return;
  }
  std::mt19937 mt;
  std::vector<Eigen::Vector3f> random_vectors(32);
  for (auto& vec : random_vectors) {
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

NaiveScreenSpaceAmbientOcclusion::~NaiveScreenSpaceAmbientOcclusion() {}

void NaiveScreenSpaceAmbientOcclusion::draw(
  const TextureRenderer& renderer,
  const glk::Texture& color_texture,
  const glk::Texture& depth_texture,
  const TextureRendererInput::Ptr& input,
  glk::FrameBuffer* frame_buffer) {
  if (frame_buffer) {
    frame_buffer->bind();
  }

  ssao_shader.use();
  ssao_shader.set_uniform("color_sampler", 0);
  ssao_shader.set_uniform("depth_sampler", 1);

  auto view_matrix = input->get<Eigen::Matrix4f>("view_matrix");
  auto projection_matrix = input->get<Eigen::Matrix4f>("projection_matrix");
  if (!view_matrix || !projection_matrix) {
    using namespace glk::console;
    std::cerr << bold_red << "error: view and projection matrices must be set" << reset << std::endl;
    return;
  }
  Eigen::Matrix4f projection_view_matrix = (*projection_matrix) * (*view_matrix);
  ssao_shader.set_uniform("projection_view_matrix", projection_view_matrix);
  ssao_shader.set_uniform("inv_projection_view_matrix", projection_view_matrix.inverse().eval());

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, color_texture.id());
  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D, depth_texture.id());
  glActiveTexture(GL_TEXTURE0);

  renderer.draw_plain(ssao_shader);

  if (frame_buffer) {
    frame_buffer->unbind();
  }
}

}  // namespace glk
