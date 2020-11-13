#ifndef GLK_SCREEN_SPACE_ATTRIBUTE_ESTIMATION_HPP
#define GLK_SCREEN_SPACE_ATTRIBUTE_ESTIMATION_HPP

#include <random>
#include <iostream>
#include <glk/path.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/effects/screen_effect.hpp>

namespace glk {

class ScreenSpaceAttributeEstimation: public ScreenEffect {
public:
  ScreenSpaceAttributeEstimation(const Eigen::Vector2i& size) {
    if(!texture_shader.init(get_data_path() + "/shader/texture.vert", get_data_path() + "/shader/texture.frag")) {
      return;
    }

    if(!pos_shader.init(get_data_path() + "/shader/texture.vert", get_data_path() + "/shader/ssae_pos.frag")) {
      return;
    }

    if(!normal_shader.init(get_data_path() + "/shader/texture.vert", get_data_path() + "/shader/ssae_normal.frag")) {
      return;
    }

    if(!occlusion_shader.init(get_data_path() + "/shader/texture.vert", get_data_path() + "/shader/ssae_ao.frag")) {
      return;
    }

    if(!bilateral_shader.init(get_data_path() + "/shader/texture.vert", get_data_path() + "/shader/bilateral.frag")) {
      return;
    }

    set_size(size);
    smooth_normal = true;

    // set AO random vectors
    std::mt19937 mt;
    auto sample_random_vector = [&] {
      double u = std::uniform_real_distribution<>(0.0, 1.0)(mt);
      double v = std::uniform_real_distribution<>(-1.0, 1.0)(mt);
      double w = std::uniform_real_distribution<>(0.0, 1.0)(mt);

      double r = w * 1.0;

      Eigen::Vector3f vec;
      vec.x() = r * std::sqrt(1 - v * v) * std::cos(2.0 * M_PI * u);
      vec.y() = r * std::sqrt(1 - v * v) * std::sin(2.0 * M_PI * u);
      vec.z() = r * v;
      return vec;
    };

    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> random_vectors(16);
    for(auto& vec : random_vectors) {
      vec = sample_random_vector();
    }

    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> randomization(128 * 128);
    for(auto& vec : randomization) {
      vec = sample_random_vector();
    }
    randomization_texture.reset(new glk::Texture(Eigen::Vector2i(128, 128), GL_RGB32F, GL_RGB, GL_FLOAT, randomization.data()));
    randomization_texture->bind();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    randomization_texture->unbind();

    pos_shader.use();
    pos_shader.set_uniform("depth_sampler", 0);

    normal_shader.use();
    normal_shader.set_uniform("depth_sampler", 0);
    normal_shader.set_uniform("position_sampler", 1);

    occlusion_shader.use();
    occlusion_shader.set_uniform("depth_sampler", 0);
    occlusion_shader.set_uniform("position_sampler", 1);
    occlusion_shader.set_uniform("normal_sampler", 2);
    occlusion_shader.set_uniform("randomization_sampler", 3);
    occlusion_shader.set_uniform("ao_radius", 0.05f);
    occlusion_shader.set_uniform("random_vectors", random_vectors);

    bilateral_shader.use();
    bilateral_shader.set_uniform("color_sampler", 0);
    bilateral_shader.set_uniform("position_sampler", 1);
    bilateral_shader.set_uniform("half_kernel_size", 10);
    bilateral_shader.set_uniform("sigma_x", 10.0f);
    bilateral_shader.set_uniform("sigma_d", 0.01f);
  }
  virtual ~ScreenSpaceAttributeEstimation() override {}

  virtual void set_size(const Eigen::Vector2i& size) override {
    position_buffer.reset(new glk::FrameBuffer(size, 0, false));
    position_buffer->add_color_buffer(GL_RGBA32F, GL_RGB, GL_FLOAT);  // position

    position_smoothing_x_buffer.reset(new glk::FrameBuffer(size, 0, false));
    position_smoothing_x_buffer->add_color_buffer(GL_RGBA32F, GL_RGB, GL_FLOAT);

    position_smoothing_y_buffer.reset(new glk::FrameBuffer(size, 0, false));
    position_smoothing_y_buffer->add_color_buffer(GL_RGBA32F, GL_RGB, GL_FLOAT);

    normal_buffer.reset(new glk::FrameBuffer(size, 0, false));
    normal_buffer->add_color_buffer(GL_RGB32F, GL_RGB, GL_FLOAT);

    occlusion_buffer.reset(new glk::FrameBuffer(size, 0, false));
    occlusion_buffer->add_color_buffer(GL_R32F, GL_RGB, GL_FLOAT);
    occlusion_buffer->add_color_buffer(GL_RGB32F, GL_RGB, GL_FLOAT);

    bilateral_x_buffer.reset(new glk::FrameBuffer(size, 0, false));
    bilateral_x_buffer->add_color_buffer(GL_RGBA32F, GL_RGB, GL_FLOAT);

    bilateral_y_buffer.reset(new glk::FrameBuffer(size, 0, false));
    bilateral_y_buffer->add_color_buffer(GL_RGBA32F, GL_RGB, GL_FLOAT);
  }

  const glk::Texture& position() const {
    return position_buffer->color();
  }

  const glk::Texture& normal() const {
    return normal_buffer->color();
  }

  const glk::Texture& occlusion() const {
    return bilateral_y_buffer->color(0);
  }

  virtual void draw(const TextureRenderer& renderer, const glk::Texture& color_texture, const glk::Texture& depth_texture, const TextureRendererInput::Ptr& input) override {
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);

    // position
    auto view_matrix = input->get<Eigen::Matrix4f>("view_matrix");
    auto projection_matrix = input->get<Eigen::Matrix4f>("projection_matrix");
    if(!view_matrix || !projection_matrix) {
      std::cerr << "view and projection matrices must be set" << std::endl;
      return;
    }
    Eigen::Matrix4f projection_view_matrix = (*projection_matrix) * (*view_matrix);
    Eigen::Matrix4f inv_projection_view_matrix = projection_view_matrix.inverse();
    Eigen::Vector2f inv_frame_size = 1.0f / position_buffer->size().array().cast<float>();

    pos_shader.use();
    pos_shader.set_uniform("inv_projection_view_matrix", inv_projection_view_matrix);

    depth_texture.bind();
    position_buffer->bind();
    renderer.draw_plain(pos_shader);
    position_buffer->unbind();

    if(smooth_normal) {
      // apply bilateral filter to position buffer
      bilateral_shader.use();
      bilateral_shader.set_uniform("inv_frame_size", inv_frame_size);
      bilateral_shader.set_uniform("filter_direction", Eigen::Vector2f(1.0f, 0.0f));
      bilateral_shader.set_uniform("second_pass", false);

      // filter in x-axis
      position_smoothing_x_buffer->bind();
      position_buffer->color(0).bind(GL_TEXTURE0);
      position_buffer->color(0).bind(GL_TEXTURE1);
      renderer.draw_plain(bilateral_shader);
      position_smoothing_x_buffer->unbind();

      bilateral_shader.set_uniform("filter_direction", Eigen::Vector2f(0.0f, 1.0f));
      bilateral_shader.set_uniform("second_pass", true);

      // filter in y-axis
      position_smoothing_y_buffer->bind();
      position_smoothing_x_buffer->color().bind(GL_TEXTURE0);
      position_buffer->color().bind(GL_TEXTURE1);
      renderer.draw_plain(bilateral_shader);
      position_smoothing_y_buffer->unbind();
    }

    // normal
    Eigen::Vector3f view_point = view_matrix->inverse().block<3, 1>(0, 3);

    normal_shader.use();
    normal_shader.set_uniform("view_point", view_point);
    normal_shader.set_uniform("inv_frame_size", inv_frame_size);

    normal_buffer->bind();
    depth_texture.bind(GL_TEXTURE0);
    if(smooth_normal) {
      position_smoothing_y_buffer->color().bind(GL_TEXTURE1);
    } else {
      position_buffer->color().bind(GL_TEXTURE1);
    }
    renderer.draw_plain(normal_shader);
    normal_buffer->unbind();

    // ambient occlusion
    Eigen::Vector2f randomization_coord_scale = color_texture.size().cast<float>().array() / randomization_texture->size().cast<float>().array();

    occlusion_shader.use();
    occlusion_shader.set_uniform("depth_sampler", 0);
    occlusion_shader.set_uniform("position_sampler", 1);
    occlusion_shader.set_uniform("normal_sampler", 2);
    occlusion_shader.set_uniform("randomization_sampler", 3);

    occlusion_shader.set_uniform("projection_view_matrix", projection_view_matrix);
    occlusion_shader.set_uniform("randomization_coord_scale", randomization_coord_scale);

    occlusion_buffer->bind();
    depth_texture.bind(GL_TEXTURE0);
    position_buffer->color().bind(GL_TEXTURE1);
    normal_buffer->color().bind(GL_TEXTURE2);
    randomization_texture->bind(GL_TEXTURE3);
    renderer.draw_plain(occlusion_shader);
    occlusion_buffer->unbind();

    // bilateral filter
    bilateral_shader.use();
    bilateral_shader.set_uniform("inv_frame_size", inv_frame_size);
    bilateral_shader.set_uniform("filter_direction", Eigen::Vector2f(1.0f, 0.0f));
    bilateral_shader.set_uniform("second_pass", false);

    // filter in x-axis
    bilateral_x_buffer->bind();
    occlusion_buffer->color(0).bind(GL_TEXTURE0);
    position_buffer->color(0).bind(GL_TEXTURE1);
    renderer.draw_plain(bilateral_shader);
    bilateral_x_buffer->unbind();

    bilateral_shader.set_uniform("filter_direction", Eigen::Vector2f(0.0f, 1.0f));
    bilateral_shader.set_uniform("second_pass", true);

    // filter in y-axis
    bilateral_y_buffer->bind();
    bilateral_x_buffer->color().bind(GL_TEXTURE0);
    position_buffer->color().bind(GL_TEXTURE1);
    renderer.draw_plain(bilateral_shader);
    bilateral_y_buffer->unbind();

    glDisable(GL_TEXTURE_2D);
    glEnable(GL_DEPTH_TEST);
  }

private:
  bool smooth_normal;

  glk::GLSLShader texture_shader;
  glk::GLSLShader pos_shader;
  glk::GLSLShader normal_shader;
  glk::GLSLShader occlusion_shader;
  glk::GLSLShader bilateral_shader;

  std::unique_ptr<glk::Texture> randomization_texture;

  std::unique_ptr<glk::FrameBuffer> position_buffer;
  std::unique_ptr<glk::FrameBuffer> position_smoothing_x_buffer;
  std::unique_ptr<glk::FrameBuffer> position_smoothing_y_buffer;

  std::unique_ptr<glk::FrameBuffer> normal_buffer;
  std::unique_ptr<glk::FrameBuffer> occlusion_buffer;
  std::unique_ptr<glk::FrameBuffer> bilateral_x_buffer;
  std::unique_ptr<glk::FrameBuffer> bilateral_y_buffer;
};

}  // namespace glk

#endif