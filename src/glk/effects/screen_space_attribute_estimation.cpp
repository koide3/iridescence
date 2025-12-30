#include <random>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glk/path.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/console_colors.hpp>
#include <glk/effects/screen_scape_attribute_estimation.hpp>

namespace glk {

ScreenSpaceAttributeEstimation::ScreenSpaceAttributeEstimation(const Eigen::Vector2i& size, BufferType rendering_type) {
  if (!texture_shader.init(get_data_path() + "/shader/texture.vert", get_data_path() + "/shader/texture.frag")) {
    return;
  }

  if (!pos_shader.init(get_data_path() + "/shader/texture.vert", get_data_path() + "/shader/ssae_pos.frag")) {
    return;
  }

  if (!normal_shader.init(get_data_path() + "/shader/texture.vert", get_data_path() + "/shader/ssae_normal.frag")) {
    return;
  }

  if (!occlusion_shader.init(get_data_path() + "/shader/texture.vert", get_data_path() + "/shader/ssae_ao.frag")) {
    return;
  }

  if (!bilateral_shader.init(get_data_path() + "/shader/texture.vert", get_data_path() + "/shader/bilateral.frag")) {
    return;
  }

  set_size(size);
  smooth_normal = true;
  this->rendering_type = rendering_type;

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

  std::vector<Eigen::Vector3f> random_vectors(16);
  for (auto& vec : random_vectors) {
    vec = sample_random_vector();
  }

  std::vector<Eigen::Vector3f> randomization(128 * 128);
  for (auto& vec : randomization) {
    vec = sample_random_vector().normalized();
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
  normal_shader.set_uniform("normal_sampler", 2);

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
ScreenSpaceAttributeEstimation::~ScreenSpaceAttributeEstimation() {}

void ScreenSpaceAttributeEstimation::set_size(const Eigen::Vector2i& size) {
  position_buffer.reset(new glk::FrameBuffer(size, 0, false));
  position_buffer->add_color_buffer(0, GL_RGBA32F, GL_RGB, GL_FLOAT);  // position

  position_smoothing_x_buffer.reset(new glk::FrameBuffer(size, 0, false));
  position_smoothing_x_buffer->add_color_buffer(0, GL_RGBA32F, GL_RGB, GL_FLOAT);

  position_smoothing_y_buffer.reset(new glk::FrameBuffer(size, 0, false));
  position_smoothing_y_buffer->add_color_buffer(0, GL_RGBA32F, GL_RGB, GL_FLOAT);

  normal_buffer.reset(new glk::FrameBuffer(size, 0, false));
  normal_buffer->add_color_buffer(0, GL_RGB32F, GL_RGB, GL_FLOAT);

  occlusion_buffer.reset(new glk::FrameBuffer(size, 0, false));
  occlusion_buffer->add_color_buffer(0, GL_R32F, GL_RGB, GL_FLOAT);
  occlusion_buffer->add_color_buffer(1, GL_RGB32F, GL_RGB, GL_FLOAT);

  bilateral_x_buffer.reset(new glk::FrameBuffer(size, 0, false));
  bilateral_x_buffer->add_color_buffer(0, GL_RGBA32F, GL_RGB, GL_FLOAT);

  bilateral_y_buffer.reset(new glk::FrameBuffer(size, 0, false));
  bilateral_y_buffer->add_color_buffer(0, GL_RGBA32F, GL_RGB, GL_FLOAT);
}

void ScreenSpaceAttributeEstimation::set_smooth_normal(bool smooth_normal) {
  this->smooth_normal = smooth_normal;
}

void ScreenSpaceAttributeEstimation::set_rendering_buffer(BufferType buffer_type) {
  this->rendering_type = buffer_type;
}

const glk::Texture& ScreenSpaceAttributeEstimation::position() const {
  return position_buffer->color();
}

const glk::Texture& ScreenSpaceAttributeEstimation::normal() const {
  return normal_buffer->color();
}

const glk::Texture& ScreenSpaceAttributeEstimation::occlusion() const {
  return bilateral_y_buffer->color(0);
}

void ScreenSpaceAttributeEstimation::draw(
  const TextureRenderer& renderer,
  const glk::Texture& color_texture,
  const glk::Texture& depth_texture,
  const TextureRendererInput::Ptr& input,
  glk::FrameBuffer* frame_buffer) {
  using namespace glk::console;

  glDisable(GL_DEPTH_TEST);

  // position
  auto view_matrix = input->get<Eigen::Matrix4f>("view_matrix");
  auto projection_matrix = input->get<Eigen::Matrix4f>("projection_matrix");
  if (!view_matrix || !projection_matrix) {
    std::cerr << bold_red << "error: view and projection matrices must be set" << reset << std::endl;
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

  if (smooth_normal) {
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
  if (smooth_normal) {
    position_smoothing_y_buffer->color().bind(GL_TEXTURE1);
  } else {
    position_buffer->color().bind(GL_TEXTURE1);
  }

  glActiveTexture(GL_TEXTURE2);
  auto normal_texture = input->get<GLuint>("normal_texture");
  if (normal_texture) {
    glBindTexture(GL_TEXTURE_2D, *normal_texture);
  } else {
    glBindTexture(GL_TEXTURE_2D, 0);
  }
  glActiveTexture(GL_TEXTURE0);

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

  if (rendering_type != BufferType::NONE) {
    if (frame_buffer) {
      frame_buffer->bind();
    }

    const glk::Texture* textures[] = {
      &depth_texture,
      &position_buffer->color(),
      &position_smoothing_x_buffer->color(),
      &position_smoothing_y_buffer->color(),
      &normal_buffer->color(),
      &occlusion_buffer->color(),
      &bilateral_x_buffer->color(),
      &bilateral_y_buffer->color()};
    const auto texture = textures[static_cast<int>(rendering_type) - 1];
    texture->bind();

    texture_shader.use();
    renderer.draw_plain(texture_shader);

    if (frame_buffer) {
      frame_buffer->unbind();
    }
  }

  glEnable(GL_DEPTH_TEST);
}

}  // namespace glk
