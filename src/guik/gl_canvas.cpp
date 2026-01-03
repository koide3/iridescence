#include <guik/gl_canvas.hpp>

#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <glk/path.hpp>
#include <glk/colormap.hpp>
#include <glk/glsl_shader.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/texture_renderer.hpp>
#include <glk/effects/plain_rendering.hpp>
#include <glk/console_colors.hpp>

#include <guik/viewer/light_viewer.hpp>
#include <guik/camera/camera_control.hpp>
#include <guik/camera/basic_projection_control.hpp>
#include <guik/camera/orbit_camera_control_xy.hpp>

namespace guik {

using namespace glk::console;

/**
 * @brief Construct a new GLCanvas object
 *
 * @param size
 */
GLCanvas::GLCanvas(const Eigen::Vector2i& size, const std::string& shader_name, size_t num_color_buffers) : size(size), clear_color(0.27f, 0.27f, 0.27f, 1.0f) {
  frame_buffer.reset(new glk::FrameBuffer(size, num_color_buffers));
  shader.reset(new glk::GLSLShader());
  if (!shader->init(glk::get_data_path() + "/shader/" + shader_name)) {
    shader.reset();
    return;
  }

  shader->use();

  shader->set_uniform("point_scale_mode", 1);
  shader->set_uniform("point_shape_mode", 1);
  shader->set_uniform("point_size", 0.025f);
  shader->set_uniform("point_scale", 1.0f);
  shader->set_uniform("point_size_offset", 0.0f);

  shader->set_uniform("viewport_size", size.cast<float>().eval());
  shader->set_uniform("model_matrix", Eigen::Matrix4f::Identity().eval());

  shader->set_uniform("color_mode", 0);
  shader->set_uniform("material_color", Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f));
  shader->set_uniform("z_range", Eigen::Vector2f(-3.0f, 5.0f));
  shader->set_uniform("cmap_range", Eigen::Vector2f(0.0f, 1.0f));
  shader->set_uniform("colormap_axis", Eigen::Vector3f(0.0f, 0.0f, 1.0f));

  shader->set_uniform("colormap_sampler", 0);
  shader->set_uniform("texture_sampler", 1);

  texture_shader.reset(new glk::GLSLShader());
  if (!texture_shader->init(glk::get_data_path() + "/shader/texture")) {
    texture_shader.reset();
    return;
  }

  auto colormap_table = glk::colormap_table(glk::COLORMAP::TURBO);
  colormap.reset(new glk::Texture(Eigen::Vector2i(256, 1), GL_RGBA, GL_RGB, GL_UNSIGNED_BYTE, colormap_table.data()));

  camera_control.reset(new guik::OrbitCameraControlXY());
  projection_control.reset(new guik::BasicProjectionControl(size));

  texture_renderer.reset(new glk::TextureRenderer());

  normal_buffer_id = info_buffer_id = dynamic_flag_buffer_id = 0;
  last_projection_view_matrix.setIdentity();

  is_partial_rendering_enabled = false;
  clear_partial_rendering_flag = false;
  partial_rendering_clear_thresh = 1e-6;

  alpha_blend_sfactor = GL_SRC_ALPHA;
  alpha_blend_dfactor = GL_ONE_MINUS_SRC_ALPHA;
  blend_depth_write = true;

  keyboard_control_speed = 5.0;
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool GLCanvas::ready() const {
  return frame_buffer && shader && camera_control && texture_renderer;
}

/**
 * @brief
 */
bool GLCanvas::load_shader(const std::string& shader_name) {
  shader.reset(new glk::GLSLShader());
  if (!shader->init(glk::get_data_path() + "/shader/" + shader_name)) {
    shader.reset();
    return false;
  }

  shader->use();

  shader->set_uniform("point_scale_mode", 1);
  shader->set_uniform("point_shape_mode", 1);
  shader->set_uniform("point_size", 0.025f);
  shader->set_uniform("point_scale", 1.0f);
  shader->set_uniform("point_size_offset", 0.0f);

  shader->set_uniform("viewport_size", size.cast<float>().eval());
  shader->set_uniform("model_matrix", Eigen::Matrix4f::Identity().eval());

  shader->set_uniform("color_mode", 0);
  shader->set_uniform("material_color", Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f));
  shader->set_uniform("z_range", Eigen::Vector2f(-3.0f, 5.0f));
  shader->set_uniform("cmap_range", Eigen::Vector2f(0.0f, 1.0f));
  shader->set_uniform("colormap_axis", Eigen::Vector3f(0.0f, 0.0f, 1.0f));

  shader->set_uniform("colormap_sampler", 0);
  shader->set_uniform("texture_sampler", 1);

  return true;
}

void GLCanvas::set_colormap(glk::COLORMAP colormap_type) {
  auto colormap_table = glk::colormap_table(colormap_type);
  colormap.reset(new glk::Texture(Eigen::Vector2i(256, 1), GL_RGBA, GL_RGB, GL_UNSIGNED_BYTE, colormap_table.data()));
}

void GLCanvas::set_effect(const std::shared_ptr<glk::ScreenEffect>& effect) {
  screen_effect = effect;
  screen_effect->set_size(size);
  if (!screen_effect_buffer) {
    screen_effect_buffer.reset(new glk::FrameBuffer(size, 1, false));
  }
}

const std::shared_ptr<glk::ScreenEffect>& GLCanvas::get_effect() const {
  return screen_effect;
}

void GLCanvas::set_bg_texture(const std::shared_ptr<glk::Texture>& bg_texture) {
  this->bg_texture = bg_texture;
}

void GLCanvas::set_blend_func(GLenum sfactor, GLenum dfactor) {
  alpha_blend_sfactor = sfactor;
  alpha_blend_dfactor = dfactor;
}

void GLCanvas::set_blend_depth_write(bool blend_depth_write) {
  this->blend_depth_write = blend_depth_write;
}

void GLCanvas::enable_normal_buffer() {
  normal_buffer_id = frame_buffer->num_color_buffers();
  frame_buffer->add_color_buffer(2, GL_RGB32F, GL_RGB, GL_FLOAT);
}

void GLCanvas::enable_info_buffer() {
  info_buffer_id = frame_buffer->num_color_buffers();
  frame_buffer->add_color_buffer(1, GL_RGBA32I, GL_RGBA_INTEGER, GL_INT);
}

void GLCanvas::enable_partial_rendering(double clear_thresh) {
  is_partial_rendering_enabled = true;
  partial_rendering_clear_thresh = clear_thresh;

  if (dynamic_flag_buffer_id > 0) {
    return;
  }

  dynamic_flag_buffer_id = frame_buffer->num_color_buffers();
  frame_buffer->add_color_buffer(3, GL_R8UI, GL_RED_INTEGER, GL_UNSIGNED_BYTE);

  const std::string data_path = glk::get_data_path();
  partial_clear_shader.reset(new glk::GLSLShader());
  if (!partial_clear_shader->init(data_path + "/shader/texture.vert", data_path + "/shader/partial_clear.frag")) {
    partial_clear_shader.reset();
  }
}

void GLCanvas::disable_partial_rendering() {
  is_partial_rendering_enabled = false;
}

void GLCanvas::clear_partial_rendering() {
  clear_partial_rendering_flag = true;
}

bool GLCanvas::normal_buffer_enabled() const {
  return normal_buffer_id > 0;
}

bool GLCanvas::info_buffer_enabled() const {
  return info_buffer_id > 0;
}

bool GLCanvas::partial_rendering_enabled() const {
  return is_partial_rendering_enabled;
}

const glk::Texture& GLCanvas::color_buffer() const {
  return frame_buffer->color();
}

const glk::Texture& GLCanvas::depth_buffer() const {
  return frame_buffer->depth();
}

const glk::Texture& GLCanvas::normal_buffer() const {
  return frame_buffer->color(normal_buffer_id);
}

const glk::Texture& GLCanvas::info_buffer() const {
  return frame_buffer->color(info_buffer_id);
}

const glk::Texture& GLCanvas::dynamic_flag_buffer() const {
  return frame_buffer->color(dynamic_flag_buffer_id);
}

/**
 * @brief Set the Size object
 *
 * @param size
 */
void GLCanvas::set_size(const Eigen::Vector2i& size) {
  this->size = size;

  projection_control->set_size(size);
  frame_buffer->set_size(size);

  if (screen_effect) {
    screen_effect->set_size(size);
  }

  if (screen_effect_buffer) {
    screen_effect_buffer->set_size(size);
  }
}

/**
 * @brief Set the background color
 *
 * @param color
 */
void GLCanvas::set_clear_color(const Eigen::Vector4f& color) {
  clear_color = color;
}

/**
 * @brief
 *
 */
void GLCanvas::bind() {
  frame_buffer->bind();
  glDisable(GL_SCISSOR_TEST);

  Eigen::Matrix4f view_matrix = camera_control->view_matrix();
  Eigen::Matrix4f projection_matrix = projection_control->projection_matrix();

  bool clear_buffer = true;
  if (is_partial_rendering_enabled) {
    Eigen::Matrix4f projection_view_matrix = projection_matrix * view_matrix;
    clear_buffer = (last_projection_view_matrix - projection_view_matrix).norm() > partial_rendering_clear_thresh || clear_partial_rendering_flag;
    clear_partial_rendering_flag = false;

    last_projection_view_matrix = projection_view_matrix;
  }

  shader->use();
  shader->set_uniform("point_scale_mode", 1);
  shader->set_uniform("point_shape_mode", 1);
  shader->set_uniform("point_size", 0.025f);
  shader->set_uniform("point_scale", 1.0f);
  shader->set_uniform("point_size_offset", 0.0f);

  shader->set_uniform("viewport_size", size.cast<float>().eval());
  shader->set_uniform("view_matrix", view_matrix);
  shader->set_uniform("inv_view_matrix", view_matrix.inverse().eval());
  shader->set_uniform("projection_matrix", projection_matrix);
  shader->set_uniform("info_enabled", info_buffer_id > 0);
  shader->set_uniform("normal_enabled", normal_buffer_id > 0);
  shader->set_uniform("partial_rendering_enabled", is_partial_rendering_enabled);

  shader->set_uniform("colormap_sampler", 0);
  shader->set_uniform("texture_sampler", 1);

  if (clear_buffer) {
    glClearColor(clear_color[0], clear_color[1], clear_color[2], clear_color[3]);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (bg_texture) {
      glDepthMask(GL_FALSE);

      texture_shader->use();

      bg_texture->bind();
      texture_renderer->draw_plain(*texture_shader);
      bg_texture->unbind();

      shader->use();

      glDepthMask(GL_TRUE);
    }

    if (normal_buffer_id) {
      GLfloat clear_color[] = {0.0f, 0.0f, 0.0f};
      glClearTexImage(frame_buffer->color(normal_buffer_id).id(), 0, GL_RGB, GL_FLOAT, clear_color);
    }

    if (info_buffer_id) {
      GLint clear_color[] = {-1, -1, -1, -1};
      glClearTexImage(frame_buffer->color(info_buffer_id).id(), 0, GL_RGBA_INTEGER, GL_INT, clear_color);
      shader->set_uniform("info_values", Eigen::Vector4i(-1, -1, -1, -1));
    }

    if (is_partial_rendering_enabled) {
      GLint clear_color[] = {255, 255, 255, 255};
      glClearTexImage(frame_buffer->color(dynamic_flag_buffer_id).id(), 0, GL_RED_INTEGER, GL_UNSIGNED_BYTE, clear_color);
      shader->set_uniform("dynamic_object", 255);
    }
  } else {
    // partial clear
    partial_clear_shader->use();
    partial_clear_shader->set_uniform("dynamic_flag_sampler", 0);
    partial_clear_shader->set_uniform("clear_color", clear_color);
    partial_clear_shader->set_uniform("info_enabled", info_buffer_id > 0);
    partial_clear_shader->set_uniform("normal_enabled", normal_buffer_id > 0);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_ALWAYS);
    glBindTexture(GL_TEXTURE_2D, dynamic_flag_buffer().id());

    texture_renderer->draw_plain(*partial_clear_shader);

    shader->use();
    glDepthFunc(GL_LESS);
  }

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

  glBindTexture(GL_TEXTURE_2D, colormap->id());
}

/**
 * @brief
 *
 */
void GLCanvas::unbind() {
  glBindTexture(GL_TEXTURE_2D, 0);

  glDisable(GL_DEPTH_TEST);
  glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);

  glFlush();

  frame_buffer->unbind();

  if (screen_effect) {
    glk::TextureRendererInput::Ptr input(new glk::TextureRendererInput());
    input->set("view_matrix", camera_control->view_matrix());
    input->set("projection_matrix", projection_control->projection_matrix());

    if (normal_buffer_id) {
      input->set("normal_texture", normal_buffer().id());
    }

    screen_effect->draw(*texture_renderer.get(), frame_buffer->color(), frame_buffer->depth(), input, screen_effect_buffer.get());

    frame_buffer->bind();
    glDisable(GL_SCISSOR_TEST);
    glDepthMask(GL_FALSE);

    texture_renderer->draw(screen_effect_buffer->color());

    glDepthMask(GL_TRUE);
    glEnable(GL_SCISSOR_TEST);
    frame_buffer->unbind();
  }
}

/**
 * @brief
 *
 */
void GLCanvas::bind_second() {
  frame_buffer->bind();
  glDisable(GL_SCISSOR_TEST);

  shader->use();
  if (info_buffer_id) {
    shader->set_uniform("info_values", Eigen::Vector4i(-1, -1, -1, -1));
  }

  if (is_partial_rendering_enabled) {
    shader->set_uniform("dynamic_object", 255);
  }

  glEnable(GL_BLEND);
  if (!blend_depth_write) {
    glDepthMask(GL_FALSE);
  }
  glBlendFunc(alpha_blend_sfactor, alpha_blend_dfactor);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

  glBindTexture(GL_TEXTURE_2D, colormap->id());
}

/**
 * @brief
 *
 */
void GLCanvas::unbind_second() {
  glBindTexture(GL_TEXTURE_2D, 0);

  if (!blend_depth_write) {
    glDepthMask(GL_TRUE);
  }
  glDisable(GL_BLEND);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);

  glFlush();

  frame_buffer->unbind();
}

/**
 * @brief
 *
 */
void GLCanvas::render_to_screen(int color_buffer_id) {
  glDisable(GL_SCISSOR_TEST);
  glDisable(GL_DEPTH_TEST);
  texture_renderer->draw(frame_buffer->color(color_buffer_id));
  glEnable(GL_SCISSOR_TEST);
  glEnable(GL_DEPTH_TEST);
}

/**
 * @brief
 *
 */
void GLCanvas::mouse_control() {
  const ImGuiIO& io = ImGui::GetIO();
  const double dt = 60.0 * io.DeltaTime;
  auto mouse_pos = ImGui::GetMousePos();
  auto drag_delta = ImGui::GetMouseDragDelta();

  Eigen::Vector2f p(mouse_pos.x, mouse_pos.y);

  for (int i = 0; i < 3; i++) {
    if (ImGui::IsMouseClicked(i)) {
      camera_control->mouse(p, i, true);
    }
    if (ImGui::IsMouseReleased(i)) {
      camera_control->mouse(p, i, false);
    }
    if (ImGui::IsMouseDragging(i)) {
      camera_control->drag(p, i);
    }

    if (io.KeyCtrl) {
      const double speed_factor = io.KeysDown[GLFW_KEY_LEFT_SHIFT] || io.KeysDown[GLFW_KEY_RIGHT_SHIFT] ? 4.0 : 1.0;
      const double speed = keyboard_control_speed * speed_factor;

      Eigen::Vector2f arrow(0, 0);
      if (io.KeysDown[GLFW_KEY_LEFT]) {
        arrow[0] = speed * dt;
      } else if (io.KeysDown[GLFW_KEY_RIGHT]) {
        arrow[0] = -speed * dt;
      }

      if (io.KeysDown[GLFW_KEY_UP]) {
        arrow[1] = speed * dt;
      } else if (io.KeysDown[GLFW_KEY_DOWN]) {
        arrow[1] = -speed * dt;
      }

      if (!arrow.isZero()) {
        camera_control->arrow(arrow);
      }

      double updown = 0;
      if (io.KeysDown[GLFW_KEY_PAGE_UP]) {
        updown = speed * dt;
      } else if (io.KeysDown[GLFW_KEY_PAGE_DOWN]) {
        updown = -speed * dt;
      }

      camera_control->updown(updown);

      if (io.KeysDown[GLFW_KEY_HOME]) {
        keyboard_control_speed *= std::pow(1.01, dt);
      } else if (io.KeysDown[GLFW_KEY_END]) {
        keyboard_control_speed *= std::pow(0.99, dt);
      }

      keyboard_control_speed = std::max(1.0, keyboard_control_speed);
    }

    camera_control->scroll(Eigen::Vector2f(io.MouseWheel, io.MouseWheelH));
    projection_control->set_depth_range(camera_control->depth_range());
  }

  camera_control->update();
}

/**
 * @brief
 *
 * @param p
 * @param window
 * @return Eigen::Vector4i
 */
Eigen::Vector4i GLCanvas::pick_info(const Eigen::Vector2i& p, int window) const {
  if (!info_buffer_id) {
    std::cerr << bold_yellow << "warning: info buffer has not been enabled!!" << reset << std::endl;
    return Eigen::Vector4i::Constant(-1);
  }

  if (p[0] < 5 || p[1] < 5 || p[0] > size[0] - 5 || p[1] > size[1] - 5) {
    return Eigen::Vector4i(-1, -1, -1, -1);
  }

  std::vector<int> pixels = frame_buffer->color(info_buffer_id).read_pixels<int>(GL_RGBA_INTEGER, GL_INT, 4);

  std::vector<Eigen::Vector2i> ps;

  for (int i = -window; i <= window; i++) {
    for (int j = -window; j <= window; j++) {
      ps.push_back(Eigen::Vector2i(i, j));
    }
  }

  std::sort(ps.begin(), ps.end(), [=](const Eigen::Vector2i& lhs, const Eigen::Vector2i& rhs) { return lhs.norm() < rhs.norm(); });
  for (int i = 0; i < ps.size(); i++) {
    Eigen::Vector2i p_ = p + ps[i];
    int index = ((size[1] - p[1]) * size[0] + p_[0]) * 4;
    Eigen::Vector4i info = Eigen::Map<Eigen::Vector4i>(&pixels[index]);

    if ((info.array() != -1).any()) {
      return info;
    }
  }

  return Eigen::Vector4i(-1, -1, -1, -1);
}

/**
 * @brief
 *
 * @param p
 * @param window
 * @return float
 */
float GLCanvas::pick_depth(const Eigen::Vector2i& p, int window) const {
  if (p[0] < 5 || p[1] < 5 || p[0] > size[0] - 5 || p[1] > size[1] - 5) {
    return -1.0f;
  }

  std::vector<float> pixels = frame_buffer->depth().read_pixels<float>(GL_DEPTH_COMPONENT, GL_FLOAT, 1);

  std::vector<Eigen::Vector2i> ps;

  for (int i = -window; i <= window; i++) {
    for (int j = -window; j <= window; j++) {
      ps.push_back(Eigen::Vector2i(i, j));
    }
  }

  std::sort(ps.begin(), ps.end(), [=](const Eigen::Vector2i& lhs, const Eigen::Vector2i& rhs) { return lhs.norm() < rhs.norm(); });
  for (int i = 0; i < ps.size(); i++) {
    Eigen::Vector2i p_ = p + ps[i];
    int index = ((size[1] - p[1]) * size[0] + p_[0]);
    float depth = pixels[index];

    if (depth < 1.0f) {
      return depth;
    }
  }

  return 1.0f;
}

/**
 * @brief
 *
 * @param p
 * @param depth
 * @return Eigen::Vector3f
 */
Eigen::Vector3f GLCanvas::unproject(const Eigen::Vector2i& p, float depth) const {
  Eigen::Matrix4f view_matrix = camera_control->view_matrix();
  Eigen::Matrix4f projection_matrix = projection_control->projection_matrix();

  glm::mat4 view = glm::make_mat4(view_matrix.data());
  glm::mat4 projection = glm::make_mat4(projection_matrix.data());

  glm::vec4 viewport = glm::vec4(0, 0, size[0], size[1]);
  glm::vec3 wincoord = glm::vec3(p[0], size[1] - p[1], depth);
  glm::vec3 objcoord = glm::unProject(wincoord, view, projection, viewport);

  return Eigen::Vector3f(objcoord.x, objcoord.y, objcoord.z);
}

void GLCanvas::draw_ui() {
  ImGui::Begin("shader setting");
  ImGui::End();
}

}  // namespace guik
