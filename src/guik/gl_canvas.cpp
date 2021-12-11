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
#include <guik/camera/orbit_camera_control_xy.hpp>

namespace guik {

using namespace glk::console;

/**
 * @brief Construct a new GLCanvas object
 *
 * @param size
 */
GLCanvas::GLCanvas(const Eigen::Vector2i& size, const std::string& shader_name)
    : size(size), clear_color(0.27f, 0.27f, 0.27f, 1.0f) {
  frame_buffer.reset(new glk::FrameBuffer(size, 1));
  shader.reset(new glk::GLSLShader());
  if(!shader->init(glk::get_data_path() + "/shader/" + shader_name)) {
    shader.reset();
    return;
  }

  shader->use();

  shader->set_uniform("point_size", 10.0f);
  shader->set_uniform("point_scale", 1.0f);
  shader->set_uniform("point_size_offset", 0.0f);

  shader->set_uniform("model_matrix", Eigen::Matrix4f::Identity().eval());

  shader->set_uniform("color_mode", 0);
  shader->set_uniform("material_color", Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f));
  shader->set_uniform("z_range", Eigen::Vector2f(-3.0f, 5.0f));
  shader->set_uniform("colormap_axis", Eigen::Vector3f(0.0f, 0.0f, 1.0f));

  shader->set_uniform("colormap_sampler", 0);
  shader->set_uniform("texture_sampler", 1);

  auto colormap_table = glk::colormap_table(glk::COLORMAP::TURBO);
  colormap.reset(new glk::Texture(Eigen::Vector2i(256, 1), GL_RGBA, GL_RGB, GL_UNSIGNED_BYTE, colormap_table.data()));

  camera_control.reset(new guik::OrbitCameraControlXY());
  projection_control.reset(new guik::ProjectionControl(size));

  texture_renderer.reset(new glk::TextureRenderer());

  normal_buffer_id = info_buffer_id = 0;
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
  if(!shader->init(glk::get_data_path() + "/shader/" + shader_name)) {
    shader.reset();
    return false;
  }

  shader->use();

  shader->set_uniform("point_size", 10.0f);
  shader->set_uniform("point_scale", 1.0f);
  shader->set_uniform("point_size_offset", 0.0f);

  shader->set_uniform("model_matrix", Eigen::Matrix4f::Identity().eval());

  shader->set_uniform("color_mode", 0);
  shader->set_uniform("material_color", Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f));
  shader->set_uniform("z_range", Eigen::Vector2f(-3.0f, 5.0f));
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
  if(!screen_effect_buffer) {
    screen_effect_buffer.reset(new glk::FrameBuffer(size, 1, false));
  }
}

const std::shared_ptr<glk::ScreenEffect>& GLCanvas::get_effect() const {
  return screen_effect;
}

void GLCanvas::enable_normal_buffer() {
  normal_buffer_id = frame_buffer->num_color_buffers();
  frame_buffer->add_color_buffer(2, GL_RGB32F, GL_RGB, GL_FLOAT);
}

void GLCanvas::enable_info_buffer() {
  info_buffer_id = frame_buffer->num_color_buffers();
  frame_buffer->add_color_buffer(1, GL_RGBA32I, GL_RGBA_INTEGER, GL_INT);
}

bool GLCanvas::normal_buffer_enabled() const {
  return normal_buffer_id > 0;
}

bool GLCanvas::info_buffer_enabled() const {
  return info_buffer_id > 0;
}

const glk::Texture& GLCanvas::normal_buffer() const {
  return frame_buffer->color(normal_buffer_id);
}

const glk::Texture& GLCanvas::info_buffer() const {
  return frame_buffer->color(info_buffer_id);
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

  if(screen_effect) {
    screen_effect->set_size(size);
  }

  if(screen_effect_buffer) {
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
void GLCanvas::bind(bool clear_buffer) {
  frame_buffer->bind();
  glDisable(GL_SCISSOR_TEST);
  if(clear_buffer) {
    glClearColor(clear_color[0], clear_color[1], clear_color[2], clear_color[3]);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  }

  Eigen::Matrix4f view_matrix = camera_control->view_matrix();
  Eigen::Matrix4f projection_matrix = projection_control->projection_matrix();

  shader->use();
  shader->set_uniform("view_matrix", view_matrix);
  shader->set_uniform("inv_view_matrix", view_matrix.inverse().eval());
  shader->set_uniform("projection_matrix", projection_matrix);
  shader->set_uniform("info_enabled", info_buffer_id > 0);
  shader->set_uniform("normal_enabled", normal_buffer_id > 0);

  shader->set_uniform("colormap_sampler", 0);
  shader->set_uniform("texture_sampler", 1);

  if(normal_buffer_id && clear_buffer) {
    GLfloat clear_color[] = {0.0f, 0.0f, 0.0f};
    glClearTexImage(frame_buffer->color(normal_buffer_id).id(), 0, GL_RGB, GL_FLOAT, clear_color);
  }

  if(info_buffer_id && clear_buffer) {
    GLint clear_color[] = {-1, -1, -1, -1};
    glClearTexImage(frame_buffer->color(info_buffer_id).id(), 0, GL_RGBA_INTEGER, GL_INT, clear_color);
    shader->set_uniform("info_values", Eigen::Vector4i(-1, -1, -1, -1));
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

  if(screen_effect) {
    glk::TextureRendererInput::Ptr input(new glk::TextureRendererInput());
    input->set("view_matrix", camera_control->view_matrix());
    input->set("projection_matrix", projection_control->projection_matrix());

    if(normal_buffer_id) {
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
  if(info_buffer_id) {
    shader->set_uniform("info_values", Eigen::Vector4i(-1, -1, -1, -1));
  }

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
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
  ImGuiIO& io = ImGui::GetIO();
  auto mouse_pos = ImGui::GetMousePos();
  auto drag_delta = ImGui::GetMouseDragDelta();

  Eigen::Vector2i p(mouse_pos.x, mouse_pos.y);

  for(int i = 0; i < 3; i++) {
    if(ImGui::IsMouseClicked(i)) {
      camera_control->mouse(p, i, true);
    }
    if(ImGui::IsMouseReleased(i)) {
      camera_control->mouse(p, i, false);
    }
    if(ImGui::IsMouseDragging(i)) {
      camera_control->drag(p, i);
    }

    if(io.KeyCtrl) {
      Eigen::Vector2i arrow(0, 0);
      if(io.KeysDown[GLFW_KEY_LEFT]) {
        arrow[0] = 1;
      } else if(io.KeysDown[GLFW_KEY_RIGHT]) {
        arrow[0] = -1;
      }

      if(io.KeysDown[GLFW_KEY_UP]) {
        arrow[1] = 1;
      } else if(io.KeysDown[GLFW_KEY_DOWN]) {
        arrow[1] = -1;
      }

      if(!arrow.isZero()) {
        camera_control->arrow(arrow);
      }

      int updown = 0;
      if(io.KeysDown[GLFW_KEY_PAGE_UP]) {
        updown = 1;
      } else if(io.KeysDown[GLFW_KEY_PAGE_DOWN]) {
        updown = -1;
      }

      camera_control->updown(updown);
    }

    camera_control->scroll(Eigen::Vector2f(io.MouseWheel, io.MouseWheelH));
    projection_control->set_depth_range(camera_control->depth_range());
  }
}

/**
 * @brief
 *
 * @param p
 * @param window
 * @return Eigen::Vector4i
 */
Eigen::Vector4i GLCanvas::pick_info(const Eigen::Vector2i& p, int window) const {
  if(!info_buffer_id) {
    std::cerr << bold_yellow << "warning: info buffer has not been enabled!!" << reset << std::endl;
    return Eigen::Vector4i::Constant(-1);
  }

  if(p[0] < 5 || p[1] < 5 || p[0] > size[0] - 5 || p[1] > size[1] - 5) {
    return Eigen::Vector4i(-1, -1, -1, -1);
  }

  std::vector<int> pixels = frame_buffer->color(info_buffer_id).read_pixels<int>(GL_RGBA_INTEGER, GL_INT);

  std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> ps;

  for(int i = -window; i <= window; i++) {
    for(int j = -window; j <= window; j++) {
      ps.push_back(Eigen::Vector2i(i, j));
    }
  }

  std::sort(ps.begin(), ps.end(), [=](const Eigen::Vector2i& lhs, const Eigen::Vector2i& rhs) { return lhs.norm() < rhs.norm(); });
  for(int i = 0; i < ps.size(); i++) {
    Eigen::Vector2i p_ = p + ps[i];
    int index = ((size[1] - p[1]) * size[0] + p_[0]) * 4;
    Eigen::Vector4i info = Eigen::Map<Eigen::Vector4i>(&pixels[index]);

    if((info.array() != -1).any()) {
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
  if(p[0] < 5 || p[1] < 5 || p[0] > size[0] - 5 || p[1] > size[1] - 5) {
    return -1.0f;
  }

  std::vector<float> pixels = frame_buffer->depth().read_pixels<float>(GL_DEPTH_COMPONENT, GL_FLOAT);

  std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> ps;

  for(int i = -window; i <= window; i++) {
    for(int j = -window; j <= window; j++) {
      ps.push_back(Eigen::Vector2i(i, j));
    }
  }

  std::sort(ps.begin(), ps.end(), [=](const Eigen::Vector2i& lhs, const Eigen::Vector2i& rhs) { return lhs.norm() < rhs.norm(); });
  for(int i = 0; i < ps.size(); i++) {
    Eigen::Vector2i p_ = p + ps[i];
    int index = ((size[1] - p[1]) * size[0] + p_[0]);
    float depth = pixels[index];

    if(depth < 1.0f) {
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
