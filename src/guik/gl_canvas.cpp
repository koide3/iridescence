#include <guik/gl_canvas.hpp>

#include <imgui.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <glk/path.hpp>
#include <glk/glsl_shader.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/texture_renderer.hpp>
#include <glk/effects/plain_rendering.hpp>
#include <glk/effects/screen_space_ambient_occlusion.hpp>

#include <guik/camera_control.hpp>

namespace guik {

/**
 * @brief Construct a new GLCanvas object
 *
 * @param size
 */
GLCanvas::GLCanvas(const Eigen::Vector2i& size, const std::string& shader_name) : size(size) {
  frame_buffer.reset(new glk::FrameBuffer(size, 3));
  shader.reset(new glk::GLSLShader());
  if(!shader->init(glk::get_data_path() + "/shader/" + shader_name)) {
    shader.reset();
    return;
  }

  shader->use();

  shader->set_uniform("point_size", 10.0f);
  shader->set_uniform("point_scale", 1.0f);

  shader->set_uniform("model_matrix", Eigen::Matrix4f::Identity().eval());

  shader->set_uniform("color_mode", 0);
  shader->set_uniform("material_color", Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f));
  shader->set_uniform("z_range", Eigen::Vector2f(-3.0f, 5.0f));

  camera_control.reset(new guik::ArcCameraControl());
  projection_control.reset(new guik::ProjectionControl(size));
  texture_renderer.reset(new glk::TextureRenderer());
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool GLCanvas::ready() const { return frame_buffer && shader && camera_control && texture_renderer; }

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

  shader->set_uniform("model_matrix", Eigen::Matrix4f::Identity().eval());

  shader->set_uniform("color_mode", 0);
  shader->set_uniform("material_color", Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f));
  shader->set_uniform("z_range", Eigen::Vector2f(-3.0f, 5.0f));

  return true;
}

void GLCanvas::set_effect(const std::shared_ptr<glk::ScreenEffect>& effect) {
  texture_renderer->set_effect(effect);
}

/**
 * @brief Set the Size object
 *
 * @param size
 */
void GLCanvas::set_size(const Eigen::Vector2i& size) {
  this->size = size;
  projection_control->set_size(size);
  frame_buffer.reset(new glk::FrameBuffer(size, 2));
}

/**
 * @brief
 *
 */
void GLCanvas::bind() {
  frame_buffer->bind();
  glDisable(GL_SCISSOR_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // GLint clear_color[] = {-1, -1, -1, -1};
  // glClearTexImage(frame_buffer->color(1).id(), 0, GL_RGBA_INTEGER, GL_INT, clear_color);

  shader->use();

  Eigen::Matrix4f view_matrix = camera_control->view_matrix();
  Eigen::Matrix4f projection_matrix = projection_control->projection_matrix();

  shader->set_uniform("view_matrix", view_matrix);
  shader->set_uniform("inv_view_matrix", view_matrix.inverse().eval());
  shader->set_uniform("projection_matrix", projection_matrix);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
}

/**
 * @brief
 *
 */
void GLCanvas::unbind() {
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
  texture_renderer->draw(frame_buffer->color(color_buffer_id), frame_buffer->depth());
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
  if (p[0] < 5 || p[1] < 5 || p[0] > size[0] - 5 || p[1] > size[1] - 5) {
    return Eigen::Vector4i(-1, -1, -1, -1);
  }

  std::vector<int> pixels = frame_buffer->color(1).read_pixels<int>(GL_RGBA_INTEGER, GL_INT);

  std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> ps;

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

    if (info[3] >= 0) {
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

  std::vector<float> pixels = frame_buffer->depth().read_pixels<float>(GL_DEPTH_COMPONENT, GL_FLOAT);

  std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> ps;

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

  Eigen::Matrix4f vp = projection_matrix * view_matrix;

  Eigen::Vector2f p_(static_cast<float>(p[0]) / size[0], static_cast<float>(p[1]) / size[1]);
  Eigen::Vector4f unprojected = vp.inverse() * Eigen::Vector4f(p_[0], 1.0f - p_[1], depth, 1.0f);

  return unprojected.head<3>();
}

void GLCanvas::draw_ui() {
  ImGui::Begin("shader setting");
  ImGui::End();
}

}  // namespace guik
