#include <guik/viewer/light_viewer.hpp>

/**
 * @brief Custom drawable class that directly use OpenGL commands to render points
*/
class CustomDrawable : public glk::Drawable {
public:
  CustomDrawable() {
    std::vector<Eigen::Vector3f> vertices;
    std::vector<Eigen::Vector4f> colors;

    for(int x = 0; x <= 5; x++) {
      for(int y = 0; y <= 5; y++) {
        for(int z = 0; z <= 5; z++) {
          vertices.push_back(Eigen::Vector3f(x - 2.5f, y - 2.5f, z));
          colors.push_back(Eigen::Vector4f(x / 5.0f, y / 5.0f, z / 5.0f, 1.0f));
        }
      }
    }

    num_points = vertices.size();

    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * num_points, vertices.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &cbo);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4 * num_points, colors.data(), GL_STATIC_DRAW);
  }

  virtual ~CustomDrawable() {
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &cbo);
  }

  void draw(glk::GLSLShader& shader) const override {
    shader.set_uniform("color_mode", guik::ColorMode::VERTEX_COLOR);
    shader.set_uniform("point_scale", 30.0f);

    GLint position_loc = shader.attrib("vert_position");
    GLint color_loc = shader.attrib("vert_color");

    glBindVertexArray(vao);
    glEnableVertexAttribArray(position_loc);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glEnableVertexAttribArray(color_loc);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glVertexAttribPointer(color_loc, 4, GL_FLOAT, GL_FALSE, 0, 0);

    glDrawArrays(GL_POINTS, 0, num_points);

    glDisableVertexAttribArray(position_loc);
    glDisableVertexAttribArray(color_loc);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glDisableVertexAttribArray(0);
  }

private:
  int num_points;

  GLuint vao;  // vertex array object
  GLuint vbo;  // vertex buffer object
  GLuint cbo;  // color buffer object
};

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();
  viewer->update_drawable("points", std::make_shared<CustomDrawable>());
  viewer->spin();
  return 0;
}