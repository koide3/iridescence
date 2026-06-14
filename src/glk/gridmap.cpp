#include <glk/gridmap.hpp>
#include <glk/texture.hpp>
#include <glk/colormap.hpp>

namespace glk {

GridMap::GridMap(double resolution, int width, int height, const unsigned char* values, int alpha, ColorMode mode) {
  update_color(width, height, values, alpha, mode);
  init_vao(resolution, width, height);
}

GridMap::GridMap(double resolution, int width, int height, float scale, const float* values, float alpha, ColorMode mode) {
  update_color(width, height, scale, values, alpha, mode);
  init_vao(resolution, width, height);
}

void GridMap::update_color(const unsigned char* values, int alpha, ColorMode mode) {
  if (texture == nullptr) return;
  update_color(texture->size().x(), texture->size().y(), values, alpha, mode);
}

void GridMap::update_color(int width, int height, const unsigned char* values, int alpha, ColorMode mode) {
  std::vector<unsigned char> rgba(width * height * 4);
  for (int i = 0; i < width * height; i++) {
    unsigned char x = values[i];
    Eigen::Map<Eigen::Matrix<unsigned char, 3, 1>> rgb(rgba.data() + i * 4);

    switch (mode) {
      case ColorMode::RAW:
        rgb.setConstant(x);
        break;
      case ColorMode::TURBO:
        rgb = glk::colormap(glk::COLORMAP::TURBO, x).cast<unsigned char>().head<3>();
        break;
      case ColorMode::PROB:
        rgb.setConstant(100 - x);
        break;
      case ColorMode::PROB_TURBO:
        rgb = glk::colormap(glk::COLORMAP::TURBO, (100 - x) * 255.0 / 100.0).cast<unsigned char>().head<3>();
        break;
      case ColorMode::RGBA:
        std::copy(values + i * 4, values + i * 4 + 4, rgba.begin() + i * 4);
        break;
    }
    if (mode != ColorMode::RGBA) {
      rgba[i * 4 + 3] = alpha;
    }
  }

  texture.reset(new Texture(Eigen::Vector2i(width, height), GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data()));
  texture->bind();
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  texture->unbind();
}

void GridMap::update_color(float scale, const float* values, float alpha, ColorMode mode) {
  if (texture == nullptr) return;
  update_color(texture->size().x(), texture->size().y(), scale, values, alpha, mode);
}

void GridMap::update_color(int width, int height, float scale, const float* values, float alpha, ColorMode mode) {
  std::vector<float> rgba(width * height * 4);
  for (int i = 0; i < width * height; i++) {
    float x = scale * values[i];
    Eigen::Map<Eigen::Vector3f> rgb(rgba.data() + i * 4);

    switch (mode) {
      case ColorMode::RAW:
        rgb.setConstant(x);
        break;
      case ColorMode::TURBO:
        rgb = glk::colormapf(glk::COLORMAP::TURBO, x).head<3>();
        break;
      case ColorMode::PROB:
        rgb.setConstant(1.0f - x);
        break;
      case ColorMode::PROB_TURBO:
        rgb = glk::colormapf(glk::COLORMAP::TURBO, (1.0f - x)).head<3>();
        break;
      case ColorMode::RGBA:
        std::copy(values + i * 4, values + i * 4 + 4, rgba.begin() + i * 4);
        break;
    }
    if (mode != ColorMode::RGBA) {
      rgba[i * 4 + 3] = alpha;
    }
  }

  texture.reset(new Texture(Eigen::Vector2i(width, height), GL_RGBA, GL_RGBA, GL_FLOAT, rgba.data()));
  texture->bind();
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  texture->unbind();
}

GridMap::~GridMap() {
  glDeleteBuffers(1, &vao);
  glDeleteBuffers(1, &vbo);
  glDeleteBuffers(1, &tbo);
}

void GridMap::init_vao(double resolution, int width, int height) {
  std::vector<Eigen::Vector3f> vertices;
  std::vector<Eigen::Vector2f> texcoords;

  vertices.push_back(Eigen::Vector3f(-resolution * width / 2, -resolution * height / 2, 0.0f));
  vertices.push_back(Eigen::Vector3f(resolution * width / 2, -resolution * height / 2, 0.0f));
  vertices.push_back(Eigen::Vector3f(-resolution * width / 2, resolution * height / 2, 0.0f));
  vertices.push_back(Eigen::Vector3f(resolution * width / 2, resolution * height / 2, 0.0f));

  texcoords.push_back(Eigen::Vector2f(0.0f, 0.0f));
  texcoords.push_back(Eigen::Vector2f(1.0f, 0.0f));
  texcoords.push_back(Eigen::Vector2f(0.0f, 1.0f));
  texcoords.push_back(Eigen::Vector2f(1.0f, 1.0f));

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(Eigen::Vector3f) * vertices.size(), vertices.data(), GL_STATIC_DRAW);

  glGenBuffers(1, &tbo);
  glBindBuffer(GL_ARRAY_BUFFER, tbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(Eigen::Vector2f) * texcoords.size(), texcoords.data(), GL_STATIC_DRAW);
}

void GridMap::draw(glk::GLSLShader& shader) const {
  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D, texture->id());
  glActiveTexture(GL_TEXTURE0);

  glBindVertexArray(vao);

  GLint position_loc = shader.attrib("vert_position");
  glEnableVertexAttribArray(position_loc);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);

  GLint texcoord_loc = shader.attrib("vert_texcoord");
  glEnableVertexAttribArray(texcoord_loc);
  glBindBuffer(GL_ARRAY_BUFFER, tbo);
  glVertexAttribPointer(texcoord_loc, 2, GL_FLOAT, GL_FALSE, 0, 0);

  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

  glDisableVertexAttribArray(position_loc);
  glDisableVertexAttribArray(texcoord_loc);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);

  // texture->unbind(GL_TEXTURE1);
}

}  // namespace glk