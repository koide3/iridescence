#ifndef GLK_PLAIN_EFFECT_HPP
#define GLK_PLAIN_EFFECT_HPP

#include <glk/path.hpp>
#include <glk/effects/screen_effect.hpp>

namespace glk {

class PlainRendering : public ScreenEffect {
public:
  PlainRendering() {
    if(!plain_shader.init(get_data_path() + "/shader/texture")) {
      return;
    }
  }
  virtual ~PlainRendering() {}

  virtual glk::GLSLShader& shader(PASS_TYPE pass) override {
    return plain_shader;
  }

private:
  glk::GLSLShader plain_shader;
};

}  // namespace glk

#endif