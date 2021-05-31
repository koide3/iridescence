#ifndef GLK_QUERY_HPP
#define GLK_QUERY_HPP

#include <GL/gl3w.h>

namespace glk {

class Query {
public:
  Query();
  ~Query();

  void begin(GLenum target);
  void end();

  template<typename T>
  T get() const;

private:
  GLuint query;
  GLenum target;
};
}  // namespace glk

#endif