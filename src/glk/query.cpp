#include <glk/query.hpp>

namespace glk {

Query::Query() {
  glGenQueries(1, &query);
}

Query ::~Query() {
  glDeleteQueries(1, &query);
}

void Query::begin(GLenum target) {
  this->target = target;
  glBeginQuery(target, query);
}

void Query::end() {
  glEndQuery(target);
}

template<>
int Query::get() const {
  int result;
  glGetQueryObjectiv(query, GL_QUERY_RESULT, &result);
  return result;
}

}