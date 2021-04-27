#ifndef GLK_PROFILER_HPP
#define GLK_PROFILER_HPP

#include <string>
#include <vector>
#include <iostream>
#include <boost/format.hpp>

#include <GL/gl3w.h>

namespace glk {

class Profiler {
public:
  Profiler(const std::string& prof_name, int max_num_queries = 16) : prof_name(prof_name) {
    queries.resize(max_num_queries);
    glGenQueries(max_num_queries, queries.data());
  }

  ~Profiler() {
    if(labels.empty()) {
      glDeleteQueries(queries.size(), queries.data());
      return;
    }

    glEndQuery(GL_TIME_ELAPSED);

    std::cout << "--- " << prof_name << " ---" << std::endl;

    int max_name_length = 0;
    for(const auto& label : labels) {
      max_name_length = std::max<int>(max_name_length, label.size());
    }
    std::string label_format = "\%-" + std::to_string(max_name_length) + "s";

    double sum_time_msec = 0.0;
    for(int i = 0; i < labels.size(); i++) {
      int result = 0;
      glGetQueryObjectiv(queries[i], GL_QUERY_RESULT, &result);
      double time_msec = result / 1e6;
      sum_time_msec += time_msec;

      std::cout << boost::format(label_format) % labels[i] << boost::format(":%.3f[msec] (%.3f[msec])") % time_msec % sum_time_msec << std::endl;
    }
    std::cout << "***" << std::endl;
    std::cout << boost::format(label_format) % "total(approx):" << boost::format("%.3f") % sum_time_msec << "[msec]" << std::endl;

    glDeleteQueries(queries.size(), queries.data());
  }

  void add(const std::string& label) {
    int current = labels.size();
    if(current != 0) {
      glEndQuery(GL_TIME_ELAPSED);
    }

    if(current == queries.size()) {
      int n = queries.size();
      queries.resize(n * 2);
      glGenQueries(n, queries.data() + n);
    }

    glBeginQuery(GL_TIME_ELAPSED, queries[current]);
    labels.push_back(label);
  }

private:
  std::string prof_name;

  std::vector<std::string> labels;
  std::vector<GLuint> queries;
};
}  // namespace glk

#endif