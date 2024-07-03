#ifndef GLK_PROFILER_HPP
#define GLK_PROFILER_HPP

#include <chrono>
#include <string>
#include <vector>
#include <iomanip>
#include <sstream>
#include <iostream>

#include <GL/gl3w.h>

namespace glk {

class GLProfiler {
public:
  GLProfiler(const std::string& prof_name, bool enabled = true, int max_num_queries = 16) : prof_name(prof_name), enabled(enabled) {
    if (!enabled) {
      return;
    }
    queries.resize(max_num_queries);
    glGenQueries(max_num_queries, queries.data());
  }

  ~GLProfiler() {
    if (!enabled) {
      return;
    }

    if (labels.empty()) {
      glDeleteQueries(queries.size(), queries.data());
      return;
    }

    glEndQuery(GL_TIME_ELAPSED);

    std::stringstream sst;
    sst << "--- " << prof_name << " ---\n";

    int max_name_length = 0;
    for (const auto& label : labels) {
      max_name_length = std::max<int>(max_name_length, label.size());
    }
    std::string label_format = "%-" + std::to_string(max_name_length) + "s";

    double sum_time_msec = 0.0;
    for (int i = 0; i < labels.size(); i++) {
      int result = 0;
      glGetQueryObjectiv(queries[i], GL_QUERY_RESULT, &result);
      double time_msec = result / 1e6;
      sum_time_msec += time_msec;

      sst << "- " << std::left << std::setfill(' ') << std::setw(max_name_length) << labels[i] << " : " << std::fixed << std::setprecision(3) << time_msec << "[msec] ("
          << std::fixed << std::setprecision(3) << sum_time_msec << "[msec])\n";
    }
    sst << "***\n";
    sst << "- total(approx) : " << std::fixed << std::setprecision(3) << sum_time_msec << "[msec]";

    std::cout << sst.str() << std::endl;

    glDeleteQueries(queries.size(), queries.data());
  }

  void add(const std::string& label) {
    if (!enabled) {
      return;
    }

    int current = labels.size();
    if (current != 0) {
      glEndQuery(GL_TIME_ELAPSED);
    }

    if (current == queries.size()) {
      int n = queries.size();
      queries.resize(n * 2);
      glGenQueries(n, queries.data() + n);
    }

    glBeginQuery(GL_TIME_ELAPSED, queries[current]);
    labels.push_back(label);
  }

private:
  const std::string prof_name;
  const bool enabled;

  std::vector<std::string> labels;
  std::vector<GLuint> queries;
};

class RealProfiler {
public:
  RealProfiler(const std::string& prof_name, bool enabled = true) : prof_name(prof_name), enabled(enabled) {}

  ~RealProfiler() {
    if (!enabled) {
      return;
    }

    if (labels.empty()) {
      return;
    }

    labels.push_back("end");
    times.push_back(std::chrono::high_resolution_clock::now());

    std::stringstream sst;
    sst << "--- " << prof_name << " ---\n";

    int max_name_length = 0;
    for (const auto& label : labels) {
      max_name_length = std::max<int>(max_name_length, label.size());
    }

    for (int i = 0; i < labels.size() - 1; i++) {
      double time_msec = std::chrono::duration_cast<std::chrono::nanoseconds>(times[i + 1] - times[i]).count() / 1e6;
      double sum_time_msec = std::chrono::duration_cast<std::chrono::nanoseconds>(times[i + 1] - times.front()).count() / 1e6;

      sst << "- " << std::left << std::setfill(' ') << std::setw(max_name_length) << labels[i] << " : " << std::fixed << std::setprecision(3) << time_msec << "[msec] ("
          << std::fixed << std::setprecision(3) << sum_time_msec << "[msec])\n";
    }

    sst << "***\n";
    double sum_time_msec = std::chrono::duration_cast<std::chrono::nanoseconds>(times.back() - times.front()).count() / 1e6;

    sst << " - total(approx) : " << std::fixed << std::setprecision(3) << sum_time_msec << "[msec]";
    std::cout << sst.str() << std::endl;
  }

  void add(const std::string& label) {
    if (!enabled) {
      return;
    }

    labels.push_back(label);
    times.push_back(std::chrono::high_resolution_clock::now());
  }

private:
  const std::string prof_name;
  const bool enabled;

  std::vector<std::string> labels;
  std::vector<std::chrono::high_resolution_clock::time_point> times;
};

class GLRealProfiler {
public:
  GLRealProfiler(const std::string& prof_name, bool enabled = true, int max_num_queries = 16) : prof_name(prof_name), enabled(enabled) {
    if (!enabled) {
      return;
    }
    queries.resize(max_num_queries);
    glGenQueries(max_num_queries, queries.data());
  }

  ~GLRealProfiler() {
    if (!enabled) {
      return;
    }

    if (labels.empty()) {
      glDeleteQueries(queries.size(), queries.data());
      return;
    }

    glEndQuery(GL_TIME_ELAPSED);
    times.push_back(std::chrono::high_resolution_clock::now());

    std::stringstream sst;
    sst << "--- " << prof_name << " ---\n";

    int max_name_length = 0;
    for (const auto& label : labels) {
      max_name_length = std::max<int>(max_name_length, label.size());
    }
    std::string label_format = "%-" + std::to_string(max_name_length) + "s";

    double sum_time_msec_gl = 0.0;
    for (int i = 0; i < labels.size(); i++) {
      int result = 0;
      glGetQueryObjectiv(queries[i], GL_QUERY_RESULT, &result);
      double time_msec_gl = result / 1e6;
      sum_time_msec_gl += time_msec_gl;

      double time_msec_real = std::chrono::duration_cast<std::chrono::nanoseconds>(times[i + 1] - times[i]).count() / 1e6;
      double sum_time_msec_real = std::chrono::duration_cast<std::chrono::nanoseconds>(times[i + 1] - times.front()).count() / 1e6;

      sst << "- " << std::left << std::setfill(' ') << std::setw(max_name_length) << labels[i] << " : " << std::fixed << std::setprecision(3) << time_msec_gl << "[msec] " << "r"
          << std::fixed << std::setprecision(3) << time_msec_real << "[msec] (" << std::fixed << std::setprecision(3) << sum_time_msec_gl << "[msec] r" << std::fixed
          << std::setprecision(3) << sum_time_msec_real << "[msec])\n";
    }
    sst << "***\n";
    double sum_time_msec_real = std::chrono::duration_cast<std::chrono::nanoseconds>(times.back() - times.front()).count() / 1e6;

    sst << "- " << std::left << std::setfill(' ') << std::setw(max_name_length) << labels.back() << " : " << std::fixed << std::setprecision(3) << 0.0 << "[msec] " << "r"
        << std::fixed << std::setprecision(3) << 0.0 << "[msec] (" << std::fixed << std::setprecision(3) << sum_time_msec_gl << "[msec] r" << std::fixed << std::setprecision(3)
        << sum_time_msec_real << "[msec])\n";

    std::cout << sst.str() << std::endl;

    glDeleteQueries(queries.size(), queries.data());
  }

  void add(const std::string& label) {
    if (!enabled) {
      return;
    }

    int current = labels.size();
    if (current != 0) {
      glEndQuery(GL_TIME_ELAPSED);
    }

    if (current == queries.size()) {
      int n = queries.size();
      queries.resize(n * 2);
      glGenQueries(n, queries.data() + n);
    }

    glBeginQuery(GL_TIME_ELAPSED, queries[current]);
    labels.push_back(label);
    times.push_back(std::chrono::high_resolution_clock::now());
  }

private:
  const std::string prof_name;
  const bool enabled;

  std::vector<std::string> labels;
  std::vector<GLuint> queries;
  std::vector<std::chrono::high_resolution_clock::time_point> times;
};

}  // namespace glk

#endif