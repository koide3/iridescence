#include <spdlog/spdlog.h>
#include <spdlog/sinks/ringbuffer_sink.h>
#include <guik/spdlog_sink.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  auto ringbuffer_sink = std::make_shared<spdlog::sinks::ringbuffer_sink_mt>(128);
  spdlog::default_logger()->sinks().emplace_back(ringbuffer_sink);
  guik::create_logger_ui(ringbuffer_sink);

  return 0;
}

