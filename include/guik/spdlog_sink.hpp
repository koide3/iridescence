#ifndef GUIK_SPDLOG_SINK_HPP
#define GUIK_SPDLOG_SINK_HPP

#include <deque>
#include <spdlog/sinks/base_sink.h>
#include <guik/viewer/light_viewer.hpp>

struct TextBuffer {
public:
  size_t size() const { return texts.size(); }

  template <typename String>
  void emplate_back(const spdlog::level::level_enum level, const String& text) {
    texts.emplace_back(level, text);
  }

  void pop_front() { texts.pop_front(); }

  std::mutex mutex;
  std::deque<std::pair<spdlog::level::level_enum, std::string>> texts;
};

namespace guik {
template <typename Mutex>
class spdlog_sink : public spdlog::sinks::base_sink<Mutex> {
public:
  spdlog_sink(const int max_num_texts = 1024) : max_num_texts(max_num_texts), texts(new TextBuffer) {}

  std::function<void()> create_callback() const {
    return [texts = this->texts] {
      ImGui::SetNextWindowSize(ImVec2(500, 400), ImGuiCond_FirstUseEver);
      if (!ImGui::Begin("logging", nullptr)) {
        ImGui::End();
        return;
      }

      if (ImGui::BeginChild("scrolling", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar)) {
        std::array<ImVec4, spdlog::level::n_levels> colors;
        colors[static_cast<int>(spdlog::level::trace)] = ImVec4(0.7f, 0.7f, 0.7f, 0.5f);
        colors[static_cast<int>(spdlog::level::debug)] = ImVec4(0.6f, 0.9f, 1.0f, 0.7f);
        colors[static_cast<int>(spdlog::level::info)] = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
        colors[static_cast<int>(spdlog::level::warn)] = ImVec4(1.0f, 1.0f, 0.0f, 1.0f);
        colors[static_cast<int>(spdlog::level::err)] = ImVec4(1.0f, 0.5f, 0.0f, 1.0f);
        colors[static_cast<int>(spdlog::level::critical)] = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
        colors[static_cast<int>(spdlog::level::off)] = ImVec4(0.3f, 0.3f, 0.3f, 1.0f);

        std::lock_guard<std::mutex> lock(texts->mutex);
        for (const auto& log : texts->texts) {
          const auto& level = log.first;
          const auto& text = log.second;

          const auto& color = colors[static_cast<int>(level)];
          ImGui::TextColored(color, text.c_str());
        }

        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) {
          ImGui::SetScrollHereY(1.0f);
        }
      }

      ImGui::EndChild();
      ImGui::End();
    };
  }

protected:
  void sink_it_(const spdlog::details::log_msg& msg) override {
    spdlog::memory_buf_t formatted;
    spdlog::sinks::base_sink<Mutex>::formatter_->format(msg, formatted);

    std::lock_guard<std::mutex> lock(texts->mutex);
    while (max_num_texts && texts->texts.size() > max_num_texts) {
      texts->pop_front();
    }
    texts->emplate_back(msg.level, fmt::to_string(formatted));
  }

  void flush_() override {}

private:
  int max_num_texts;
  std::shared_ptr<TextBuffer> texts;
};

#include "spdlog/details/null_mutex.h"
#include <mutex>
using spdlog_sink_mt = spdlog_sink<std::mutex>;
using spdlog_sink_st = spdlog_sink<spdlog::details::null_mutex>;

}  // namespace guik

#endif