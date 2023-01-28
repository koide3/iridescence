#ifndef GUIK_SPDLOG_SINK_HPP
#define GUIK_SPDLOG_SINK_HPP

#include <deque>
#include <imgui.h>
#include <spdlog/spdlog.h>

#if SPDLOG_VERSION >= 10600
#include <spdlog/pattern_formatter.h>
#else
#include <spdlog/details/pattern_formatter.h>
#endif

namespace guik {

/**
 * usage:
 *
 * auto ringbuffer_sink = std::make_shared<spdlog::sinks::ringbuffer_sink_mt>(128);
 * spdlog::default_logger()->sinks().emplace_back(ringbuffer_sink);
 * viewer->register_ui_callback("logging", guik::create_logger_ui(ringbuffer_sink));
 */
template <typename RingBufferSink>
std::function<void()> create_logger_ui(const std::shared_ptr<RingBufferSink>& sink, double bg_alpha = 1.0) {
  auto enabled = std::make_shared<bool>(true);
  return [=] {
    if (!(*enabled)) {
      return;
    }

    const auto log_messages = sink->last_raw();

    ImGui::SetNextWindowSize(ImVec2(660, 400), ImGuiCond_FirstUseEver);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, bg_alpha));
    ImGui::Begin("logging", enabled.get());
    ImGui::PopStyleColor();

    if (ImGui::BeginChild("scrolling", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar)) {
      std::array<ImVec4, 6> colors;
      colors[static_cast<int>(spdlog::level::trace)] = ImVec4(0.7f, 0.7f, 0.7f, 0.5f);
      colors[static_cast<int>(spdlog::level::debug)] = ImVec4(0.6f, 0.9f, 1.0f, 0.7f);
      colors[static_cast<int>(spdlog::level::info)] = ImVec4(0.9f, 1.0f, 0.9f, 1.0f);
      colors[static_cast<int>(spdlog::level::warn)] = ImVec4(1.0f, 1.0f, 0.0f, 1.0f);
      colors[static_cast<int>(spdlog::level::err)] = ImVec4(1.0f, 0.5f, 0.0f, 1.0f);
      colors[static_cast<int>(spdlog::level::critical)] = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);

      spdlog::pattern_formatter formatter;

      for (const auto& log : log_messages) {
        const auto& level = log.level;

        spdlog::memory_buf_t formatted;
        formatter.format(log, formatted);
        std::string text = fmt::to_string(formatted);

        const auto& color = colors[static_cast<int>(level)];
        ImGui::TextColored(color, "%s", text.c_str());
      }

      if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) {
        ImGui::SetScrollHereY(1.0f);
      }
    }

    ImGui::EndChild();
    ImGui::End();
  };
}

}  // namespace guik

#endif