#ifndef GUIK_PROGRESS_MODAL_HPP
#define GUIK_PROGRESS_MODAL_HPP

#include <any>
#include <mutex>
#include <atomic>
#include <thread>
#include <string>
#include <optional>
#include <iostream>
#include <functional>

#include <imgui.h>

#include <guik/progress_interface.hpp>

namespace guik {

class ProgressModal : public ProgressInterface {
public:
  ProgressModal(const std::string& modal_name) : modal_name(modal_name), running(false), max(0), current(0) {}
  virtual ~ProgressModal() override {
    if(thread.joinable()) {
      thread.join();
    }
  }

  virtual void set_title(const std::string& title) override {
    std::lock_guard<std::mutex> lock(mutex);
    this->title = title;
  }

  virtual void set_text(const std::string& text) override {
    std::lock_guard<std::mutex> lock(mutex);
    this->text = text;
  }

  virtual void set_maximum(int max) override {
    this->max = max;
    this->current = 0;
  }
  virtual void set_current(int current) override {
    this->current = current;
  }
  virtual void increment() override {
    current++;
  }

  template<typename T>
  void open(const std::string& task_name, const std::function<T(ProgressInterface& progress)>& task) {
    this->task_name = task_name;
    this->title.clear();
    this->text.clear();

    ImGui::OpenPopup(modal_name.c_str());
    result_ = nullptr;
    running = true;
    current = 0;

    thread = std::thread([this, task]() {
      result_ = task(*this);
      running = false;
    });
  }

  template<typename T>
  std::optional<T> run(const std::string& task_name) {
    if(task_name != this->task_name) {
      return std::nullopt;
    }

    bool terminated = false;
    if(ImGui::BeginPopupModal(modal_name.c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar)) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        if(!title.empty()) {
          ImGui::Text("%s", title.c_str());
        }

        ImGui::Text("%c %s", "|/-\\"[(int)(ImGui::GetTime() / 0.05f) & 3], text.c_str());
      }

      float fraction = current / static_cast<float>(max);
      ImGui::ProgressBar(fraction, ImVec2(128, 16));

      if(!running) {
        thread.join();
        ImGui::CloseCurrentPopup();
        terminated = true;
      }
      ImGui::EndPopup();
    }

    if(!result_.has_value() || !terminated) {
      return std::nullopt;
    }

    T ret = std::any_cast<T>(result_);
    result_ = nullptr;
    return ret;
  }

  bool is_running() const {
    return running;
  }

private:
  std::mutex mutex;
  std::string modal_name;
  std::string title;
  std::string text;

  std::atomic_bool running;
  std::atomic_int max;
  std::atomic_int current;

  std::string task_name;

  std::thread thread;
  std::any result_;
};

}  // namespace guik

#endif