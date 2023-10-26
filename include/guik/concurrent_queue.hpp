#ifndef GUIK_CONCURRENT_QUEUE_HPP
#define GUIK_CONCURRENT_QUEUE_HPP

#include <deque>
#include <mutex>

namespace guik {

template <typename T>
class ConcurrentQueue {
public:
  virtual ~ConcurrentQueue() {}

  virtual size_t size() const = 0;
  virtual void push(const T& item) = 0;
  virtual std::vector<T> try_pop(int max_num_items) = 0;
};

template <typename T>
class StdConcurrentQueue : public ConcurrentQueue<T> {
public:
  StdConcurrentQueue() {}
  virtual ~StdConcurrentQueue() override {}

  virtual size_t size() const override {
    std::lock_guard<std::mutex> lock(mutex);
    return queue.size();
  }

  virtual void push(const T& item) override {
    std::lock_guard<std::mutex> lock(mutex);
    queue.push_back(item);
  }

  virtual std::vector<T> try_pop(int max_num_items) override {
    std::lock_guard<std::mutex> lock(mutex);

    if (queue.empty()) {
      return {};
    }

    if (queue.size() <= max_num_items) {
      std::vector<T> items(queue.begin(), queue.end());
      queue.clear();
      return items;
    }

    std::vector<T> items(queue.begin(), queue.begin() + max_num_items);
    queue.erase(queue.begin(), queue.begin() + max_num_items);
    return items;
  }

private:
  mutable std::mutex mutex;
  std::deque<T> queue;
};

}  // namespace guik

#endif