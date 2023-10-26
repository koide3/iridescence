#ifndef GUIK_TBB_CONCURRENT_QUEUE_HPP
#define GUIK_TBB_CONCURRENT_QUEUE_HPP

#include <tbb/concurrent_queue.h>
#include <guik/concurrent_queue.hpp>

namespace guik {

template <typename T>
class TbbConcurrentQueue : public ConcurrentQueue<T> {
public:
  TbbConcurrentQueue() {}
  virtual ~TbbConcurrentQueue() override {}

  virtual size_t size() const override {
    return queue.unsafe_size();
  }

  virtual void push(const T& item) override {
    queue.push(item);
  }

  virtual std::vector<T> try_pop(int max_num_items) override {
    std::vector<T> items;
    items.reserve(max_num_items);

    for (int i = 0; i < max_num_items; i++) {
      T item;
      if (queue.try_pop(item)) {
        items.emplace_back(item);
      }
    }
    return items;
  }

private:
  tbb::concurrent_queue<T> queue;
};

}  // namespace guik

#endif