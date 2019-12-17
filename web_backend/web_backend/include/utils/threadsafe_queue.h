// Copyright 2019 Horizon Robotics
// Created by peizhe.chen on 3/1/2019
// A simple thread safe queue

#ifndef INCLUDE_UTILS_THREADSAFE_QUEUE_HPP_
#define INCLUDE_UTILS_THREADSAFE_QUEUE_HPP_

#include <mutex>
#include <vector>
#include <thread>
#include <condition_variable>

namespace AdUtil {
template <typename T>
class ThreadSafeQueue {
 public:
  ThreadSafeQueue() = default;

  ThreadSafeQueue(const ThreadSafeQueue& other) {
    std::lock_guard<std::mutex> lk(other.mut_);
    data_queue_ = other.data_queue_;
  }

  void Push(const T& new_value) {
    auto data = std::make_shared<T>(std::move(new_value));
    std::lock_guard<std::mutex> lk(mut_);
    data_queue_.push_back(data);
    data_cond_.notify_one();
  }

  void Push(const std::vector<T>& new_value_list) {
    std::lock_guard<std::mutex> lk(mut_);
    for(auto new_value:new_value_list)
    {
      auto data = std::make_shared<T>(std::move(new_value));
      data_queue_.push_back(data);  
    }
    data_cond_.notify_one();
  }

  // void WaitAndPop(T& value) {
  //   std::unique_lock<std::mutex> lk(mut_);
  //   data_cond_.wait(lk, [this] { return !data_queue_.empty(); });
  //   value = std::move(*data_queue_.front());
  //   data_queue_.pop();
  // }

  // std::shared_ptr<T> WaitAndPop() {
  //   std::unique_lock<std::mutex> lk(mut_);
  //   data_cond_.wait(lk, [this] { return !data_queue_.empty(); });
  //   auto res = data_queue_.front();
  //   data_queue_.pop();
  //   return res;
  // }

  // bool TryPop(T& value) {
  //   std::lock_guard<std::mutex> lk(mut_);
  //   if (data_queue_.empty()) {
  //     return false;
  //   }
  //   value = std::move(*data_queue_.front());
  //   data_queue_.pop();
  //   return true;
  // }

  // std::shared_ptr<T> TryPop() {
  //   std::lock_guard<std::mutex> lk(mut_);
  //   if (data_queue_.empty()) {
  //     return std::shared_ptr<T>();
  //   }
  //   auto res = data_queue_.front();
  //   data_queue_.pop();
  //   return res;
  // }

  std::shared_ptr<T> WaitGetFront() {
    std::unique_lock<std::mutex> lk(mut_);
    data_cond_.wait(lk, [this] { return !data_queue_.empty(); });
    auto res = data_queue_.front();
    return res;
  }

  std::vector<std::shared_ptr<T>> WaitGetAll() {
    std::unique_lock<std::mutex> lk(mut_);
    data_cond_.wait(lk, [this] { return !data_queue_.empty(); });
    auto res = data_queue_;
    return res;
  }

  bool IsEmpty() const {
    std::lock_guard<std::mutex> lk(mut_);
    return data_queue_.empty();
  }

  void Clear() {
    std::lock_guard<std::mutex> lk(mut_);
    data_queue_ = std::vector<std::shared_ptr<T>>();
  }

  size_t Size() const {
    std::lock_guard<std::mutex> lk(mut_);
    return data_queue_.size();
  }


 private:
  mutable std::mutex mut_;
  std::vector<std::shared_ptr<T>> data_queue_;
  std::condition_variable data_cond_;
};

}  // namespace AdUtil

#endif  //  INCLUDE_UTILS_THREADSAFE_QUEUE_HPP_
