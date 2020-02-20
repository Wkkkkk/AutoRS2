//
// Created by zhihui on 12/23/19.
//

#ifndef MULTITHREAD_BLOCKING_QUEUE_H
#define MULTITHREAD_BLOCKING_QUEUE_H

#include <deque>
#include <memory>
#include <mutex>
#include <chrono>
#include <condition_variable>

namespace core {

// A thread-safe blocking queue that is useful for producer/consumer patterns
// T must be movable
    template<typename T>
    class BlockingQueue {
    public:
        static constexpr size_t kInfiniteQueueSize = 0;

        BlockingQueue() : BlockingQueue(kInfiniteQueueSize) {}

        BlockingQueue(const BlockingQueue &) = delete;

        BlockingQueue &operator=(const BlockingQueue &) = delete;

        explicit BlockingQueue(const size_t queue_size) : max_queue_size_(queue_size) {}

        void Push(T t) {
            std::unique_lock <std::mutex> lk(mutex_);
            cv_.wait(lk, [this] { return QueueNotFullCondition(); });

            deque_.push_back(std::move(t));
            cv_.notify_one();
        }

        bool PushWithTimeout(T t, const std::chrono::microseconds timeout) {
            std::unique_lock <std::mutex> lk(mutex_);
            if (!cv_.wait_for(lk, timeout, [this] { return QueueNotFullCondition(); })) {
                return false;
            }

            deque_.push_back(std::move(t));
            return true;
        }

        T Pop() {
            std::unique_lock <std::mutex> lk(mutex_);
            cv_.wait(lk, [this] { return !QueueEmptyCondition(); });

            T t = std::move(deque_.front());
            deque_.pop_front();
            return t;
        }

        T PopWithTimeout(const std::chrono::microseconds timeout) {
            std::unique_lock <std::mutex> lk(mutex_);
            if (!cv_.wait_for(lk, timeout, [this] { return !QueueEmptyCondition(); })) {
                return nullptr;
            }

            T t = std::move(deque_.front());
            deque_.pop_front();
            return t;
        }

        void Clear() {
            std::deque <T> new_deque;

            std::unique_lock <std::mutex> lk(mutex_);
            std::swap(new_deque, deque_);
        }

        size_t Size() const {
            std::unique_lock <std::mutex> lk(mutex_);
            return deque_.size();
        }

        bool Empty() const {
            return Size() == 0;
        }

    private:
        bool QueueEmptyCondition() {
            return deque_.empty();
        }

        bool QueueNotFullCondition() {
            return max_queue_size_ == kInfiniteQueueSize || deque_.size() < max_queue_size_;
        }

        const size_t max_queue_size_;
        mutable std::mutex mutex_;
        std::deque <T> deque_;
        std::condition_variable cv_;
    };
}

#endif //MULTITHREAD_BLOCKING_QUEUE_H
