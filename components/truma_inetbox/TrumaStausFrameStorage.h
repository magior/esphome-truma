#pragma once

#include <atomic>
#include <mutex>

#include "esphome/core/automation.h"

namespace esphome {
namespace truma_inetbox {

// Storage for a Truma status frame (e.g. heater, aircon, clock).
//
// Ownership: `data_` belongs to the main loop. The LIN eventTask_ hands a downloaded status over with
// `set_status`, which writes `incoming_` under `lock_`; the main loop adopts it into `data_` in `update()`
// before the callbacks run. Event task code that needs the last status reads `incoming_` under `lock_`.
// The lock is taken only by the event task and the main loop, never by the UART task: the flags the
// UART task reads are atomic.
template<typename T> class TrumaStausFrameStorage {
 public:
  // Main loop only.
  bool get_status_valid() { return this->data_valid_.load(); };
  const T *get_status() { return &this->data_; };
  // LIN event task.
  virtual void set_status(T val) {
    std::lock_guard<std::mutex> guard(this->lock_);
    this->incoming_ = val;
    this->data_updated_.store(true);
    this->on_status_received_();
  };
  // Main loop.
  void update() {
    {
      std::lock_guard<std::mutex> guard(this->lock_);
      if (!this->data_updated_.exchange(false)) {
        return;
      }
      this->data_ = this->incoming_;
      this->data_valid_.store(true);
      this->on_status_adopted_();
    }
    this->dump_data();
    this->state_callback_.call(&this->data_);
  };
  virtual void reset() {
    std::lock_guard<std::mutex> guard(this->lock_);
    this->data_valid_.store(false);
    this->data_updated_.store(false);
  };
  void add_on_message_callback(std::function<void(const T *)> callback) {
    this->state_callback_.add(std::move(callback));
  };
  virtual void dump_data() const = 0;

 protected:
  // Called with `lock_` held: a status was handed over (event task) / adopted into `data_` (main loop).
  virtual void on_status_received_() {}
  virtual void on_status_adopted_() {}

  CallbackManager<void(const T *)> state_callback_{};
  std::mutex lock_;
  // Main loop only.
  T data_{};
  // Last status from the event task, guarded by `lock_`.
  T incoming_{};
  // `data_` holds an adopted status (main loop view).
  std::atomic<bool> data_valid_{false};
  // `incoming_` holds a status the main loop has not adopted yet.
  std::atomic<bool> data_updated_{false};
};

}  // namespace truma_inetbox
}  // namespace esphome
