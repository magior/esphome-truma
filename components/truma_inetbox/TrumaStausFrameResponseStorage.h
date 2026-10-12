#pragma once

#include <atomic>
#include <mutex>

#include "TrumaStausFrameStorage.h"
#include "TrumaStructs.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace truma_inetbox {

class TrumaiNetBoxApp;

// Ownership: `update_status_` belongs to the main loop (actions, `update_prepare`). `update_submit` copies it
// into `submitted_` under `lock_`; the LIN eventTask_ builds the upload from `submitted_` under `lock_` in
// `create_update_data`. The flags stay atomic because the UART task reads `has_update()` without the lock.
template<typename T, typename TResponse>
class TrumaStausFrameResponseStorage : public TrumaStausFrameStorage<T>, public Parented<TrumaiNetBoxApp> {
 public:
  void reset() override {
    TrumaStausFrameStorage<T>::reset();
    std::lock_guard<std::mutex> guard(this->lock_);
    this->update_status_prepared_.store(false);
    this->update_status_unsubmitted_.store(false);
    this->update_status_stale_.store(false);
    this->stale_clear_pending_ = false;
  }
  virtual bool can_update() { return this->data_valid_.load(); }
  virtual TResponse *update_prepare() = 0;
  // Main loop: hands the prepared update to the event task.
  void update_submit() {
    std::lock_guard<std::mutex> guard(this->lock_);
    this->submitted_ = this->update_status_;
    this->update_status_unsubmitted_.store(true);
  }
  bool has_update() const { return this->update_status_unsubmitted_.load(); }
  // LIN event task.
  virtual void create_update_data(StatusFrame *response, uint8_t *response_len, uint8_t command_counter) = 0;

 protected:
  // LIN event task, with `lock_` held.
  inline void update_submitted() {
    this->update_status_prepared_.store(false);
    this->update_status_unsubmitted_.store(false);
    this->update_status_stale_.store(true);
    // A status received before this upload does not end the stale state.
    this->stale_clear_pending_ = false;
  }
  void on_status_received_() override { this->stale_clear_pending_ = true; }
  // The main loop adopts the first status received after the upload: `update_prepare` starts from it again.
  void on_status_adopted_() override {
    if (this->stale_clear_pending_) {
      this->stale_clear_pending_ = false;
      this->update_status_stale_.store(false);
    }
  }

  // Prepared means `update_status_` was copied from `data_`.
  std::atomic<bool> update_status_prepared_{false};
  // Prepared means an update is already awaiting fetch from CP plus.
  std::atomic<bool> update_status_unsubmitted_{false};
  // I have submitted my update request to CP plus, but I have not received an update with new heater values from CP
  // plus.
  std::atomic<bool> update_status_stale_{false};
  // Main loop only.
  TResponse update_status_{};
  // Guarded by `lock_`: the last submitted update and whether a status arrived after the last upload.
  TResponse submitted_{};
  bool stale_clear_pending_{false};
};

}  // namespace truma_inetbox
}  // namespace esphome
