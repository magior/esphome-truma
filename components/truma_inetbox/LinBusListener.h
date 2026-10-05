#pragma once

#include "LinBusLog.h"
#include "LinBusDiag.h"
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

#include <atomic>

#ifdef USE_ESP32
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif  // USE_ESP32

namespace esphome {
namespace truma_inetbox {

static constexpr size_t TRUMA_MSG_QUEUE_LENGTH = 6;
static constexpr size_t TRUMA_LOG_QUEUE_LENGTH = 6;

static constexpr uint8_t DIAGNOSTIC_FRAME_MASTER = 0x3c;
static constexpr uint8_t DIAGNOSTIC_FRAME_SLAVE = 0x3d;
// Experimental: seen on the Combi 4 bus only (issue #25). Forwarded regardless of master/slave classification.
static constexpr uint8_t LIN_PID_STATUS_2 = 0x22;
// Read-only: vent mode in byte 5 (issue #25). Forwarded regardless of master/slave classification.
static constexpr uint8_t LIN_PID_COMMAND_STATUS = 0x20;

enum class LIN_CHECKSUM { LIN_CHECKSUM_VERSION_1, LIN_CHECKSUM_VERSION_2 };

struct QUEUE_LIN_MSG {
  uint8_t current_PID;
  uint8_t data[8];
  uint8_t len;
};

class LinBusListener : public PollingComponent, public uart::UARTDevice {
 public:
  float get_setup_priority() const override { return setup_priority::DATA; }

  void dump_config() override;
  void setup() override;
  void update() override;

  void set_lin_checksum(LIN_CHECKSUM val) { this->lin_checksum_ = val; }
  void set_cs_pin(GPIOPin *pin) { this->cs_pin_ = pin; }
  void set_fault_pin(GPIOPin *pin) { this->fault_pin_ = pin; }
  void set_observer_mode(bool val) { this->observer_mode_.store(val, std::memory_order_relaxed); }
  bool get_observer_mode() const { return this->observer_mode_.load(std::memory_order_relaxed); }
  bool get_lin_bus_fault() { return fault_on_lin_bus_reported_ > 3; }

  void process_lin_msg_queue(TickType_t xTicksToWait);
  void process_log_queue(TickType_t xTicksToWait);

 protected:
  LIN_CHECKSUM lin_checksum_ = LIN_CHECKSUM::LIN_CHECKSUM_VERSION_2;
  GPIOPin *cs_pin_ = nullptr;
  GPIOPin *fault_pin_ = nullptr;
  // Written by the main loop, read by the UART task and the LIN event task.
  std::atomic<bool> observer_mode_{false};

  // Must only be called from uartEventTask_ — not ISR-safe, not main-loop-safe.
  void write_lin_answer_(const uint8_t *data, uint8_t len);
  bool check_for_lin_fault_();
  virtual bool answer_lin_order_(const uint8_t pid) = 0;
  virtual void lin_message_received_(const uint8_t pid, const uint8_t *message, uint8_t length) = 0;
  // Reads every available UART byte through the LIN state machine. Called by uartEventTask_ and by
  // host-side test harnesses that feed bytes without an ESP32 UART.
  void onReceive_();

  // Filled by the UART task (uartEventTask_), drained by the main loop. Copy-only: never blocks.
  SpscRing<DiagFrame, 64> diag_frames_;
  // LIN messages dropped because the LIN message queue was full. Counted by the UART task, taken by the main loop.
  std::atomic<uint32_t> lin_msg_dropped_{0};

 private:
  uint32_t diag_pid_us_ = 0;
  uint32_t diag_echo_us_ = 0;
  uint8_t diag_written_[9] = {};
  uint8_t diag_written_len_ = 0;
  bool diag_pushed_ = false;
  void diag_push_frame_();

  // Microseconds per UART Baud
  uint32_t time_per_baud_;
  // LIN break = 13 bit times minimum (spec allows 9–15)
  static constexpr uint8_t lin_break_length = 13;
  // Microseconds per LIN Break
  uint32_t time_per_lin_break_;
  static constexpr uint8_t frame_length_ = 8 /* bits */ + 1 /* Start bit */ + 2 /* Stop bits */;
  // Microseconds per UART Byte (UART Frame)
  uint32_t time_per_pid_;
  // Microseconds per UART Byte (UART Frame)
  uint32_t time_per_first_byte_;
  // Microseconds per UART Byte (UART Frame)
  uint32_t time_per_byte_;

  uint8_t fault_on_lin_bus_reported_ = 0;
  bool can_write_lin_answer_ = false;

  enum read_state {
    READ_STATE_BREAK,
    READ_STATE_SYNC,
    READ_STATE_SID,
    READ_STATE_DATA,
    READ_STATE_ACT,
  };
  read_state current_state_ = READ_STATE_BREAK;
  uint8_t current_PID_with_parity_ = 0x00;
  uint8_t current_PID_ = 0x00;
  bool current_PID_order_answered_ = false;
  bool current_data_valid = true;
  uint8_t current_data_count_ = 0;
  // up to 8 byte data frame + CRC
  uint8_t current_data_[9] = {};
  uint32_t last_data_received_ = 0;

  void current_state_reset_() {
    this->current_state_ = READ_STATE_BREAK;
    this->current_PID_with_parity_ = 0x00;
    this->current_PID_ = 0x00;
    this->current_PID_order_answered_ = false;
    this->current_data_valid = true;
    this->current_data_count_ = 0;
    memset(this->current_data_, 0, sizeof(this->current_data_));
    this->diag_pushed_ = false;
    this->diag_written_len_ = 0;
  };
  void read_lin_frame_();
  void clear_uart_buffer_();
  void setup_framework();

  // Declaration order matters: storage → static-queue-struct → handle.
  // xQueueCreateStatic initialises the storage internally; BSS zero-init is sufficient.
  uint8_t lin_msg_static_queue_storage[TRUMA_MSG_QUEUE_LENGTH * sizeof(QUEUE_LIN_MSG)];
  StaticQueue_t lin_msg_static_queue_;
  QueueHandle_t lin_msg_queue_ =
      xQueueCreateStatic(/* uxQueueLength */ TRUMA_MSG_QUEUE_LENGTH,
                         /* uxItemSize */ sizeof(QUEUE_LIN_MSG),
                         /* pucQueueStorageBuffer */ lin_msg_static_queue_storage, &lin_msg_static_queue_);

#if ESPHOME_LOG_LEVEL > ESPHOME_LOG_LEVEL_NONE
  uint8_t log_static_queue_storage[TRUMA_LOG_QUEUE_LENGTH * sizeof(QUEUE_LOG_MSG)];
  StaticQueue_t log_static_queue_;
  QueueHandle_t log_queue_ =
      xQueueCreateStatic(/* uxQueueLength */ TRUMA_LOG_QUEUE_LENGTH,
                         /* uxItemSize */ sizeof(QUEUE_LOG_MSG),
                         /* pucQueueStorageBuffer */ log_static_queue_storage, &log_static_queue_);
#endif

#ifdef USE_ESP32
  TaskHandle_t eventTaskHandle_;
  static void eventTask_(void *args);
#endif  // USE_ESP32
#ifdef USE_ESP32_FRAMEWORK_ESP_IDF
  TaskHandle_t uartEventTaskHandle_;
  static void uartEventTask_(void *args);
#endif  // USE_ESP32_FRAMEWORK_ESP_IDF
};

}  // namespace truma_inetbox
}  // namespace esphome