#include "TrumaCpPlusBinarySensor.h"
#include "esphome/core/log.h"
#include "esphome/components/truma_inetbox/helpers.h"

namespace esphome {
namespace truma_inetbox {

static const char *const TAG = "truma_inetbox.cpplus_binary_sensor";

// 60 seconds without a valid frame on the bus = disconnected. The CP Plus runs a burst about every 15 s even
// with the panel asleep, but addresses the box only when it has something for it (C3: 27-102 s apart at rest).
static constexpr uint32_t CP_PLUS_TIMEOUT_US = 60 * 1000 * 1000;

void TrumaCpPlusBinarySensor::update() {
  const uint32_t last_frame = this->parent_->get_last_valid_frame();
  // A frame that already timed out keeps the sensor off until a newer one arrives; otherwise the
  // subtraction below would report "connected" for 60 s every time micros() wraps around (~71 min).
  if (this->parent_->get_lin_bus_fault() || last_frame == 0 || last_frame == this->timed_out_frame_) {
    this->publish_state(false);
    return;
  }
  // Subtraction idiom is wraparound-safe for the 32-bit micros() counter.
  const uint32_t since_last_frame = micros() - last_frame;
  if (since_last_frame >= CP_PLUS_TIMEOUT_US) {
    this->timed_out_frame_ = last_frame;
  }
  this->publish_state(since_last_frame < CP_PLUS_TIMEOUT_US);
}

void TrumaCpPlusBinarySensor::dump_config() { LOG_BINARY_SENSOR("", "Truma CP Plus Binary Sensor", this); }
}  // namespace truma_inetbox
}  // namespace esphome