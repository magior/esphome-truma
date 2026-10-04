#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace esphome {
namespace truma_inetbox {

// One LIN slot as seen on the bus, recorded by the UART task for diagnostics.
struct DiagFrame {
  uint32_t t_us;        // micros() when the PID was read
  uint8_t pid_byte;     // protected identifier as on the wire
  uint8_t len;          // bytes after the PID: data + checksum, or the echo of our answer
  uint8_t data[9];
  bool ours;            // we answered this slot
  bool echo_ok;         // our bytes came back unchanged (only meaningful when ours)
  uint16_t latency_us;  // response start after the PID (only when ours)
};

enum class DiagEventKind : uint8_t { B2_ANSWERED, HEARTBEAT, DOWNLOAD, UPLOAD, ACK, HEATER_ERROR, ANSWER_DROPPED };

// Protocol event recorded by the LIN event task.
struct DiagEvent {
  uint32_t t_us;
  DiagEventKind kind;
  uint8_t a;
  uint8_t b;
  uint16_t v;
};

// Single-producer / single-consumer ring (producer: a LIN task, consumer: the ESPHome main loop).
// Never blocks the producer: when full, the item is counted as dropped. Holds N - 1 items.
template<typename T, size_t N> class SpscRing {
 public:
  bool push(const T &item) {
    const size_t head = this->head_.load(std::memory_order_relaxed);
    const size_t next = (head + 1) % N;
    if (next == this->tail_.load(std::memory_order_acquire)) {
      this->dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    this->buf_[head] = item;
    this->head_.store(next, std::memory_order_release);
    return true;
  }
  bool pop(T *item) {
    const size_t tail = this->tail_.load(std::memory_order_relaxed);
    if (tail == this->head_.load(std::memory_order_acquire))
      return false;
    *item = this->buf_[tail];
    this->tail_.store((tail + 1) % N, std::memory_order_release);
    return true;
  }
  uint32_t take_dropped() { return this->dropped_.exchange(0, std::memory_order_relaxed); }

 private:
  std::array<T, N> buf_{};
  std::atomic<size_t> head_{0};
  std::atomic<size_t> tail_{0};
  std::atomic<uint32_t> dropped_{0};
};

// Response-start latency in 100 µs bins; the last bin collects everything from 10 ms up. Main loop only.
class LatencyHistogram {
 public:
  static constexpr uint32_t BIN_US = 100;
  static constexpr size_t BINS = 101;

  void add(uint32_t us) {
    this->bins_[std::min<size_t>(us / BIN_US, BINS - 1)]++;
    if (this->count_ == 0 || us < this->min_)
      this->min_ = us;
    if (us > this->max_)
      this->max_ = us;
    this->count_++;
  }
  // Upper edge of the bin that holds the given percentile; 0 when empty.
  uint32_t percentile_us(uint8_t pct) const {
    if (this->count_ == 0)
      return 0;
    const uint32_t rank = (this->count_ * pct + 99) / 100;
    uint32_t seen = 0;
    for (size_t i = 0; i < BINS; i++) {
      seen += this->bins_[i];
      if (seen >= rank)
        return (i + 1) * BIN_US;
    }
    return BINS * BIN_US;
  }
  uint32_t count() const { return this->count_; }
  uint32_t min() const { return this->min_; }
  uint32_t max() const { return this->max_; }
  void reset() {
    this->bins_.fill(0);
    this->count_ = 0;
    this->min_ = 0;
    this->max_ = 0;
  }

 private:
  std::array<uint32_t, BINS> bins_{};
  uint32_t count_ = 0;
  uint32_t min_ = 0;
  uint32_t max_ = 0;
};

}  // namespace truma_inetbox
}  // namespace esphome
