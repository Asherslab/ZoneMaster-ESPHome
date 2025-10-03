#pragma once
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include <vector>
#include <algorithm>

namespace esphome {
namespace zonemaster_proto {

static const char *const TAG = "zonemaster_proto";

class ZonemasterProto : public Component, public uart::UARTDevice {
 public:
  ZonemasterProto() = default;

  // Config
  void set_device_id(uint8_t id) { this->device_id_ = id; }
  void set_poll_interval(uint32_t ms) { this->poll_interval_ms_ = ms; }

  // Sensors
  void set_button1(binary_sensor::BinarySensor *s) { this->btn_[0] = s; }
  void set_button2(binary_sensor::BinarySensor *s) { this->btn_[1] = s; }
  void set_button3(binary_sensor::BinarySensor *s) { this->btn_[2] = s; }
  void set_button4(binary_sensor::BinarySensor *s) { this->btn_[3] = s; }
  void set_button5(binary_sensor::BinarySensor *s) { this->btn_[4] = s; }
  void set_button6(binary_sensor::BinarySensor *s) { this->btn_[5] = s; }

  void setup() override {
    ESP_LOGI(TAG, "ZonemasterProto setup: device_id=0x%02X, poll_interval=%u ms", this->device_id_, this->poll_interval_ms_);
    this->last_poll_ms_ = 0;
  }

  void loop() override {
    // Read incoming UART bytes into buffer
    while (this->available()) {
      uint8_t b;
      if (!this->read_byte(&b)) break;
      buf_.push_back(b);
      if (buf_.size() > 512) buf_.erase(buf_.begin(), buf_.end() - 256);
      this->extract_and_process_frames_();
    }

    // Optional periodic polling
    // const uint32_t now = millis();
    // if (this->poll_interval_ms_ > 0 && now - this->last_poll_ms_ >= this->poll_interval_ms_) {
    //   this->send_request_(this->device_id_);
    //   this->last_poll_ms_ = now;
    // }
  }

 protected:
  // ==== CRC-8 Dallas/Maxim helpers ====
  static inline uint8_t reflect8_(uint8_t x) {
    x = (uint8_t)((x & 0xF0u) >> 4) | (uint8_t)((x & 0x0Fu) << 4);
    x = (uint8_t)((x & 0xCCu) >> 2) | (uint8_t)((x & 0x33u) << 2);
    x = (uint8_t)((x & 0xAAu) >> 1) | (uint8_t)((x & 0x55u) << 1);
    return x;
  }

  static uint8_t crc8_maxim_(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
      uint8_t b = reflect8_(data[i]);  // RefIn
      crc ^= b;
      for (int k = 0; k < 8; k++) {
        if (crc & 0x80) crc = uint8_t((crc << 1) ^ 0x31);
        else            crc <<= 1;
      }
    }
    crc = reflect8_(crc);              // RefOut
    return crc;                        // xorout=0x00
  }

  void send_request_(uint8_t id) {
    // Request format: AA 00 30 <ID> 01 00 00 <CRC> 55
    std::vector<uint8_t> msg {0xAA, 0x00, 0x30, id, 0x01, 0x00, 0x00};
    uint8_t crc = crc8_maxim_(&msg[1], msg.size() - 1);  // exclude leading AA
    msg.push_back(crc);
    msg.push_back(0x55);
    this->write_array(msg);
    ESP_LOGV(TAG, "TX %uB: AA 00 30 %02X 01 00 00 %02X 55", (unsigned) msg.size(), id, crc);
  }

  void extract_and_process_frames_() {
    // find start 0xAA, end 0x55
    for (;;) {
      auto it_start = std::find(buf_.begin(), buf_.end(), 0xAA);
      if (it_start == buf_.end()) { buf_.clear(); return; }
      auto it_end = std::find(it_start + 1, buf_.end(), 0x55);
      if (it_end == buf_.end()) {
        if (it_start != buf_.begin()) buf_.erase(buf_.begin(), it_start);
        return; // wait for more
      }
      std::vector<uint8_t> fr(it_start, it_end + 1);
      // consume up to end
      buf_.erase(buf_.begin(), it_end + 1);

      if (fr.size() < 9) continue;

      // Identify response: AA 30 00 ID 81 01 DATA CRC 55
      const bool looks_response = (fr[0] == 0xAA && fr[1] == 0x30 && fr[2] == 0x00 && fr.back() == 0x55);
      const bool looks_request  = (fr[0] == 0xAA && fr[1] == 0x00 && fr[2] == 0x30 && fr.back() == 0x55);

      if (!(looks_response || looks_request)) {
        ESP_LOGV(TAG, "Skipped frame (unknown sig): len=%u", (unsigned)fr.size());
        continue;
      }

      // Compute CRC over bytes AFTER AA and BEFORE CRC (exclude last two)
      const uint8_t crc_rx = fr[fr.size() - 2];
      const uint8_t *p = &fr[1];
      const size_t n = fr.size() - 3;
      const uint8_t crc_calc = crc8_maxim_(p, n);

      if (crc_rx != crc_calc) {
        ESP_LOGW(TAG, "CRC mismatch (exp=%02X got=%02X), dropping", crc_calc, crc_rx);
        continue;
      }

      if (looks_response) {
        if (fr.size() < 9) continue;
        const uint8_t id   = fr[3];
        const uint8_t data = fr[6];
        if (id != this->device_id_) {
          // If you want to accept any device, comment this out.
          ESP_LOGV(TAG, "Response for other ID 0x%02X (=0x%02X), ignoring", id, this->device_id_);
          continue;
        }
        ESP_LOGV(TAG, "RX: %02X %02X %02X %02X %02X %02X %02X %02X %02X", fr[0], fr[1], fr[2], fr[3], fr[4], fr[5], fr[6], fr[7], fr[8]);
        this->publish_buttons_(data);
      } else {
        // Valid request seen on bus (optional to log/debug)
        ESP_LOGV(TAG, "REQ OK len=%u", (unsigned)fr.size());
      }
    }
  }

  void publish_buttons_(uint8_t data) {
    // Bits: b0..b5 => buttons 1..6
    for (int i = 0; i < 6; i++) {
      if (btn_[i] != nullptr) {
        const bool on = (data >> i) & 0x01;
        btn_[i]->publish_state(on);
      }
    }
  }

  // State
  std::vector<uint8_t> buf_;
  binary_sensor::BinarySensor *btn_[6] {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
  uint8_t  device_id_{0xB1};
  uint32_t poll_interval_ms_{500};
  uint32_t last_poll_ms_{0};
};

}  // namespace zonemaster_proto
}  // namespace esphome
