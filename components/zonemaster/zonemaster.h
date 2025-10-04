#pragma once
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include <vector>
#include <algorithm>

namespace esphome {
namespace zonemaster {

static const char *const TAG = "zonemaster";

class Zonemaster : public Component, public uart::UARTDevice {
 public:
  Zonemaster() = default;

  // Config
  void set_accept_any_response(bool v) { this->accept_any_response_ = v; }
  void set_response_window(uint32_t ms) { this->response_window_ms_ = ms; }

  // Sensors
  void set_button1(binary_sensor::BinarySensor *s) { this->btn_[0] = s; }
  void set_button2(binary_sensor::BinarySensor *s) { this->btn_[1] = s; }
  void set_button3(binary_sensor::BinarySensor *s) { this->btn_[2] = s; }
  void set_button4(binary_sensor::BinarySensor *s) { this->btn_[3] = s; }
  void set_button5(binary_sensor::BinarySensor *s) { this->btn_[4] = s; }
  void set_button6(binary_sensor::BinarySensor *s) { this->btn_[5] = s; }

  void setup() override {
    ESP_LOGI(TAG, "RX-only; accept_any_response=%s, response_window=%ums",
             accept_any_response_ ? "true" : "false",
             response_window_ms_);
  }

  void loop() override {
    // Read incoming UART bytes into buffer
    while (this->available()) {
      uint8_t b;
      if (!this->read_byte(&b)) break;
      buf_.push_back(b);
      if (buf_.size() > 1024) buf_.erase(buf_.begin(), buf_.end() - 512);
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

  void send_button_state(uint8_t id, bool b1, bool b2, bool b3, bool b4, bool b5, bool b6) {
   uint8_t data = 0;
   if (b1) data |= 0x01;
   if (b2) data |= 0x02;
   if (b3) data |= 0x04;
   if (b4) data |= 0x08;
   if (b5) data |= 0x10;
   if (b6) data |= 0x20;
 
   std::vector<uint8_t> msg = {0xAA, 0x00, 0x30, id, 0x01, 0x00, data};
   uint8_t crc = crc8_maxim_(&msg[1], msg.size() - 1); // exclude AA
   msg.push_back(crc);
   msg.push_back(0x55);
 
   this->write_array(msg);
   ESP_LOGI(TAG, "Sent button state: ID=0x%02X DATA=0x%02X CRC=0x%02X", id, data, crc);
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

      //ESP_LOGI(TAG, "Size: %X", fr.size());
      if (fr.size() < 9) continue;

      // Identify response: AA 30 00 ID 81 01 DATA CRC 55
      const bool is_rsp = (fr[0] == 0xAA && fr[1] != 0x00 && fr[2] == 0x00 && fr.back() == 0x55);
      const bool is_req  = (fr[0] == 0xAA && fr[1] == 0x00 && fr[2] != 0x00 && fr.back() == 0x55);

      ESP_LOGV(TAG, "TEST: %02X %02X %02X %02X %02X %02X %02X %02X %02X [%d, %d]", fr[0], fr[1], fr[2], fr[3], fr[4], fr[5], fr[6], fr[7], fr[8], is_req, is_rsp);
     
      if (!(is_rsp || is_req)) {
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

      if (is_req)
      {
        ESP_LOGI(TAG, "TX: %02X %02X %02X %02X %02X %02X %02X %02X %02X", fr[0], fr[1], fr[2], fr[3], fr[4], fr[5], fr[6], fr[7], fr[8]);
      }
      if (is_rsp)
      {
        ESP_LOGI(TAG, "RX: %02X %02X %02X %02X %02X %02X %02X %02X %02X", fr[0], fr[1], fr[2], fr[3], fr[4], fr[5], fr[6], fr[7], fr[8]);
      }
     
      if (is_req) {
        // Request format: AA 00 30 ID 01 00 00 CRC 55
        const uint8_t id = fr[3];
        last_req_id_   = id;
        last_req_ms_   = millis();
        ESP_LOGV(TAG, "REQ OK id=0x%02X", id);
      } else if (is_rsp) {
        // Response format: AA 30 00 ID 81 01 DATA CRC 55
        const uint8_t id   = fr[3];
        const uint8_t data = fr[6];

        if (accept_any_response_) {
          publish_buttons_(data);
          ESP_LOGV(TAG, "RSP OK (any) id=0x%02X data=0x%02X", id, data);
        } else {
          const uint32_t now = millis();
          const bool id_matches   = (last_req_id_.has_value() && id == last_req_id_.value());
          const bool within_window = (last_req_id_.has_value() && (now - last_req_ms_) <= response_window_ms_);
          if (id_matches && within_window) {
            publish_buttons_(data);
            ESP_LOGV(TAG, "RSP OK matched id=0x%02X data=0x%02X", id, data);
          } else {
            ESP_LOGV(TAG, "RSP ignored id=0x%02X (last_req=%s, window=%s)",
                     id,
                     last_req_id_.has_value() ? "set" : "unset",
                     last_req_id_.has_value() ? ((now - last_req_ms_) <= response_window_ms_ ? "OK" : "expired") : "n/a");
          }
        }
      } else {
        ESP_LOGV(TAG, "Unknown frame sig, len=%u", (unsigned) fr.size());
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

  // Request tracking (rolling ID observed on the bus)
  optional<uint8_t> last_req_id_{};
  uint32_t last_req_ms_{0};

  // Config flags
  bool     accept_any_response_{false};
  uint32_t response_window_ms_{150};
};

}  // namespace zonemaster
}  // namespace esphome
