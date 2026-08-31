#include "actron_modbus_switch.h"

#include "esphome/core/log.h"

namespace esphome {
namespace actron_modbus {

static const char *const TAG = "actron_modbus.switch";

static uint16_t parse_u16(std::span<const uint8_t> data) {
  if (data.size() < 2) {
    return 0;
  }
  return (uint16_t(data[0]) << 8) | uint16_t(data[1]);
}

void ActronModbusSwitch::setup() {
  // Wait for the first successful poll; don't force a boot write.
}

void ActronModbusSwitch::dump_config() {
  ESP_LOGCONFIG(TAG, "Actron Modbus Switch '%s':", this->get_name().c_str());
  ESP_LOGCONFIG(TAG, "  Address: %u", this->address_);
  ESP_LOGCONFIG(TAG, "  Bitmask: 0x%04X", this->bitmask_);
  ESP_LOGCONFIG(TAG, "  Optimistic: %s", YESNO(this->optimistic_));
  ESP_LOGCONFIG(TAG, "  Settle timeout: %ums", (unsigned) this->settle_timeout_ms_);
}

void ActronModbusSwitch::write_state(bool state) {
  if (this->parent_ == nullptr) {
    return;
  }

  uint16_t value = state ? this->bitmask_ : 0;
  if (this->optimistic_) {
    this->publish_state(state);
    this->guard_.expected = value;
    this->guard_.since_ms = millis();
  }

  this->parent_->queue_command(
      modbus_controller::ModbusCommandItem::create_write_single_command(this->parent_, this->address_, value));
}

void ActronModbusSwitch::update() {
  if (this->parent_ == nullptr) {
    return;
  }

  if (this->read_pending_) {
    if ((millis() - this->read_started_ms_) < this->settle_timeout_ms_) {
      return;
    }
    ESP_LOGD(TAG, "'%s' read timed out", this->get_name().c_str());
    this->read_pending_ = false;
    this->read_generation_++;
  }

  this->request_read_();
}

void ActronModbusSwitch::request_read_() {
  using modbus_controller::ModbusCommandItem;
  using modbus::EntityType;

  uint32_t generation = ++this->read_generation_;
  this->read_pending_ = true;
  this->read_started_ms_ = millis();

  this->parent_->queue_command(ModbusCommandItem::create_read_command(
      this->parent_, EntityType::HOLDING, this->address_, 1,
      [this, generation](EntityType, uint16_t, std::span<const uint8_t> data) {
        this->handle_read_(data, generation);
      }));
}

uint16_t ActronModbusSwitch::guarded_raw_(uint16_t raw) {
  if (!this->optimistic_ || !this->guard_.expected.has_value()) {
    return raw;
  }

  if (raw == *this->guard_.expected) {
    this->guard_.expected.reset();
    return raw;
  }

  if ((millis() - this->guard_.since_ms) < this->settle_timeout_ms_) {
    return *this->guard_.expected;
  }

  this->guard_.expected.reset();
  return raw;
}

void ActronModbusSwitch::handle_read_(std::span<const uint8_t> data, uint32_t generation) {
  if (generation != this->read_generation_) {
    return;
  }
  this->read_pending_ = false;

  uint16_t raw = parse_u16(data) & this->bitmask_;
  uint16_t value = this->guarded_raw_(raw);
  this->publish_state(value != 0);
}

}  // namespace actron_modbus
}  // namespace esphome
