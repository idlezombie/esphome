#pragma once

#include <optional>
#include <span>

#include "esphome/components/modbus_controller/modbus_controller.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"

namespace esphome {
namespace actron_modbus {

class ActronModbusSwitch : public switch_::Switch, public PollingComponent {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  void write_state(bool state) override;

  void set_parent(modbus_controller::ModbusController *parent) { parent_ = parent; }
  void set_address(uint16_t address) { address_ = address; }
  void set_bitmask(uint16_t bitmask) { bitmask_ = bitmask; }
  void set_settle_timeout_ms(uint32_t timeout_ms) { settle_timeout_ms_ = timeout_ms; }
  void set_optimistic(bool optimistic) { optimistic_ = optimistic; }

 protected:
  struct FieldGuard {
    std::optional<uint16_t> expected;
    uint32_t since_ms{0};
  };

  void request_read_();
  uint16_t guarded_raw_(uint16_t raw);
  void handle_read_(std::span<const uint8_t> data, uint32_t generation);

  modbus_controller::ModbusController *parent_{nullptr};
  uint16_t address_{0};
  uint16_t bitmask_{0x0001};
  uint32_t settle_timeout_ms_{5000};
  uint32_t read_started_ms_{0};
  uint32_t read_generation_{0};
  bool optimistic_{true};
  bool read_pending_{false};
  FieldGuard guard_;
};

}  // namespace actron_modbus
}  // namespace esphome
