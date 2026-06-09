#pragma once

#include <optional>
#include <vector>

#include "esphome/components/climate/climate.h"
#include "esphome/components/modbus_controller/modbus_controller.h"
#include "esphome/core/component.h"

namespace esphome {
namespace actron_modbus {

class ActronModbusClimate : public climate::Climate, public PollingComponent {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

  void set_parent(modbus_controller::ModbusController *parent) { parent_ = parent; }
  void set_power_register(uint16_t reg) { power_register_ = reg; }
  void set_fan_register(uint16_t reg) { fan_register_ = reg; }
  void set_mode_register(uint16_t reg) { mode_register_ = reg; }
  void set_setpoint_register(uint16_t reg) { setpoint_register_ = reg; }
  void set_room_temp_register(uint16_t reg) { room_temp_register_ = reg; }
  void set_command_interval_ms(uint32_t interval_ms) { command_interval_ms_ = interval_ms; }
  void set_settle_timeout_ms(uint32_t timeout_ms) { settle_timeout_ms_ = timeout_ms; }
  void set_optimistic(bool optimistic) { optimistic_ = optimistic; }

 protected:
  enum class PendingType : uint8_t { POWER, MODE, SETPOINT, FAN };
  struct PendingCommand {
    PendingType type;
    uint16_t reg;
    uint16_t value;
  };
  struct FieldGuard {
    std::optional<uint16_t> expected;
    uint32_t since_ms{0};
  };

  void request_reads_();
  void dispatch_next_write_();
  void queue_or_replace_(PendingType type, uint16_t reg, uint16_t value);
  bool has_pending_() const { return !pending_.empty(); }
  void mark_expected_(PendingType type, uint16_t value);
  uint16_t guarded_raw_(FieldGuard &guard, uint16_t raw);
  void publish_and_save_();
  void finish_read_(uint32_t generation);

  void handle_power_read_(const std::vector<uint8_t> &data, uint32_t generation);
  void handle_fan_read_(const std::vector<uint8_t> &data, uint32_t generation);
  void handle_mode_read_(const std::vector<uint8_t> &data, uint32_t generation);
  void handle_setpoint_read_(const std::vector<uint8_t> &data, uint32_t generation);
  void handle_room_temp_read_(const std::vector<uint8_t> &data, uint32_t generation);

  bool available_{true};
  bool optimistic_{true};

  modbus_controller::ModbusController *parent_{nullptr};

  uint16_t power_register_{1};
  uint16_t fan_register_{4};
  uint16_t mode_register_{101};
  uint16_t setpoint_register_{102};
  uint16_t room_temp_register_{851};

  uint32_t command_interval_ms_{200};
  uint32_t settle_timeout_ms_{5000};
  uint32_t last_write_dispatch_ms_{0};
  uint32_t read_batch_started_ms_{0};
  uint32_t read_generation_{0};
  uint16_t pending_read_callbacks_{0};

  std::optional<uint16_t> raw_power_;
  std::optional<uint16_t> raw_mode_;
  std::optional<uint16_t> raw_fan_;
  std::optional<uint16_t> raw_setpoint_;
  std::optional<uint16_t> raw_room_temp_;

  FieldGuard guard_power_;
  FieldGuard guard_mode_;
  FieldGuard guard_fan_;
  FieldGuard guard_setpoint_;

  std::vector<PendingCommand> pending_;
};

}  // namespace actron_modbus
}  // namespace esphome
