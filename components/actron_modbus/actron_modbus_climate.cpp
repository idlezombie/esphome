#include "actron_modbus_climate.h"

#include <algorithm>
#include <cmath>

#include "esphome/core/log.h"

namespace esphome {
namespace actron_modbus {

static const char *const TAG = "actron_modbus.climate";

static uint16_t parse_u16(const std::vector<uint8_t> &data) {
  if (data.size() < 2) {
    return 0;
  }
  return (uint16_t(data[0]) << 8) | uint16_t(data[1]);
}

static climate::ClimateMode mode_from_raw(uint16_t power, uint16_t mode) {
  if (power == 0) {
    return climate::CLIMATE_MODE_OFF;
  }
  switch (mode) {
    case 1:
      return climate::CLIMATE_MODE_HEAT;
    case 2:
      return climate::CLIMATE_MODE_COOL;
    case 3:
      return climate::CLIMATE_MODE_AUTO;
    case 4:
      return climate::CLIMATE_MODE_FAN_ONLY;
    default:
      return climate::CLIMATE_MODE_AUTO;
  }
}

static uint16_t raw_mode_from_mode(climate::ClimateMode mode) {
  switch (mode) {
    case climate::CLIMATE_MODE_HEAT:
      return 1;
    case climate::CLIMATE_MODE_COOL:
      return 2;
    case climate::CLIMATE_MODE_AUTO:
      return 3;
    case climate::CLIMATE_MODE_FAN_ONLY:
      return 4;
    default:
      return 3;
  }
}

static climate::ClimateFanMode fan_from_raw(uint16_t fan) {
  switch (fan) {
    case 1:
      return climate::CLIMATE_FAN_LOW;
    case 2:
      return climate::CLIMATE_FAN_MEDIUM;
    case 3:
      return climate::CLIMATE_FAN_HIGH;
    default:
      return climate::CLIMATE_FAN_LOW;
  }
}

static uint16_t raw_from_fan(climate::ClimateFanMode fan) {
  switch (fan) {
    case climate::CLIMATE_FAN_LOW:
      return 1;
    case climate::CLIMATE_FAN_MEDIUM:
      return 2;
    case climate::CLIMATE_FAN_HIGH:
      return 3;
    default:
      return 1;
  }
}

void ActronModbusClimate::setup() {
  this->mode = climate::CLIMATE_MODE_OFF;
  this->fan_mode = climate::CLIMATE_FAN_LOW;
  this->target_temperature = 24.0f;
  this->current_temperature = NAN;
  this->action = climate::CLIMATE_ACTION_OFF;
  this->publish_state();
}

void ActronModbusClimate::loop() {
  if (this->parent_ == nullptr) {
    return;
  }

  bool next_available = !this->parent_->get_module_offline();
  if (next_available != this->available_) {
    this->available_ = next_available;
    this->publish_state();
  }

  if (!this->has_pending_()) {
    return;
  }
  if ((millis() - this->last_write_dispatch_ms_) < this->command_interval_ms_) {
    return;
  }

  this->dispatch_next_write_();
  this->last_write_dispatch_ms_ = millis();
}

void ActronModbusClimate::update() {
  if (this->parent_ == nullptr) {
    return;
  }
  this->request_reads_();
}

void ActronModbusClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "Actron Modbus Climate:");
  ESP_LOGCONFIG(TAG, "  Optimistic: %s", YESNO(this->optimistic_));
  ESP_LOGCONFIG(TAG, "  Command interval: %ums", this->command_interval_ms_);
  ESP_LOGCONFIG(TAG, "  Register power: %u", this->power_register_);
  ESP_LOGCONFIG(TAG, "  Register fan: %u", this->fan_register_);
  ESP_LOGCONFIG(TAG, "  Register mode: %u", this->mode_register_);
  ESP_LOGCONFIG(TAG, "  Register setpoint: %u", this->setpoint_register_);
  ESP_LOGCONFIG(TAG, "  Register room temp: %u", this->room_temp_register_);
}

climate::ClimateTraits ActronModbusClimate::traits() {
  auto traits = climate::ClimateTraits();
  traits.set_supports_current_temperature(true);
  traits.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
      climate::CLIMATE_MODE_HEAT,
      climate::CLIMATE_MODE_COOL,
      climate::CLIMATE_MODE_AUTO,
      climate::CLIMATE_MODE_FAN_ONLY,
  });
  traits.set_supported_fan_modes({
      climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_MEDIUM,
      climate::CLIMATE_FAN_HIGH,
  });
  traits.set_visual_min_temperature(16.0f);
  traits.set_visual_max_temperature(30.0f);
  traits.set_visual_target_temperature_step(0.5f);
  traits.set_visual_current_temperature_step(0.1f);
  return traits;
}

void ActronModbusClimate::control(const climate::ClimateCall &call) {
  bool changed = false;

  if (call.get_mode().has_value()) {
    auto mode = *call.get_mode();
    this->mode = mode;
    if (mode == climate::CLIMATE_MODE_OFF) {
      this->action = climate::CLIMATE_ACTION_OFF;
      this->queue_or_replace_(PendingType::POWER, this->power_register_, 0);
    } else {
      this->action = climate::CLIMATE_ACTION_IDLE;
      this->queue_or_replace_(PendingType::POWER, this->power_register_, 1);
      this->queue_or_replace_(PendingType::MODE, this->mode_register_, raw_mode_from_mode(mode));
    }
    changed = true;
  }

  if (call.get_fan_mode().has_value()) {
    auto fan_mode = *call.get_fan_mode();
    this->fan_mode = fan_mode;
    this->queue_or_replace_(PendingType::FAN, this->fan_register_, raw_from_fan(fan_mode));
    changed = true;
  }

  if (call.get_target_temperature().has_value()) {
    float temp = *call.get_target_temperature();
    if (temp < 16.0f) {
      temp = 16.0f;
    }
    if (temp > 30.0f) {
      temp = 30.0f;
    }
    float stepped = std::round(temp * 2.0f) / 2.0f;
    this->target_temperature = stepped;
    auto raw = uint16_t(std::lround(stepped * 10.0f));
    this->queue_or_replace_(PendingType::SETPOINT, this->setpoint_register_, raw);
    changed = true;
  }

  if (changed && this->optimistic_) {
    this->last_publish_was_device_ = false;
    this->publish_state();
  }
}

void ActronModbusClimate::request_reads_() {
  using modbus_controller::ModbusCommandItem;
  using modbus::ModbusRegisterType;

  this->parent_->queue_command(ModbusCommandItem::create_read_command(
      this->parent_, ModbusRegisterType::HOLDING, this->power_register_, 1,
      [this](ModbusRegisterType, uint16_t, const std::vector<uint8_t> &data) { this->handle_power_read_(data); }));

  this->parent_->queue_command(ModbusCommandItem::create_read_command(
      this->parent_, ModbusRegisterType::HOLDING, this->fan_register_, 1,
      [this](ModbusRegisterType, uint16_t, const std::vector<uint8_t> &data) { this->handle_fan_read_(data); }));

  this->parent_->queue_command(ModbusCommandItem::create_read_command(
      this->parent_, ModbusRegisterType::HOLDING, this->mode_register_, 1,
      [this](ModbusRegisterType, uint16_t, const std::vector<uint8_t> &data) { this->handle_mode_read_(data); }));

  this->parent_->queue_command(ModbusCommandItem::create_read_command(
      this->parent_, ModbusRegisterType::HOLDING, this->setpoint_register_, 1,
      [this](ModbusRegisterType, uint16_t, const std::vector<uint8_t> &data) { this->handle_setpoint_read_(data); }));

  this->parent_->queue_command(ModbusCommandItem::create_read_command(
      this->parent_, ModbusRegisterType::HOLDING, this->room_temp_register_, 1,
      [this](ModbusRegisterType, uint16_t, const std::vector<uint8_t> &data) { this->handle_room_temp_read_(data); }));
}

void ActronModbusClimate::dispatch_next_write_() {
  if (!this->has_pending_()) {
    return;
  }
  auto cmd = this->pending_.front();
  this->pending_.erase(this->pending_.begin());
  this->parent_->queue_command(
      modbus_controller::ModbusCommandItem::create_write_single_command(this->parent_, cmd.reg, cmd.value));
}

void ActronModbusClimate::queue_or_replace_(PendingType type, uint16_t reg, uint16_t value) {
  auto it = std::find_if(this->pending_.begin(), this->pending_.end(),
                         [type](const PendingCommand &item) { return item.type == type; });
  if (it != this->pending_.end()) {
    it->reg = reg;
    it->value = value;
    return;
  }
  this->pending_.push_back(PendingCommand{type, reg, value});
}

void ActronModbusClimate::publish_and_save_() {
  if (!this->raw_power_.has_value() || !this->raw_mode_.has_value() || !this->raw_fan_.has_value() ||
      !this->raw_setpoint_.has_value()) {
    return;
  }

  this->mode = mode_from_raw(*this->raw_power_, *this->raw_mode_);
  this->fan_mode = fan_from_raw(*this->raw_fan_);
  this->target_temperature = float(*this->raw_setpoint_) * 0.1f;
  if (this->raw_room_temp_.has_value()) {
    this->current_temperature = float(*this->raw_room_temp_) * 0.1f;
  }

  if (this->mode == climate::CLIMATE_MODE_OFF) {
    this->action = climate::CLIMATE_ACTION_OFF;
  } else if (this->mode == climate::CLIMATE_MODE_HEAT) {
    this->action = climate::CLIMATE_ACTION_HEATING;
  } else if (this->mode == climate::CLIMATE_MODE_COOL) {
    this->action = climate::CLIMATE_ACTION_COOLING;
  } else if (this->mode == climate::CLIMATE_MODE_FAN_ONLY) {
    this->action = climate::CLIMATE_ACTION_FAN;
  } else {
    this->action = climate::CLIMATE_ACTION_IDLE;
  }

  this->last_publish_was_device_ = true;
  this->publish_state();
}

void ActronModbusClimate::handle_power_read_(const std::vector<uint8_t> &data) {
  this->raw_power_ = parse_u16(data);
  this->publish_and_save_();
}

void ActronModbusClimate::handle_fan_read_(const std::vector<uint8_t> &data) {
  this->raw_fan_ = parse_u16(data);
  this->publish_and_save_();
}

void ActronModbusClimate::handle_mode_read_(const std::vector<uint8_t> &data) {
  this->raw_mode_ = parse_u16(data);
  this->publish_and_save_();
}

void ActronModbusClimate::handle_setpoint_read_(const std::vector<uint8_t> &data) {
  this->raw_setpoint_ = parse_u16(data);
  this->publish_and_save_();
}

void ActronModbusClimate::handle_room_temp_read_(const std::vector<uint8_t> &data) {
  this->raw_room_temp_ = parse_u16(data);
  this->publish_and_save_();
}

}  // namespace actron_modbus
}  // namespace esphome
