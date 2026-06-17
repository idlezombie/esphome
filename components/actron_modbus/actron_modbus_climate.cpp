#include "actron_modbus_climate.h"

#include <algorithm>
#include <cmath>

#include "esphome/core/log.h"

namespace esphome {
namespace actron_modbus {

static const char *const TAG = "actron_modbus.climate";

static const char *const PRESET_NORMAL = "Normal";
static const char *const PRESET_CONTINUOUS = "Continuous";

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
  this->set_supported_custom_presets({PRESET_NORMAL, PRESET_CONTINUOUS});
  this->set_custom_preset_(PRESET_NORMAL);
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

  if (this->pending_read_callbacks_ != 0) {
    if ((millis() - this->read_batch_started_ms_) < this->settle_timeout_ms_) {
      return;
    }
    ESP_LOGW(TAG, "Read batch timed out with %u callbacks outstanding", this->pending_read_callbacks_);
    this->pending_read_callbacks_ = 0;
  }

  this->request_reads_();
}

void ActronModbusClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "Actron Modbus Climate:");
  ESP_LOGCONFIG(TAG, "  Optimistic: %s", YESNO(this->optimistic_));
  ESP_LOGCONFIG(TAG, "  Command interval: %ums", this->command_interval_ms_);
  ESP_LOGCONFIG(TAG, "  Settle timeout: %ums", this->settle_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Register power: %u", this->power_register_);
  ESP_LOGCONFIG(TAG, "  Register fan: %u", this->fan_register_);
  ESP_LOGCONFIG(TAG, "  Register mode: %u", this->mode_register_);
  ESP_LOGCONFIG(TAG, "  Register setpoint: %u", this->setpoint_register_);
  ESP_LOGCONFIG(TAG, "  Register continuous fan: %u", this->continuous_fan_register_);
  ESP_LOGCONFIG(TAG, "  Register room temp: %u", this->room_temp_register_);
}

climate::ClimateTraits ActronModbusClimate::traits() {
  auto traits = climate::ClimateTraits();
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
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

  if (call.has_custom_preset()) {
    if (call.get_custom_preset() == PRESET_CONTINUOUS) {
      this->set_custom_preset_(PRESET_CONTINUOUS);
      this->queue_or_replace_(PendingType::CONTINUOUS_FAN, this->continuous_fan_register_, 1);
    } else {
      this->set_custom_preset_(PRESET_NORMAL);
      this->queue_or_replace_(PendingType::CONTINUOUS_FAN, this->continuous_fan_register_, 0);
    }
    changed = true;
  }

  if (changed && this->optimistic_) {
    this->publish_state();
  }
}

void ActronModbusClimate::request_reads_() {
  using modbus_controller::ModbusCommandItem;
  using modbus::ModbusRegisterType;

  uint32_t generation = ++this->read_generation_;
  this->pending_read_callbacks_ = 6;
  this->read_batch_started_ms_ = millis();

  this->parent_->queue_command(ModbusCommandItem::create_read_command(
      this->parent_, ModbusRegisterType::HOLDING, this->power_register_, 1,
      [this, generation](ModbusRegisterType, uint16_t, const std::vector<uint8_t> &data) {
        this->handle_power_read_(data, generation);
      }));

  this->parent_->queue_command(ModbusCommandItem::create_read_command(
      this->parent_, ModbusRegisterType::HOLDING, this->fan_register_, 1,
      [this, generation](ModbusRegisterType, uint16_t, const std::vector<uint8_t> &data) {
        this->handle_fan_read_(data, generation);
      }));

  this->parent_->queue_command(ModbusCommandItem::create_read_command(
      this->parent_, ModbusRegisterType::HOLDING, this->mode_register_, 1,
      [this, generation](ModbusRegisterType, uint16_t, const std::vector<uint8_t> &data) {
        this->handle_mode_read_(data, generation);
      }));

  this->parent_->queue_command(ModbusCommandItem::create_read_command(
      this->parent_, ModbusRegisterType::HOLDING, this->setpoint_register_, 1,
      [this, generation](ModbusRegisterType, uint16_t, const std::vector<uint8_t> &data) {
        this->handle_setpoint_read_(data, generation);
      }));

  this->parent_->queue_command(ModbusCommandItem::create_read_command(
      this->parent_, ModbusRegisterType::HOLDING, this->continuous_fan_register_, 1,
      [this, generation](ModbusRegisterType, uint16_t, const std::vector<uint8_t> &data) {
        this->handle_continuous_fan_read_(data, generation);
      }));

  this->parent_->queue_command(ModbusCommandItem::create_read_command(
      this->parent_, ModbusRegisterType::HOLDING, this->room_temp_register_, 1,
      [this, generation](ModbusRegisterType, uint16_t, const std::vector<uint8_t> &data) {
        this->handle_room_temp_read_(data, generation);
      }));
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
    this->mark_expected_(type, value);
    return;
  }
  this->pending_.push_back(PendingCommand{type, reg, value});
  this->mark_expected_(type, value);
}

void ActronModbusClimate::mark_expected_(PendingType type, uint16_t value) {
  if (!this->optimistic_) {
    return;
  }

  FieldGuard *guard = nullptr;
  switch (type) {
    case PendingType::POWER:
      guard = &this->guard_power_;
      break;
    case PendingType::MODE:
      guard = &this->guard_mode_;
      break;
    case PendingType::SETPOINT:
      guard = &this->guard_setpoint_;
      break;
    case PendingType::FAN:
      guard = &this->guard_fan_;
      break;
    case PendingType::CONTINUOUS_FAN:
      guard = &this->guard_continuous_fan_;
      break;
  }

  guard->expected = value;
  guard->since_ms = millis();
}

uint16_t ActronModbusClimate::guarded_raw_(FieldGuard &guard, uint16_t raw) {
  if (!guard.expected.has_value()) {
    return raw;
  }

  if (raw == *guard.expected) {
    guard.expected.reset();
    return raw;
  }

  if ((millis() - guard.since_ms) < this->settle_timeout_ms_) {
    return *guard.expected;
  }

  guard.expected.reset();
  return raw;
}

void ActronModbusClimate::publish_and_save_() {
  if (!this->raw_power_.has_value() || !this->raw_mode_.has_value() || !this->raw_fan_.has_value() ||
      !this->raw_setpoint_.has_value() || !this->raw_continuous_fan_.has_value()) {
    return;
  }

  uint16_t power = this->guarded_raw_(this->guard_power_, *this->raw_power_);
  uint16_t mode = this->guarded_raw_(this->guard_mode_, *this->raw_mode_);
  uint16_t fan = this->guarded_raw_(this->guard_fan_, *this->raw_fan_);
  uint16_t setpoint = this->guarded_raw_(this->guard_setpoint_, *this->raw_setpoint_);
  uint16_t continuous_fan = this->guarded_raw_(this->guard_continuous_fan_, *this->raw_continuous_fan_);

  this->mode = mode_from_raw(power, mode);
  this->fan_mode = fan_from_raw(fan);
  this->target_temperature = float(setpoint) * 0.1f;
  if (continuous_fan == 1) {
    this->set_custom_preset_(PRESET_CONTINUOUS);
  } else {
    this->set_custom_preset_(PRESET_NORMAL);
  }
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

  this->publish_state();
}

void ActronModbusClimate::finish_read_(uint32_t generation) {
  if (generation != this->read_generation_) {
    return;
  }
  if (this->pending_read_callbacks_ > 0) {
    this->pending_read_callbacks_--;
  }
  if (this->pending_read_callbacks_ == 0) {
    this->publish_and_save_();
  }
}

void ActronModbusClimate::handle_power_read_(const std::vector<uint8_t> &data, uint32_t generation) {
  if (generation != this->read_generation_) {
    return;
  }
  this->raw_power_ = parse_u16(data);
  this->finish_read_(generation);
}

void ActronModbusClimate::handle_fan_read_(const std::vector<uint8_t> &data, uint32_t generation) {
  if (generation != this->read_generation_) {
    return;
  }
  this->raw_fan_ = parse_u16(data);
  this->finish_read_(generation);
}

void ActronModbusClimate::handle_mode_read_(const std::vector<uint8_t> &data, uint32_t generation) {
  if (generation != this->read_generation_) {
    return;
  }
  this->raw_mode_ = parse_u16(data);
  this->finish_read_(generation);
}

void ActronModbusClimate::handle_setpoint_read_(const std::vector<uint8_t> &data, uint32_t generation) {
  if (generation != this->read_generation_) {
    return;
  }
  this->raw_setpoint_ = parse_u16(data);
  this->finish_read_(generation);
}

void ActronModbusClimate::handle_continuous_fan_read_(const std::vector<uint8_t> &data, uint32_t generation) {
  if (generation != this->read_generation_) {
    return;
  }
  this->raw_continuous_fan_ = parse_u16(data);
  this->finish_read_(generation);
}

void ActronModbusClimate::handle_room_temp_read_(const std::vector<uint8_t> &data, uint32_t generation) {
  if (generation != this->read_generation_) {
    return;
  }
  this->raw_room_temp_ = parse_u16(data);
  this->finish_read_(generation);
}

}  // namespace actron_modbus
}  // namespace esphome
