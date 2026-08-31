#pragma once

namespace esphome {
namespace actron_modbus {

// Climate takes priority on the shared Modbus hub. Zone switches defer polls
// while a climate read chain is in progress.
inline bool &climate_bus_busy() {
  static bool busy = false;
  return busy;
}

}  // namespace actron_modbus
}  // namespace esphome
