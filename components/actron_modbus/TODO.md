# actron_modbus — pending / TODO

Running list of open items for the Actron Modbus climate component. Items may be
picked up by a future agent (or human) in any order; some may already be resolved
by the time you read this — verify against the current code before starting.

## Verification (blocking before merge to `main`)

- [x] Flash and verify the HA UI sync fix on real hardware. Confirmed working —
      the old "accept → revert to old → snap back to new" toggle is gone. Still on
      the `actron_modbus` branch; `main` only holds flashed + verified work.
- [ ] Confirm the timeout fallback works: set a value the unit clamps/rejects and
      check the entity reverts to the device's actual value after `settle_timeout`
      instead of staying stuck on the requested value.
- [x] Confirm power on/off vs. mode register coupling looks right in HA (the mode
      register can read a stale non-off value while the unit is off). Confirmed
      working — single climate mode selector with OFF/heat/cool/etc behaves correctly.

## Bugs

- [x] Modbus command collision / duplication errors in ESP logs. FIXED — diagnosed from
      logs: every warning is for a `modbus_controller`-polled register (703, 906,
      1104, 1301, 5001/5002), NOT the climate component's regs — so it's bus
      saturation, not a code collision. Climate (~5 reads/s @ 1s) + controller
      feedback reads exceeded bus throughput at 9600 baud / `send_wait_time: 100ms`,
      so each controller `update()` re-queued still-pending reads. FIX APPLIED:
      raised `modbus_controller` `update_interval` to 5s (climate stays at 1s).
      Confirmed warnings cleared on the latest build.
- [x] On firmware update / reboot the device lost current status and the AC ended up
      turned OFF. FIXED — caused by the standalone "Power" switch restoring to OFF on
      boot and writing 0 to the power register; removing it resolved it. Confirmed on
      hardware.

## Home Assistant integration

- [x] Remove standalone controls now duplicated by the climate entity (power switch,
      temperature slider, AC mode select, fan speed select) and their backing raw
      registers, in `devices/esp32c3m-ss-2.yaml`. Applied and verified on hardware.
- [x] Integrate continuous fan mode (reg 105) into the climate entity as custom
      presets `Normal` / `Continuous`; removed the standalone Continuous Fan switch.
      Zone controls use `actron_modbus` switch (optimistic settle) — not folded into
      the climate entity.
- [ ] Shorten HA climate mode label `fan_only` from "Fan only" to "Fan" (ESPHome
      cannot rename standard HVAC modes; needs a Home Assistant UI/translation tweak).
- [ ] Build a companion automation to manage AC control based on house temperature
      settings and other variables.
- [x] Trim diagnostic/raw entities: removed Room Temperature (dup of climate) and
      Compressor EEV Position (no value); made the Continuous Fan and Indoor Comms
      raw registers `internal`; made the zone switches normal controls (dropped
      `entity_category: diagnostic`). No raw-register entities exposed now; kept the
      feedback sensors (coil temp, compressor demand, fan speed demand) and the
      Indoor Unit Comms Status watchdog. Applied and verified on hardware.

## Performance optimisation (next focus)

- [ ] Tune `settle_timeout` (currently defaults to `5s`). 5s was a safe starting
      point; likely can be lowered once the read cadence is understood. Exposed as
      a config option so it can be changed without a code edit.
- [x] Collapse adjacent register reads into block reads. `mode` (101) + `setpoint`
      (102) are read as one 2-register block when contiguous.
- [x] Poll `room_temp` slower via `room_temp_every` (default every 3rd climate cycle).
- [x] Serialise climate reads (one outstanding command) — ESPHome 2026.8 hub refuses
      extra frames when `Frame already active … with 2 requests pending`, which made
      the old 6-at-once batch time out with 6 callbacks outstanding and never publish.
- [x] Zone switches moved to `actron_modbus` platform with optimistic + settle guard
      (stock `modbus_controller` switch has no optimistic mode → HA flap on write).
- [ ] Re-check publish chatter after serialised reads land on hardware.
- [ ] Review overall Modbus behaviour/timing (poll cadence, command interval, queue
      usage) for sanity. Levers: `modbus` `send_wait_time` (100ms is conservative;
      lower cautiously and test for comms errors), climate `update_interval` /
      `command_interval`.

## Housekeeping

- [ ] Tag a stable release (e.g. `actron_modbus-v0.x.0`) once verified so device
      YAMLs can pin to a tag instead of tracking the branch.
- [x] Strip temporary `ESP_LOGI("agent_dbg", ...)` instrumentation from device YAML.
      (The `ESP_LOGW` read-batch-timeout warning in `update()` is intentional — keep it.)
