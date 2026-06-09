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
- [ ] Confirm power on/off vs. mode register coupling looks right in HA (the mode
      register can read a stale non-off value while the unit is off).

## Bugs

- [ ] Modbus command collision / duplication errors in ESP logs. DIAGNOSED from
      logs: every warning is for a `modbus_controller`-polled register (703, 906,
      1104, 1301, 5001/5002), NOT the climate component's regs — so it's bus
      saturation, not a code collision. Climate (~5 reads/s @ 1s) + controller
      feedback reads exceeded bus throughput at 9600 baud / `send_wait_time: 100ms`,
      so each controller `update()` re-queued still-pending reads. FIX APPLIED:
      raised `modbus_controller` `update_interval` to 5s (climate stays at 1s).
      Verify the warnings are gone after flashing.
- [x] On firmware update / reboot the device lost current status and the AC ended up
      turned OFF. FIXED — caused by the standalone "Power" switch restoring to OFF on
      boot and writing 0 to the power register; removing it resolved it. Confirmed on
      hardware.

## Home Assistant integration

- [x] Remove standalone controls now duplicated by the climate entity (power switch,
      temperature slider, AC mode select, fan speed select) and their backing raw
      registers, in `devices/esp32c3m-ss-2.yaml`. Applied and verified on hardware.
- [ ] Integrate the controls not represented by the climate entity: continuous fan
      mode (`ac_fan_cont` / reg 105) and zone controls (`ac_raw_zone_*` / regs
      5001-5002). Kept as standalone entities for now.
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
- [ ] Collapse adjacent register reads into block reads. `mode` (101) and
      `setpoint` (102) are contiguous and could be a single 2-register read instead
      of two single-register reads. `power` (1), `fan` (4) and `room_temp` (851) are
      non-contiguous and would stay separate.
- [ ] Consider polling `room_temp` at a slower cadence than the control registers —
      it changes slowly and doesn't need the 1s control-register poll rate.
- [ ] Re-check publish chatter. We now publish once per completed read batch; verify
      there's no remaining redundant `publish_state()` churn.
- [ ] Review overall Modbus behaviour/timing (poll cadence, command interval, queue
      usage) for sanity — the initial pattern was lifted from someone else's project
      and may not be an ideal fit. Levers: `modbus` `send_wait_time` (100ms is
      conservative; lower cautiously and test for comms errors), the climate
      component's own `update_interval`/`command_interval`, and block reads (below).

## Housekeeping

- [ ] Tag a stable release (e.g. `actron_modbus-v0.x.0`) once verified so device
      YAMLs can pin to a tag instead of tracking the branch.
- [ ] Strip any temporary `ESP_LOGI("agent_dbg", ...)` instrumentation before
      merging to `main`. (The `ESP_LOGW` read-batch-timeout warning in `update()` is
      intentional — keep it.)
