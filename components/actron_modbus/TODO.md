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

- [ ] Modbus command collision / duplication errors showing in ESP logs.
      Removing the standalone `modbus_controller` entities that polled the same
      registers as the climate component (power 1, fan 4, mode 101, setpoint 102)
      should cut the duplicate reads — verify after flashing. Note: room temp (851)
      is still read by both the climate entity and the "Room Temperature" sensor.
      Likely also tied to the Modbus timing review below.
- [ ] On firmware update / reboot the device loses current status and the AC ends
      up turned OFF (annoying). Hypothesis: the standalone "Power" switch was
      restoring to OFF on boot and writing 0 to the power register; now removed, so
      re-check after flashing. If it persists, investigate restore-on-boot behaviour
      and whether anything writes control registers during startup.

## Home Assistant integration

- [x] Remove standalone controls now duplicated by the climate entity (power switch,
      temperature slider, AC mode select, fan speed select) and their backing raw
      registers, in `devices/esp32c3m-ss-2.yaml`. Pending mirror into ESPHome Builder
      + flash/verify.
- [ ] Integrate the controls not represented by the climate entity: continuous fan
      mode (`ac_fan_cont` / reg 105) and zone controls (`ac_raw_zone_*` / regs
      5001-5002). Kept as standalone entities for now.
- [ ] Build a companion automation to manage AC control based on house temperature
      settings and other variables.
- [ ] Review remaining diagnostic entities (raw feedback registers) and hide/remove
      any not wanted in HA.

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
      and may not be an ideal fit. Likely related to the command collision/duplication
      bug above.

## Housekeeping

- [ ] Tag a stable release (e.g. `actron_modbus-v0.x.0`) once verified so device
      YAMLs can pin to a tag instead of tracking the branch.
- [ ] Strip any temporary `ESP_LOGI("agent_dbg", ...)` instrumentation before
      merging to `main`. (The `ESP_LOGW` read-batch-timeout warning in `update()` is
      intentional — keep it.)
