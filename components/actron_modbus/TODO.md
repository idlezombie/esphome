# actron_modbus — pending / TODO

Running list of open items for the Actron Modbus climate component. Items may be
picked up by a future agent (or human) in any order; some may already be resolved
by the time you read this — verify against the current code before starting.

## Verification (blocking before merge to `main`)

- [ ] Flash and verify the HA UI sync fix on real hardware. `main` only holds
      flashed + verified work; this is currently on the `actron_modbus` branch.
      Confirm the old "accept → revert to old → snap back to new" toggle is gone
      for target temp, power/mode, and fan changes.
- [ ] Confirm the timeout fallback works: set a value the unit clamps/rejects and
      check the entity reverts to the device's actual value after `settle_timeout`
      instead of staying stuck on the requested value.
- [ ] Confirm power on/off vs. mode register coupling looks right in HA (the mode
      register can read a stale non-off value while the unit is off).

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

## Housekeeping

- [ ] Tag a stable release (e.g. `actron_modbus-v0.x.0`) once verified so device
      YAMLs can pin to a tag instead of tracking the branch.
- [ ] Strip any temporary `ESP_LOGI("agent_dbg", ...)` instrumentation before
      merging to `main`. (The `ESP_LOGW` read-batch-timeout warning in `update()` is
      intentional — keep it.)
