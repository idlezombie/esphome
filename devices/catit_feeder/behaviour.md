# Feeder — Behavioural Contract

Invariants that must hold after every change to this device's firmware. Each
REQ has a stable ID; reference the ID in commits or chat when intentionally
changing one.

When modifying device YAML in this folder, read this file first and verify
each REQ-* still holds after your changes. If a REQ is intentionally changed,
removed, or added, update this file in the same commit and call it out in the
commit message (e.g. `feeder: relax REQ-D6 to 3s`).

## States

- **REQ-S1**: `feeder_status_state` is one of: `idle`, `dispensing`, `back_off`, `success`, `jammed`, `hopper_empty`, `bowl_missing`.
- **REQ-S2**: `motor_jammed_state` clears only on a successful dispense.
- **REQ-S3**: `hopper_empty` auto-clears to `idle` when the hopper sensor reports non-empty.
- **REQ-S4**: `back_off` is the transient state during the reverse-then-retry phase.
- **REQ-S5**: While `dispensing` or `success`, status overrides (jammed / bowl_missing / hopper_empty) from the periodic interval are suppressed.
- **REQ-S6**: `bowl_missing` auto-clears to `idle` once the bowl is detected on the scale again (and the hopper is not empty).
- **REQ-S7**: Periodic status priority is `jammed` > `bowl_missing` > `hopper_empty`.

## Dispense

- **REQ-D1**: Normal dispense is blocked when `motor_jammed_state` is true.
- **REQ-D2**: Normal dispense is blocked when the hopper sensor reads empty.
- **REQ-D2b**: Normal dispense is blocked when `bowl_missing` is true (checked before the hopper, both at entry and at the back-off retry re-check). Blocking sets status `bowl_missing` and clears `soft_fail_pending`.
- **REQ-D3**: Force dispense bypasses REQ-D1, REQ-D2, and REQ-D2b.
- **REQ-D4**: Manual button press 20–800ms triggers normal dispense.
- **REQ-D5**: Manual button press 800ms–5s triggers force dispense.
- **REQ-D6**: Status returns to `idle` 2s after a successful dispense.
- **REQ-D7**: Tacho timeout for a forward attempt is 2.3s.

## Bowl detection

- **REQ-B1**: The bowl is considered missing when `scale_feeder_weight.state < -10g`, sustained for 2s (`delayed_on: 2s`, `delayed_off: 1s`) via the `bowl_missing` binary sensor.
- **REQ-B2**: Detection uses the calibrated, pre-tare reading. The `calibrate_linear` zero point was set with the empty bowl present, so a removed bowl extrapolates strongly negative (~ -bowl_mass). The bowl's absolute mass is not stored or required, and the tare offset is intentionally NOT subtracted (the 10g margin dwarfs expected drift).
- **REQ-B3**: The calibration must not be removed — it is what anchors "empty bowl present" to 0g and makes REQ-B1 a simple fixed threshold.

## Failure handling

- **REQ-F1**: A missed tacho on the first attempt triggers a 500ms reverse backoff, then one retry forward.
- **REQ-F2**: Two consecutive failed dispense attempts set `motor_jammed_state = true` and status `jammed`.
- **REQ-F3**: A successful dispense clears `soft_fail_pending` and `motor_jammed_state`.
- **REQ-F4**: A hopper-empty detection clears `soft_fail_pending` (does not count as a fail).

## LEDs

- **REQ-L1**: Error LED solid when status == `jammed`.
- **REQ-L2**: Error LED blinks (~1Hz) when status == `hopper_empty` or `bowl_missing` (shared pattern; distinguish via the Status text sensor).
- **REQ-L3**: Wifi LED solid when connected, blinks when not.
- **REQ-L4**: Status LED power switch defaults OFF at boot (`RESTORE_DEFAULT_OFF`).
- **REQ-L5**: Diagnostics LED is the ESP32 status_led on GPIO15.

## Persistence

- **REQ-P1**: `feeder_tare_offset` persists across reboots.
- **REQ-P2**: All other globals reset to their initial value on boot.

## UX

- **REQ-U1**: Tare button captures the current raw scale weight as the new offset.
- **REQ-U2**: Manual button is debounced 20ms on / 20ms off.
- **REQ-U3**: "Dispensed Weight" reports `raw_weight − tare_offset`, clamped to 0 within ±0.5g.

## Hardware (reference)

| GPIO | Function |
|---|---|
| 0  | Motor tacho (pulse counter, rising edge) |
| 2  | HX711 DOUT |
| 15 | Diagnostics LED (status_led, inverted, ignore_strapping_warning) |
| 16 | Wifi LED (output, inverted) |
| 17 | Manual dispense button (input pullup, inverted) |
| 18 | Motor forward output |
| 19 | Hopper sensor (input pullup, inverted, problem class) |
| 20 | Motor reverse output |
| 21 | HX711 SCK |
| 22 | Status LED power output |
| 23 | Error LED (output, inverted) |

## Notes for future revisions

- If the dispense flow is rewritten, preserve REQ-F1 through REQ-F3 — the soft-fail / two-strikes pattern is what prevents nuisance jam alarms.
- The `wait_until: motor_tacho_detected is_off; timeout: 1s` priming step before each forward attempt is intentional — see the comment in the YAML — and exists to avoid latching on a residual pulse from the previous attempt.
- Status updates are driven from the 500ms interval AND from the dispense script. Be careful not to race these (REQ-S5).
