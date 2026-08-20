# APX script for camera release counter

Script counts camera shutter releases (`CAM_RELEASE`) and cross-checks that count against
feedback from an external 2-bit counter microchip (e.g. 74HC393) wired to the release line,
to detect missed or spurious releases.

## Bus mapping

`CAM_RELEASE` is a fixed Mandala field; the `usrb`/`usr` indices below are examples, remap as needed:

| Variable | Mandala | Direction | Description |
|---|---|---|---|
| `CAM_RELEASE` | `ctr.env.cam.shot` | in | camera release signal (`off`/`single`/`series`), pulses to `single` for ~50ms |
| `MC_BIT0` | `est.usrb.b2` | in | counter microchip feedback, bit0 |
| `MC_BIT1` | `est.usrb.b3` | in | counter microchip feedback, bit1 |
| `MC_RESET` | `est.usrb.b4` | out | pulse 1 then 0 to reset the counter microchip |
| `M_RELEASE_COUNTER` | `est.usr.u1` | out | telemetry: total releases counted |
| `M_MC_COUNTER` | `est.usr.u2` | out | telemetry: last microchip counter reading |
| `M_ERROR_COUNTER` | `est.usr.u3` | out | telemetry: accumulated mismatch errors |

## Behavior

`CAM_RELEASE` pulses to `single` for only ~50ms, so the task polls at 100Hz (every 10ms) to
reliably catch the edge. Every `off`→`single` transition increments `RELEASE_COUNTER`; the
`series` value is not otherwise handled by this script.

The microchip counter (`MC_BIT0`/`MC_BIT1`) is a free-running 2-bit counter (`00`, `01`, `10`,
`11`, `00`, ...) that ticks once per physical release it detects. After each `CAM_RELEASE` edge,
the script waits `CHECK_DELAY_MS` (60ms, longer than the ~50ms pulse so the check always happens
after it has fully finished) and then reads the microchip counter:

- `1` — OK, only our own release was counted
- `0` — the microchip missed the release entirely → +1 error
- `2` — 1 spurious extra count → +1 error
- `3` — 2 spurious extra counts → +2 errors

Errors are accumulated into `error_counter` and published as `M_ERROR_COUNTER`.

After the check, the script resets the microchip back to `0` by pulsing `MC_RESET` high then
low, so the next release always starts the comparison from a known `0` baseline instead of
letting the two counters drift out of sync over time. The 74HC393 only needs a ~24ns HIGH pulse
on its reset input to clear (`tW`/`tPHL`/`trec` are all well under 1us per its datasheet), so the
pulse is issued as two back-to-back publishes in the same task tick rather than held open across
an extra cycle.
