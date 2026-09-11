# APX scripts for camera release counting

Two companion scripts around the `ctr.env.cam.shot` camera trigger line:

- `nav_test.cpp` — periodically fires the camera trigger.
- `photo.cpp` — counts releases and cross-checks them against external counter-microchip
  feedback to detect missed or spurious releases.

## nav_test.cpp

| Variable | Mandala | Direction | Description |
|---|---|---|---|
| `CAM_RELEASE` | `ctr.env.cam.shot` | out | camera release signal (`off`/`single`/`series`) |
| `M_SHOTS_SENT` | `est.usrw.w4` | out | telemetry: total shots triggered (`shots_sent`) |

Once a second, sets `CAM_RELEASE` to `single`, increments `shots_sent`, and 50ms later sets it
back to `off`. Timing is tracked with `time_ms()` inside a single 100Hz polling task rather than
scheduling a second one-shot task per pulse, since `task()` is meant to be called once per named
function at startup (every script in this repo follows that convention) — calling it repeatedly
from within a running task leaks a handle each time.

## photo.cpp

`CAM_RELEASE` is a fixed Mandala field; the `usrb`/`usrw` indices below are examples, remap as needed:

| Variable | Mandala | Direction | Description |
|---|---|---|---|
| `CAM_RELEASE` | `ctr.env.cam.shot` | in | camera release signal (`off`/`single`/`series`), pulses to `single` for ~50ms |
| `MC_BIT0` | `est.usrb.b2` | in | counter microchip feedback, bit0 |
| `MC_BIT1` | `est.usrb.b3` | in | counter microchip feedback, bit1 |
| `MC_RESET` | `est.usrb.b4` | out | pulse 1 then 0 to reset the counter microchip |
| `M_RELEASE_COUNTER` | `est.usrw.w1` | out | telemetry: total releases counted |
| `M_MC_COUNTER` | `est.usrw.w2` | out | telemetry: last microchip counter reading |
| `M_ERROR_COUNTER` | `est.usrw.w3` | out | telemetry: accumulated mismatch errors |
| `M_COMMANDS_SENT` | `est.usrw.w4` | out | telemetry: `commands_sent`, only zeroed at startup for now |
| `M_MC_TOTAL` | `est.usrw.w5` | out | telemetry: cumulative sum of every microchip counter reading |

`CAM_RELEASE` pulses to `single` for only ~50ms, so the task polls at 100Hz (every 10ms) to
reliably catch the edge. Every `off`→`single` transition increments `RELEASE_COUNTER`; the
`series` value is not otherwise handled by this script.

On startup, `RELEASE_COUNTER`, `error_counter`, `commands_sent` and `mc_total` are explicitly
published as `0` (`mc_counter`/`M_MC_COUNTER` is left alone, since it will reflect a real reading
after the first check), and the microchip is reset once via `MC_RESET` so it starts from a known
state alongside the counters.

The microchip counter (`MC_BIT0`/`MC_BIT1`) is a free-running 2-bit counter (`00`, `01`, `10`,
`11`, `00`, ...) that ticks once per physical release it detects. After each `CAM_RELEASE` edge,
the script waits `CHECK_DELAY_MS` (100ms, longer than the ~50ms pulse so the check always happens
after it has fully finished) and then reads the microchip counter:

- `1` — OK, only our own release was counted
- `0` — the microchip missed the release entirely → +1 error
- `2` — 1 spurious extra count → +1 error
- `3` — 2 spurious extra counts → +2 errors

Errors are accumulated into `error_counter` and published as `M_ERROR_COUNTER`. Each reading is
also added to `mc_total` (published as `M_MC_TOTAL`), a running count of every increment the
microchip has ever reported, independent of `RELEASE_COUNTER` or any mismatch.

After the check, the script resets the microchip back to `0` by pulsing `MC_RESET` high then
low, so the next release always starts the comparison from a known `0` baseline instead of
letting the two counters drift out of sync over time. The 74HC393 only needs a ~24ns HIGH pulse
on its reset input to clear (`tW`/`tPHL`/`trec` are all well under 1us per its datasheet), so the
pulse is issued as two back-to-back publishes in the same task tick rather than held open across
an extra cycle.
