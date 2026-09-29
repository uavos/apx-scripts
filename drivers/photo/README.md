# APX scripts for camera release counting

Two companion scripts around the `ctr.env.cam.shot` camera trigger line:

- `nav_test.cpp` — periodically fires the camera trigger.
- `photo.cpp` — counts releases and cross-checks them against external counter-microchip
  feedback to detect missed, spurious, or uncommanded releases.

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
`11`, `00`, ...) that ticks once per physical release it detects, and it is read and reset back
to `0` every time the script processes a reading — so on any given tick it holds only whatever
has happened since the last time it was checked.

There are two independent paths that read and process it, both adding to `mc_total`
(`M_MC_TOTAL`) and `error_counter` (`M_ERROR_COUNTER`):

- **Commanded path** (state `wait_check`): after a `CAM_RELEASE` `off`→`single` edge, the script
  waits `CHECK_DELAY_MS` (100ms, longer than the ~50ms pulse so the check always happens after it
  has fully finished) and reads the counter, expecting exactly `1`:
  - `1` — OK, only our own release was counted
  - `0` — the microchip missed the release entirely → +1 error
  - `2` — 1 spurious extra count → +1 error
  - `3` — 2 spurious extra counts → +2 errors
- **Uncommanded path** (state `idle`): on every other tick — i.e. whenever no `CAM_RELEASE` is
  currently pending a check — the script also reads the counter. Any nonzero reading here means
  a photo happened with no command behind it at all, so the whole reading counts as error
  (`error_counter += mc_counter`), on top of being added to `mc_total`.

Either way, once a reading is taken the microchip is immediately reset back to `0` by pulsing
`MC_RESET` high then low, so the next reading (commanded or not) always starts from a known
baseline instead of letting counters drift. The 74HC393 only needs a ~24ns HIGH pulse on its
reset input to clear (`tW`/`tPHL`/`trec` are all well under 1us per its datasheet), so the pulse
is issued as two back-to-back publishes in the same task tick rather than held open across an
extra cycle.
