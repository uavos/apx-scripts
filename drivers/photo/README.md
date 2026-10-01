# APX script for camera release counting

`photo.cpp` counts releases of the `ctr.env.cam.shot` camera trigger line and cross-checks them
against external counter-microchip feedback to detect missed, spurious, or uncommanded releases.

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

Either way, once a reading is taken the microchip is reset back to `0` by pulsing `MC_RESET` high
then low (two back-to-back publishes in the same tick — the 74HC393 itself only needs a ~24ns
HIGH pulse to clear per its datasheet), so the next reading always starts from a known baseline.

That reset is a bus/GPIO round-trip though, not instant from the script's point of view, so after
issuing it the script enters a third state, `wait_reset_confirm`, and polls the counter without
processing anything until it actually reads back `0` (or `CHECK_DELAY_MS` passes, as a fallback
so it can't get stuck here forever). Only then does it return to `idle` and resume treating a
nonzero reading as a new event. Without this gate, a reset that takes more than one 10ms tick to
physically land would have its own not-yet-cleared leftover value picked up again by the
uncommanded path on every tick until it actually clears, inflating a single real release into
several extra counts.
