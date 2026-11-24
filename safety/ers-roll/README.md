# ERS launch on excessive roll

`ers-roll.cpp` sets the throttle cut (`cmd.eng.cut`) and the ERS launch (`ctr.ers.launch`) when the absolute roll (`est.att.roll`) exceeds 90° to either side.

- the roll must stay over the limit for `ERS_HOLD_MS` (200 ms by default, 0 for instant launch);
- the check is skipped in `EMG` mode (the mode after power on) and while the attitude is not valid (`est.att.valid`);
- the launch is sent once, the script does nothing after that.

Constants at the top of the file: `ERS_ROLL` [deg], `ERS_HOLD_MS` [ms], `TASK_MS` [ms] check period.
