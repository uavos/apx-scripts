const idx_hold = f_sw_sw4;

const idx_ctr_cthrottle = f_ctr_throttle;
const idx_new_ctr_cthrottle = f_radar_Vz;


main()
{
    set_var(f_cmode_dlhd, 1.0, true);
}

@OnTask()
{
    set_var(f_rpm, get_var(f_user6), true);

    hold_proc();

    ctr_throttle_correction();

    return 20;
}

Float:limit(Float:value, Float:min, Float:max)
{
    if (value < min)
        return min;

    if (value > max)
        return max;

    return value;
}


hold_proc()
{
    new mode = get_var(f_mode);

    if(mode == mode_EMG) {
        return;
    } if (mode == mode_TAXI) {
        set_var(idx_hold, 0.0, true)
    } else {
        set_var(idx_hold, 1.0, true);
    }
}

ctr_throttle_correction()
{
    new Float: ctr_exp = pow(get_var(idx_ctr_cthrottle), 1.8);

    set_var(idx_new_ctr_cthrottle, limit(ctr_exp, 0.0, 1.0), true);
}
