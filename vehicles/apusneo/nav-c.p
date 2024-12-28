#define TASK_DELAY_MS           20

//---------------------------vesc id-------------------------------
#define VESC_SET_DUTY           0
#define VESC_SET_CURRENT        1
#define VESC_SET_CURRENT_BRAKE  2
//-----------------------------------------------------------------
#define CURRENT_MAX             50.0
#define START_CURRENT_BRAKE     0.5
#define CURRENT_BRAKE           1.0     
#define THR_CHANGE_RATE         10      
#define BRAKE_CHANGE_RATE       0.1     
#define BRAKE_MAX               7       

//---------------------------Speed Throttle------------------------
new Float: air_brk = 0.0;
new Float: prev_thr = 0.0;              
new Float: delta_thr = 0.0;             
new Float: out_thr = 0.0;               
new Float: c_brake = 0.0;               
new idx_vesc_control = f_user5;         //+
new idx_vesc_mode = f_user6;            //+

//-----------------------------OnTask------------------------------
new vesc_timer = 0;
new heater_timer = 0;

const VESC_TIMEOUT = 100;
const HEATER_TIMEOUT = 1000;

//---------------------------temperature----------------------
const idx_nav_temp =        f_RT;
const idx_ctr_heater =      f_userb_2
const START_HEATER =        10;


new Float: ch_throttle = 0.0;

main()
{   
    new now = time();
    vesc_timer = now;
    heater_timer = now;
}

@OnTask()
{
    new now = time();

    if (now - vesc_timer > VESC_TIMEOUT) {
        vesc_timer = now;

        new Float: dt = VESC_TIMEOUT / 1000.0;                              
        delta_thr = (THR_CHANGE_RATE / 100.0) * dt;

        if (air_brk == 0) {
            out_thr = get_ch(0);                                            //throttle value from autopilot
            if(out_thr > prev_thr) {
                if((out_thr - prev_thr) > delta_thr) {                      
                    out_thr = prev_thr + delta_thr;
                }
            }
        }
        prev_thr = out_thr;
        ch_throttle = out_thr;

        new bool: thr_cut   =  (get_var(f_cmode_thrcut) > 0.01);
        new bool: air_brake =  (get_var(f_ctr_airbrk) > 0.1);
        new Float: airspeed =  get_var(f_airspeed);
        new rpm =              get_var(f_rpm);

        if ((ch_throttle > 0.001) && !air_brake && !thr_cut) {
            c_brake = START_CURRENT_BRAKE;
            setDuty(ch_throttle);
        } else if (((ch_throttle < 0.001) && !air_brake) || thr_cut || (air_brake && (airspeed < 8))) {
            prev_thr = 0.0;
            if (c_brake < BRAKE_MAX) {
                c_brake += BRAKE_CHANGE_RATE;
            }
            setCurrentBrake(c_brake);            
        } else if (air_brake && (airspeed > 8) && !thr_cut) {
            c_brake = START_CURRENT_BRAKE;

            if (rpm < 200) {
                out_thr = 0.05;
                ch_throttle = out_thr * CURRENT_MAX;
                setCurrent(ch_throttle);
            }

            if (rpm > 400) {
                setCurrentBrake(CURRENT_BRAKE);
            }
        }

        if (air_brake) {
            prev_thr = 0.04;                
        }
    }

    if (now - heater_timer > HEATER_TIMEOUT) {
        heater_timer = now;

        new RT = get_var(idx_nav_temp);
        if (RT < START_HEATER) {
            set_var(idx_ctr_heater, 1.0, true);
        }

        if (RT > START_HEATER * 1.5) {
            set_var(idx_ctr_heater, 0.0, true);
        }
    }

    set_var(f_user3, get_var(f_ls_ail), true);
    set_var(f_user4, get_var(f_rs_ail), true);

    return TASK_DELAY_MS;
}

setCurrent(Float: val)
{
    set_var(idx_vesc_mode, VESC_SET_CURRENT, true);
    set_var(idx_vesc_control, val, true);
}

setCurrentBrake(Float:val)
{
    set_var(idx_vesc_mode, VESC_SET_CURRENT_BRAKE, true);
    set_var(idx_vesc_control, val, true);
}

setDuty(Float:val)
{
    set_var(idx_vesc_mode, VESC_SET_DUTY, true);
    set_var(idx_vesc_control, val, true);
}

forward @vm_status()
@vm_status()
{
    printf("NAV-C:Ok...\n");
}
