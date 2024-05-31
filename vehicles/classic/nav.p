#define TASK_DELAY_MS 20

#define CAN_PACKET_SET_DUTY  0
#define CAN_PACKET_SET_CURRENT 1
#define CAN_PACKET_SET_CURRENT_BRAKE 2


//===========================
#define CURRENT_MAX 50.0
#define START_CURRENT_BRAKE 0.5
#define CURRENT_BRAKE 1.0             //ток рекуперации
#define THR_CHANGE_RATE 10            //10% изменение throttle в сек (рассчитываем на сколько может измениться throttle за одну итерацию)
#define BRAKE_CHANGE_RATE 0.1         //0.1 ампер нарастание тока за одну итерацию
#define BRAKE_MAX 7                   //максимальный ток складывания винта
//===========================

//===========================Speed Throttle========================
new last_time_throttle = 0;           //предыдущее время вызова скрипта, нужно для отслеживания изменения trottle
new Float: air_brk = 0.0;
new Float: prev_thr = 0.0;            //значение throttle на предыдущей итерации
new Float: delta_thr = 0.0;           //изменение throttle за одну итерацию
new Float: out_thr = 0.0;             //выходное значение throttle
new Float: c_brake = 0.0;             //ток складывания винта изменяется от START_CURRENT_BRAKE до 5
new idx_vesc_control = f_user5;       //+
new idx_vesc_mode = f_user6;          //+

//=============================OnTask=============================
new cntStarterRequest = 0;
new Float: ch_throttle = 0.0;

main()
{
    set_var(f_cmode_dlhd, 1.0, true);
    set_var(f_platform_Vdown, -1.0, true);

    last_time_throttle = time();
}

@OnTask()
{
    new time_now = time();

    if (++cntStarterRequest >= 2) {
        cntStarterRequest = 0;

        new bool: thr_cut   =  (get_var(f_cmode_thrcut) > 0.01);
        new bool: air_brake =  (get_var(f_ctr_airbrk) > 0.1);
        new Float: airspeed =  get_var(f_airspeed);
        new rpm =              get_var(f_rpm);


        //====speed throttle====
        new Float: dt = (time_now - last_time_throttle) / 1000.0;           //мсек -> сек
        delta_thr = (THR_CHANGE_RATE / 100.0) * dt;

        if (air_brk == 0) {
            out_thr = get_ch(0);                                            //значение throttle с автопилота
            if(out_thr > prev_thr) {
                if((out_thr - prev_thr) > delta_thr) {                      //если за одну итерацию значение поменялось больше чем delta_thr
                    out_thr = prev_thr + delta_thr;
                }
            }
        }

        prev_thr = out_thr;
        last_time_throttle = time_now;

        ch_throttle = out_thr;

        if ((ch_throttle > 0.001) && !air_brake && !thr_cut) {
            c_brake = START_CURRENT_BRAKE;
            setDuty(ch_throttle);
        } else if (((ch_throttle < 0.001) && !air_brake) || thr_cut || (air_brake && (airspeed < 8))) {
            prev_thr = 0.0;
            if(c_brake < BRAKE_MAX){
                c_brake += BRAKE_CHANGE_RATE;
            }
            setCurrentBrake(c_brake);            // ток складывания винта
        } else if (air_brake && (airspeed > 8) && !thr_cut) {
            c_brake = START_CURRENT_BRAKE;

            if (rpm < 200) {
                out_thr = 0.05;
                ch_throttle = out_thr * CURRENT_MAX;
                setCurrent(ch_throttle);
            }

            if (rpm > 400) {
                setCurrentBrake(CURRENT_BRAKE);  // рекуперативное торможение винтом
            }
        }

        if (air_brake) {
            prev_thr = 0.04;                     // для плавного выхода из airbrk
        }
    }

    return TASK_DELAY_MS;
}
//==============================================================================================
setCurrent(Float: val)
{
    set_var(idx_vesc_mode,CAN_PACKET_SET_CURRENT, true);
    set_var(idx_vesc_control,val, true);
}
//----------------------------------------------------------------------------------------------
setCurrentBrake(Float:val)
{
    set_var(idx_vesc_mode,CAN_PACKET_SET_CURRENT_BRAKE, true);
    set_var(idx_vesc_control,val, true);
}
//----------------------------------------------------------------------------------------------
setDuty(Float:val)
{
    set_var(idx_vesc_mode,CAN_PACKET_SET_DUTY, true);
    set_var(idx_vesc_control,val, true);
}
//----------------------------------------------------------------------------------------------
