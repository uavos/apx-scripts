const TASK_DELAY_MS = 50;

//ports id
const SWIWIN_PORT_ID = 30;  //9600

//data size
const SWIWIN_DATA_SIZE = 7;

//--------------------- FUEL -----------------------
#define PORT_RS485      240
#define ADR_SENS1       75
#define ADR_SENS2       76
#define ADR_SENS3       77
#define PACK_SIZE       9

//editable
#define V_MAX1          17.0    //liters
#define V_MAX2          18.3    //liters
#define V_MAX3          24.6    //liters

#define CRITICAL_LOW    7.0      // 7% = 1.1l
#define TANK_1_POINT    20.0     // 20% = 3.4l
#define TANK_2_POINT    25.0     // 25% = 4.5l
#define TANK_3_POINT    60.0     // 60% = 14.7l

#define TIME_SA         2       //sec

#define DELAY_PUMP      15000   //msec  between different pumps

//read
new MANDALA_IGNITION = f_power_ignition;
new MANDALA_ERS = f_ctrb_ers;
new MANDALA_ALGORITM = f_userb_1;

//write
new MANDALA_PUMP1 = f_userb_2;
new MANDALA_PUMP2 = f_userb_3;
new MANDALA_PUMP3 = f_userb_4;
new MANDALA_FUEL = f_fuel;
new MANDALA_FUEL_V1 = f_radar_dx;
new MANDALA_FUEL_V2 = f_radar_dy;
new MANDALA_FUEL_V3 = f_radar_dz;

new MANDALA_WARN_ANSWER1 = f_radar_Vx;
new MANDALA_WARN_ANSWER2 = f_radar_Vy;
new MANDALA_WARN_ANSWER3 = f_radar_Vz;

new msg{PACK_SIZE};
new m_sensRequest = 0; // 0 - first sens, 1 - second, ect...

new m_ers;
new m_algoritm;   //0 - enable; 1 - disable
new m_algoritmOld;

new m_ignition;
new m_ignitionOld;

new m_pump1 = 0;
new m_pump2 = 0;
new m_pump3 = 0;
new pump_stage = 1; //default stage

new Float: m_vFuelPersent1;
new Float: m_vFuelPersent2;
new Float: m_vFuelPersent3;
new m_timeAns1;
new m_timeAns2;
new m_timeAns3;
new startTimerPumpON = 0;

//---------------------Safety-----------------------
//аварийное раскрытие парашюта
const Float: ERS_ALTITUDE = 500.0;
const Float: ERS_VDOWN = 15.0;
const Float: ERS_ROLL = 50.0;

//максимальная высота раскрытия основного парашюта
const Float: MAIN_PARACHUTE_ALTITUDE = 200.0;

//отцеп парашюта
const Float: RELEASE_ALTITUDE = 15.0;           //[m]
const Float: RALEASE_VDOWN = 0.5;               //[m/sec]
const Float: RELEASE_GSPEED_WINDSPD = 5.0;      //[m/sec]

const Float: GRAVITY_NORM = 9.8;                //[m/sec^2]
const Float: G_MAX = 5.5;                       //[G]

new WAIT_TIME;                                  //[sec] задержка выпуска основного парашюта

new g_LastVspeedReleaseTime;
new g_LastMaxRollInTakeoffModeTime;

new bool:g_mainParachuteLockout;
new bool:g_releaseParachuteLockout;
new bool:g_checkAltitude;
new bool:g_lowAltitudeLockout;
new bool:g_ErsON;

//---------------------SWIWIN-----------------------
//eSWState
#define eSW_DISABLE 0
#define eSW_STOP 1
#define eSW_READY 2
#define eSW_START 3

//eSWUpdate
#define eUPDATE_RATE_20Hz 0
#define eUPDATE_RATE_50Hz 1
#define eUPDATE_RATE_100Hz 2 //2 & 3

#define eCMD_POWER 1
#define eCMD_SHORT 2
#define eCMD_UNLOCK 3
#define eCMD_IGN_PUMP 4
#define eCMD_RPM_ACC 5

#define eCMD_EXHAUST_AIR 1       // Ver:1
#define eCMD_TEST_GLOWPLUG 2     // Ver:1
#define eCMD_TEST_FUEL_VALVE 3   // Ver:1
#define eCMD_TEST_IGNI_VALVE 4   // Ver:1
#define eCMD_TEST_PUMP 5         // Ver:1
#define eCMD_TEST_STARTER 6      // Ver:1
#define eCMD_SEND_RATE_20Hz 7    // Ver:1
#define eCMD_SEND_RATE_50Hz 8    // Ver:1
#define eCMD_SEND_RATE_100Hz 9   // Ver:1
#define eCMD_RESET_FLOW 10       // Ver:2
#define eCMD_CALI_THRUST_ZERO 11 // Ver:2

new m_rev_buf{8} = {};
new m_snd_buf{8} = {};

new m_engine_rpm;

new m_engine_temp;
new m_switch;
new m_engine_status;
new m_error_code;

new m_last_error_code;
new m_last_engine_status;

new m_rc_cur_voltage;
new m_pwr_cur_voltage;
new m_pump_cur_voltage;

new m_throttle;
new m_pressure;

new m_current;
new m_thrust;

new m_pump_ign_voltage;
new m_rpm_acc;

new m_engine_max_rpm;
new m_pump_max_voltage;
new m_version;
new m_update_rate;

new m_flow_rate;
new m_flow_total;

new m_engine_idle_rpm;

new m_ctr_switch = eSW_DISABLE; //0 ~ 3, 1:0ff 2:Ready 3:Start
new m_ctr_throttle = 0;         //0 ~ 100%
new m_set_ign_pump = 0;         //0.10 ~ 5.00v
new m_set_rpm_acc = 0;          //Set RPM ACC: 10 ~ 70
new m_cmd_param = 0;            //Command

new m_set_update_rate = eUPDATE_RATE_20Hz;

new start_eng = false;

//proc time
new g_time_safety;
new g_time_swiwin;
new g_time_fuel;

main()
{
    new now = time();
    g_time_safety = now;
    g_time_swiwin = now;
    g_time_fuel = now;

    //safety
    g_LastVspeedReleaseTime = now;
    g_LastMaxRollInTakeoffModeTime = now;

    g_mainParachuteLockout = false;
    g_releaseParachuteLockout = false;
    g_checkAltitude = false;
    g_lowAltitudeLockout = false;
    g_ErsON = false;
    WAIT_TIME = 2000;

    //high precision mode
    set_var(f_cmode_dlhd, 1.0, true);

    //swiwin
    serial_listen(SWIWIN_PORT_ID, "@OnSerialSWIWIN");

    //fuel
    m_algoritmOld = false;
    m_ignitionOld = false;

    set_var(MANDALA_PUMP1, 0, true);
    set_var(MANDALA_PUMP2, 0, true);
    set_var(MANDALA_PUMP3, 0, true);

    //set_var(f_radar_dx, 15.7, true); //for debug only
    //set_var(f_radar_dy, 15.7, true); //for debug only
    //set_var(f_radar_dz, 15.7, true); //for debug only

    set_var(MANDALA_ALGORITM, 0, true); //0 - auto control, 1 - manual

    serial_listen(PORT_RS485, "@sensorHandler");

}

@OnTask()
{
    new now = time();

    if((now - g_time_safety >= 100) || g_ErsON){
        g_time_safety = now;
        safety_proc();
    }

    if(now - g_time_swiwin >= 300){
        g_time_swiwin = now;
        swiwin_proc();
    }

    if(now - g_time_fuel >= 200){
        g_time_fuel = now;
        fuel_proc();
    }

    return TASK_DELAY_MS;
}

//---------------------Safety-----------------------
//отцеп парашюта
bool: lowVspeedAndAltitudeRelease()
{
    new Float:vspeed = get_var(f_gps_Vdown);
    new Float:altitude = get_var(f_altitude);
    new now = time();
    if((vspeed > RALEASE_VDOWN) || (altitude > RELEASE_ALTITUDE))
        g_LastVspeedReleaseTime = now;
    else if((now - g_LastVspeedReleaseTime) > 0.5 * 1000) //0.5 sec
        return true;

    return false;
}

bool: checkGForceRelease()
{
    new Float:altitude = get_var(f_altitude);
    new Float:nx = get_var(f_Ax) / GRAVITY_NORM;
    new Float:ny = get_var(f_Ay) / GRAVITY_NORM;
    new Float:nz = get_var(f_Az) / GRAVITY_NORM;

    new Float:val = sqrt(nx*nx + ny*ny + nz*nz);

    if(altitude < 10.0 && val > G_MAX){
        printf("VM:G_VAL %f\n", val);
        return true;
    }
    return false;
}

bool: checkTakeOFFSefety()
{
    new bool:is_mode = (get_var(f_mode) == 7) && (get_var(f_stage) > 5);
    new bool:is_roll = abs(get_var(f_roll)) > ERS_ROLL;
    new bool:is_vspeed = (get_var(f_vspeed) < -1.0)
    new now = time();

    if(!is_mode || !is_roll || !is_vspeed)
      g_LastMaxRollInTakeoffModeTime = now;
    else if((now - g_LastMaxRollInTakeoffModeTime) > 1.0 * 1000) //1.0 sec
      return true;

    return false;
}

safety_proc()
{
    //cmode_momag on
    if((get_var(f_cmode_nomag) == 1) && (get_var(f_mode) == 7) && (get_var(f_stage) >= 5)){
        set_var(f_cmode_nomag, 0.0, true);
    }

    new bool:is_vspeed = get_var(f_gps_Vdown) > ERS_VDOWN;
    new bool:is_takeoff = checkTakeOFFSefety();
    //new bool:is_takeoff = false;
    if(get_var(f_altitude) < ERS_ALTITUDE && (is_vspeed || is_takeoff) && !get_var(f_ctrb_drp) && !g_ErsON){
        g_ErsON = true;
        set_var(f_ctrb_ers, 1, true);
        WAIT_TIME = 500;
        if (get_var(f_altitude) < MAIN_PARACHUTE_ALTITUDE)
        {
            WAIT_TIME = 10;
        }
        print("VM:ctrb_ers on\n");
        printf("VM:vspeed %d\n", is_vspeed);
        printf("VM:roll %d\n", is_takeoff);
    }

    if(get_var(f_ctrb_ers) || g_ErsON){
        g_ErsON = true;
        wait(WAIT_TIME); // задержка выпуска основного
        WAIT_TIME = 0;
        if(g_ErsON){
            //выпуск основного парашюта
            new Float: altitude = get_var(f_altitude);
            if(!g_mainParachuteLockout && altitude < MAIN_PARACHUTE_ALTITUDE){
                g_mainParachuteLockout = true;
                set_var(f_userb_8, 1, true);
                print("VM:Main parachute on\n");
            }

            //отцеп парашюта
            new bool: lvs_release = lowVspeedAndAltitudeRelease();
            new bool: gmax_release = checkGForceRelease();
            if(g_mainParachuteLockout && !g_releaseParachuteLockout && (lvs_release || gmax_release) && (altitude < 10.0)){
                g_releaseParachuteLockout = true;
                set_var(f_userb_7, 1, true);
                print("VM:Release parachute ok\n");
                printf("VM:LVS %d\n", lvs_release);
                printf("VM:GMAX %d\n", gmax_release);
            }
            return;
        }
    }

    //проверка высоты и скорости, после которой начинают работать условия
    if(!g_checkAltitude && get_var(f_altitude) > ERS_ALTITUDE && get_var(f_airspeed) > 15.0){
        g_checkAltitude = true;
        print("VM:Start check altitude\n");
    }

    //минимальная безопасная высота выпуска парашюта
    if(g_checkAltitude && !g_lowAltitudeLockout && get_var(f_altitude) < 320.0 && !get_var(f_ctrb_drp)){
        g_lowAltitudeLockout = true;
        WAIT_TIME = 1000;
        g_ErsON = true;
        set_var(f_ctrb_ers, 1, true);
        print("VM:Low altitude. Main parachute on\n");
    }

    //включение power_servo
    if(!get_var(f_power_servo)){
        set_var(f_power_servo, 1, true);
        print("VM:Power servo on\n");
    }

    //полет в красной зоне
    if(get_var(f_userb_5) && get_var(f_VM20) >= 1.0){
        set_var(f_userb_5, 0.0, true);
        set_var(f_mode, 8.0, true);
        print("VM:Start Sefety Landing...\n");
    }

    //ctrb_drop abort
    if((get_var(f_user6) > 1.0) && get_var(f_ctrb_drp) && (get_var(f_mode) == 5) && (get_var(f_altitude) < get_var(f_user6))){
        set_var(f_mode, 3, true);
        print("VM:Low ailtitude, set WPT mode\n");
    }

    //ctrb_drp off
    if(get_var(f_ctrb_drp) && (get_var(f_vspeed) > 0.0) && (get_var(f_mode) == 3) && get_var(f_altitude) > ERS_ALTITUDE){
        set_var(f_ctrb_drp, 0, true);
        print("VM:ctrb_drp off\n");
    }
}

//---------------------SWIWIN-----------------------
new crc_array[] = [
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
    0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
    0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
    0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
    0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
    0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
    0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
    0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
    0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
    0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
    0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
    0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
];

calc_crc8(buf{}, start, crc_len)
{
    new crc8 = 0;
    for(new i = start ; i < crc_len + start; i++) {
        crc8 = crc_array[crc8^buf{i}];
    }
    return crc8;
}

//Set State for RC Switch
//Input: eSW_STOP, eSW_READY, eSW_START
set_switch(sw)
{
    m_ctr_switch = sw;
}

//Set Valve for RC Throttle
//Input: 0 ~ 1000
set_throttle(val)
{
    if(val > 1000) {
        val = 1000;
    }
    m_ctr_throttle = val;
}

EcuPrintError(id)
{
    switch(id) {
    case 0: { printf("VM:error_null...\n"); }
    case 1: { printf("VM:error_timout...\n"); }
    case 2: { printf("VM:error_low_vbat...\n"); }
    case 3: { printf("VM:error_glowplug...\n"); }
    case 4: { printf("VM:error_pump...\n"); }
    case 5: { printf("VM:error_starter...\n"); }
    case 6: { printf("VM:error_rpm_low...\n"); }
    case 7: { printf("VM:error_rpm_instability...\n"); }
    case 8: { printf("VM:error_temp_high...\n"); }
    case 9: { printf("VM:error_temp_low...\n"); }
    case 10: { printf("VM:error_temp_sensor...\n"); }
    case 11: { printf("VM:error_valve_s...\n"); }
    case 12: { printf("VM:error_valve_m...\n"); }
    case 13: { printf("VM:error_lost_radio...\n"); }
    case 14: { printf("VM:error_starter_ctl_htmp...\n"); }
    case 15: { printf("VM:error_pump_ctl_htmp...\n"); }
    case 16: { printf("VM:error_clutch_error...\n"); }
    case 17: { printf("VM:error_current_overload...\n"); }
    case 18: { printf("VM:error_engine_offline...\n"); }
    }
}

EcuPrintStatus(id)
{
    switch(id) {
    case 0: { printf("VM:stop...\n"); }
    case 1: { printf("VM:ready...\n"); }
    case 2: { printf("VM:ign_stick_down...\n"); }
    case 3: { printf("VM:ign_run...\n"); }
    case 4: { printf("VM:preheat...\n"); }
    case 5: { printf("VM:fuelramp...\n"); }
    case 6: { printf("VM:run_learn_stick_max...\n"); }
    case 7: { printf("VM:run_learn_stick_min...\n"); }
    case 8: { printf("VM:run_learn_rc...\n"); }
    case 9: { printf("VM:run_stick_min...\n"); }
    case 10: { printf("VM:run_pump_limit...\n"); }
    case 11: { printf("VM:running...\n"); }
    case 12: { printf("VM:cooling...\n"); }
    case 13: { printf("VM:restart...\n"); }
    case 14: { printf("VM:test_glowplug...\n"); }
    case 15: { printf("VM:test_valve_main...\n"); }
    case 16: { printf("VM:test_valve_ignite...\n"); }
    case 17: { printf("VM:test_oilpump...\n"); }
    case 18: { printf("VM:test_starter...\n"); }
    case 19: { printf("VM:exhaust_air...\n"); }
    }
}

state_comm()
{
    new rpm = 0;

    //Engine RPM
    rpm = (((m_rev_buf{1}&0xFF)) << 0) | (((m_rev_buf{2}&0xFF)) << 8);
    m_engine_rpm = rpm * 10;
}

state_id_state1()
{
    new tmp = 0;
    new err_code = 0;

    err_code = ((m_rev_buf{3}&0xE0)>>5) | ((m_rev_buf{4}&0x03)<<3);

    m_engine_status = (m_rev_buf{3} & 0x1F);
    m_error_code    = (err_code & 0x1F);

    //Engine Temperature
    tmp = (((m_rev_buf{4}&0x1C)) << 6) | (((m_rev_buf{5}&0xFF)) << 0);
    m_engine_temp = tmp - 50;

    m_switch = ((m_rev_buf{4}&0x60) >> 5);
}

state_id_state2()
{
    m_rc_cur_voltage   = m_rev_buf{3};
    m_pwr_cur_voltage  = m_rev_buf{4};
    m_pump_cur_voltage = m_rev_buf{5};
}

state_id_state3()
{
    new  press = 0;

    m_throttle = m_rev_buf{3};
    if (m_throttle > 100){
        m_throttle = 100;
    }

    press = ((m_rev_buf{4}) << 0) | ((m_rev_buf{5}) << 8);
    m_pressure = press * 2;
}

state_id_state4()
{
    new temp = 0;

    temp = ((m_rev_buf{3}) << 0) | ((m_rev_buf{4}) << 8);
    m_current = temp & 0x1FF;

    temp = ((m_rev_buf{5}) << 0) | (((m_rev_buf{4}&0xFE)) << 7);
    m_thrust = temp & 0x7FFF;
}

state_id_state5()
{
    m_pump_ign_voltage = (m_rev_buf{3}) * 2;    //0.10 ~  5.00v
    m_rpm_acc = m_rev_buf{4};                   //10 ~ 70

    if (m_rpm_acc < 10) m_rpm_acc = 10;
    if (m_rpm_acc > 70) m_rpm_acc = 70;
}

state_id_state6()
{
    m_engine_max_rpm   = (m_rev_buf{3}) * 1000;
    m_pump_max_voltage = m_rev_buf{4};
    m_version          = (m_rev_buf{5} >> 2) & 0x3F;
    m_update_rate      = (m_rev_buf{5} >> 0) & 0x03;
}

state_id_state7()
{
    new temp = 0;
    temp = ((m_rev_buf{3}) << 0) | ((m_rev_buf{4}) << 8);
    m_flow_rate  = temp & 0x3FF;

    temp = ((m_rev_buf{4}) >> 2) | ((m_rev_buf{5}) << 6);
    m_flow_total = temp & 0x3FFF;
}

state_id_state8()
{
    m_engine_idle_rpm  = (m_rev_buf{3}) * 1000;
}

print_and_save_param()
{
    if(m_error_code != m_last_error_code){
        EcuPrintError(m_error_code);
        m_last_error_code = m_error_code;
    }

    if(m_engine_status != m_last_engine_status){
        EcuPrintStatus(m_engine_status);
        m_last_engine_status = m_engine_status;
    }

    if(m_engine_rpm < 4000) {
        set_var(f_rpm, 0, true);
    } else {
        set_var(f_rpm, m_engine_rpm / 10, true);
    }
    set_var(f_EGT, m_engine_temp, true);
    set_var(f_user5, m_throttle, true);
    set_var(f_Vm, m_pwr_cur_voltage / 10.0, true);
}

get_send_buf()
{
    m_snd_buf{0} = 0xFF; //Header
    m_snd_buf{4} = 0xFF;

    if (m_set_ign_pump){
        //Unlock
        m_snd_buf{1} = eCMD_UNLOCK << 4;
        m_snd_buf{2} = 0x00;
        m_snd_buf{3} = calc_crc8(m_snd_buf, 1, 2);
        //Set Param: Ignition Pump Voltage
        m_snd_buf{5} = eCMD_IGN_PUMP << 4;
        m_snd_buf{6} = m_set_ign_pump;
        m_snd_buf{7} = calc_crc8(m_snd_buf, 5, 2);
        m_set_ign_pump = 0;
        return 8;
    } else if (m_set_rpm_acc){
        //Unlock
        m_snd_buf{1} = eCMD_UNLOCK << 4;
        m_snd_buf{2} = 0x00;
        m_snd_buf{3} = calc_crc8(m_snd_buf, 1, 2);
        //Set Param: RPM ACC
        m_snd_buf{5} = eCMD_RPM_ACC << 4;
        m_snd_buf{6} = m_set_rpm_acc;
        m_snd_buf{7} = calc_crc8(m_snd_buf, 5, 2);
        m_set_rpm_acc = 0;
        return 8;
    } else{

        //Send Control(Switch & Throttle)
        m_snd_buf{1} = eCMD_POWER << 4;
        m_snd_buf{1} |= (m_ctr_switch & 0x03) << 2;
        m_snd_buf{1} |= ((m_ctr_throttle & 0x300) >> 8);
        m_snd_buf{2} = ((m_ctr_throttle & 0x0FF) >> 0);
        m_snd_buf{3} = calc_crc8(m_snd_buf, 1, 2);

        if (m_update_rate != m_set_update_rate && m_version >= 1){
            switch (m_set_update_rate){
            case eUPDATE_RATE_20Hz:
                m_snd_buf{6} = eCMD_SEND_RATE_20Hz;
            case eUPDATE_RATE_50Hz:
                m_snd_buf{6} = eCMD_SEND_RATE_50Hz;
            case eUPDATE_RATE_100Hz:
                m_snd_buf{6} = eCMD_SEND_RATE_100Hz;
            }
            m_snd_buf{5} = eCMD_SHORT << 4;
            m_snd_buf{7} = calc_crc8(m_snd_buf, 5, 2);
            m_cmd_param = 0;
            return 8;
        }
        //Send Short Command
        else if (m_cmd_param){
            m_snd_buf{5} = eCMD_SHORT << 4;
            m_snd_buf{6} = m_cmd_param;
            m_snd_buf{7} = calc_crc8(m_snd_buf, 5, 2);
            m_cmd_param = 0;
            return 8;
        }
        return 4;
    }
}

forward @OnSerialSWIWIN(cnt)
@OnSerialSWIWIN(cnt)
{
    if(cnt != SWIWIN_DATA_SIZE){
        return;
    }

    for(new i = 0; i < cnt; i++){
        m_rev_buf{i} = serial_byte(i);
    }

    if(calc_crc8(m_rev_buf, 0, SWIWIN_DATA_SIZE-1) == m_rev_buf{SWIWIN_DATA_SIZE-1}){
        state_comm();
        switch(m_rev_buf{0} & 0x0F){
            case 1: state_id_state1();
            case 2: state_id_state2();
            case 3: state_id_state3();
            case 4: state_id_state4();
            case 5: state_id_state5();
            case 6: state_id_state6();
            case 7: state_id_state7();
            case 8: state_id_state8();
        }
        print_and_save_param();
    }
}

swiwin_proc()
{
    new on_power_ignition = get_var(f_power_ignition) > 0.0;

    //start
    if(on_power_ignition && get_var(f_sw_starter) > 0.0){
        set_switch(eSW_START);
        set_var(f_sw_starter, 0.0, true);
        start_eng = true;
    }

    //ready
    if(on_power_ignition && m_switch != eSW_START && !start_eng){
        set_switch(eSW_READY);
    }

    //stop and ctr_throttle
    if(on_power_ignition){
        set_throttle(get_var(f_ctr_throttle) * 1000);
    } else{
        set_switch(eSW_STOP);
        start_eng = false;
    }

    //cooling
    if(m_engine_temp > 100.0 && !start_eng){
        set_switch(eSW_READY);
        set_var(f_power_ignition, 1.0, true);
    }


    new len = get_send_buf();
    if(len){
        serial_write(SWIWIN_PORT_ID, m_snd_buf, len, serialmode:NODE);
    }
}

//--------------------- FUEL -----------------------
turn_off_all_pumps()
{
  set_var(MANDALA_PUMP1, 0, true);
  set_var(MANDALA_PUMP2, 0, true);
  set_var(MANDALA_PUMP3, 0, true);
}

turn_on_all_pumps()
{
  set_var(MANDALA_PUMP1, 1, true);
  set_var(MANDALA_PUMP2, 1, true);
  set_var(MANDALA_PUMP3, 1, true);
}

turn_on_pump_1()
{
  if(time() > startTimerPumpON + DELAY_PUMP) {
    if(m_pump2 || m_pump3) {
      set_var(MANDALA_PUMP2, 0, true);
      set_var(MANDALA_PUMP3, 0, true);
      return; //wait 200ms till the next iteration
    }
    if(!m_pump1) {
      set_var(MANDALA_PUMP1, 1, true);
      startTimerPumpON = time();
    }
  }
}
turn_on_pump_2()
{
  if(time() > startTimerPumpON + DELAY_PUMP) {
    if(m_pump1 || m_pump3) {
      set_var(MANDALA_PUMP1, 0, true);
      set_var(MANDALA_PUMP3, 0, true);
      return; //wait 200ms till the next iteration
    }
    if(!m_pump2) {
      set_var(MANDALA_PUMP2, 1, true);
      startTimerPumpON = time();
    }
  }
}
turn_on_pump_3()
{
  if(time() > startTimerPumpON + DELAY_PUMP) {
    if(m_pump1 || m_pump2) {
      set_var(MANDALA_PUMP1, 0, true);
      set_var(MANDALA_PUMP2, 0, true);
      return; //wait 200ms till the next iteration
    }
    if(!m_pump3) {
      set_var(MANDALA_PUMP3, 1, true);
      startTimerPumpON = time();
    }
  }
}

pump_stage_1() //get fuel from tank 3 untill the point
{
  if(m_vFuelPersent3 > TANK_3_POINT)
    turn_on_pump_3();
  else {
      pump_stage = 2;
      printf("fuel stage: 2");
  }
}
pump_stage_2() //get fuel from tank 3 untill empty and from 1 untill the point
{
  if(m_vFuelPersent1 > TANK_1_POINT){
    if(m_vFuelPersent3 > CRITICAL_LOW){
      if(m_vFuelPersent1 > (m_vFuelPersent3 + 100 - TANK_3_POINT)){
        turn_on_pump_1();
      } else {
        turn_on_pump_3();
      }
    } else{
      turn_on_pump_1();
    }
  } else{
    pump_stage = 3;
    printf("fuel stage: 3");
  }
}
pump_stage_3() //get fuel from tank 2 untill specified points
{
  if(m_vFuelPersent3 < CRITICAL_LOW){
    if(m_vFuelPersent2 > TANK_2_POINT){
      turn_on_pump_2();
    } else {
      pump_stage = 4;
      printf("fuel stage: 4");
    }
  } else {
    turn_on_pump_3();
  }
}
pump_stage_4() //get fuel from tank 1 and 2 untill empty
{
  if(m_vFuelPersent3 < CRITICAL_LOW){
    if(m_vFuelPersent1 > CRITICAL_LOW){
      if(m_vFuelPersent2 > CRITICAL_LOW){
        if(m_vFuelPersent1 > m_vFuelPersent2){
          turn_on_pump_1();
        } else {
          turn_on_pump_2();
        }
      } else {
        turn_on_pump_1();
      }
    } else if(m_vFuelPersent2 > CRITICAL_LOW){
      turn_on_pump_2();
    } else {
      turn_on_all_pumps();
    }
  } else {
    turn_on_pump_3();
  }
}

fuel_auto_control()
{
    switch (pump_stage) {
      case 1: pump_stage_1();
      case 2: pump_stage_2();
      case 3: pump_stage_3();
      case 4: pump_stage_4();
    }

}

fuel_manual_control()
{
 // nothing is needed to be implemented
}

calcCrc(buf{}, size)
{
    new crc = 0x00;
    new j = 0;
    for(j = 0; j < size; j++){
        new i = buf{j} ^ crc;
        crc = 0;
        if (i & 0x01) crc ^= 0x5e;
        if (i & 0x02) crc ^= 0xbc;
        if (i & 0x04) crc ^= 0x61;
        if (i & 0x08) crc ^= 0xc2;
        if (i & 0x10) crc ^= 0x9d;
        if (i & 0x20) crc ^= 0x23;
        if (i & 0x40) crc ^= 0x46;
        if (i & 0x80) crc ^= 0x8c;
    }
    return crc%256;
}

check_answer_time()
{
    //check answer time from sensor
    if(m_timeAns1+TIME_SA*1000 < time())
      set_var(MANDALA_WARN_ANSWER1, 1, true);
    else
      set_var(MANDALA_WARN_ANSWER1, 0, true);

    if(m_timeAns2+TIME_SA*1000 < time())
      set_var(MANDALA_WARN_ANSWER2, 1, true);
    else
      set_var(MANDALA_WARN_ANSWER2, 0, true);

    if(m_timeAns3+TIME_SA*1000 < time())
      set_var(MANDALA_WARN_ANSWER3, 1, true);
    else
      set_var(MANDALA_WARN_ANSWER3, 0, true);

}


forward @sensorHandler(cnt);
@sensorHandler(cnt)
{
    if(cnt == 9){   //typePack(0)[0x3E] adr(1) fmt(2) data(3) crc(8)
        if(serial_byte(0) == 0x3E && serial_byte(2) == 0x06) {
            new data{9};
            for(new i = 0; i < PACK_SIZE; i++)
                data{i} = serial_byte(i);

            if(data{8} == calcCrc(data, 8)) {

                if(data{1} == ADR_SENS1) {
                    m_timeAns1 = time();
                    m_vFuelPersent1 = (data{4} | (data{5} << 8)) / 10.0;
                    new Float: m_vFuel1 = m_vFuelPersent1 * V_MAX1 / 100.0;
                    set_var(MANDALA_FUEL_V1, m_vFuel1, true);
                    //printf("sens1: %f\r\n", m_vFuel1);
                }

                if(data{1} == ADR_SENS2) {
                    m_timeAns2 = time();
                    m_vFuelPersent2 = (data{4} | (data{5} << 8)) / 10.0;
                    new Float: m_vFuel2 = m_vFuelPersent2 * V_MAX2 / 100.0;
                    set_var(MANDALA_FUEL_V2, m_vFuel2, true);
                    //printf("sens2: %f\r\n", m_vFuel2);
                }

                if(data{1} == ADR_SENS3) {
                    m_timeAns3 = time();
                    m_vFuelPersent3 = (data{4} | (data{5} << 8)) / 10.0;
                    new Float: m_vFuel3 = m_vFuelPersent3 * V_MAX3 / 100.0;
                    set_var(MANDALA_FUEL_V3, m_vFuel3, true);
                    //printf("sens3: %f\r\n", m_vFuel3);
                }
            }
        }
    }
}


fuel_proc()
{
//=========  test fuel sensor  ======================== //for debug only
/*    new Float: ctr_thr = get_var(f_rc_throttle);
    if(ctr_thr < 0.01)
      ctr_thr = 0.01;
    if(m_pump1) {
      set_var(f_radar_dx, get_var(f_radar_dx) - 0.05 * ctr_thr, true);
      if(get_var(f_radar_dx) < 0.1)
      set_var(f_radar_dx, 0.1, true);
    }
    if(m_pump2) {
      set_var(f_radar_dy, get_var(f_radar_dy) - 0.05 * ctr_thr, true);
      if(get_var(f_radar_dy) < 0.1)
      set_var(f_radar_dy, 0.1, true);
    }
    if(m_pump3) {
      set_var(f_radar_dz, get_var(f_radar_dz) - 0.05 * ctr_thr, true);
      if(get_var(f_radar_dz) < 0.1)
      set_var(f_radar_dz, 0.1, true);
    }
    if(m_vFuelPersent3 < 100 && (m_pump1 || m_pump2 || m_pump3))
      set_var(f_radar_dz, get_var(f_radar_dz) + 0.01 * ctr_thr, true); */
//=======================================================

    check_answer_time();

    new Float: fl1 = get_var(MANDALA_FUEL_V1);
    new Float: fl2 = get_var(MANDALA_FUEL_V2);
    new Float: fl3 = get_var(MANDALA_FUEL_V3);
    set_var(MANDALA_FUEL, fl1 + fl2 + fl3, true);

    //m_vFuelPersent1 = fl1 * 100 / V_MAX1; //for debug only
    //m_vFuelPersent2 = fl2 * 100 / V_MAX2; //for debug only
    //m_vFuelPersent3 = fl3 * 100 / V_MAX3; //for debug only

    m_ignition = get_var(MANDALA_IGNITION);
    m_algoritm = get_var(MANDALA_ALGORITM);
    m_ers = get_var(MANDALA_ERS);
    m_pump1 = get_var(MANDALA_PUMP1);
    m_pump2 = get_var(MANDALA_PUMP2);
    m_pump3 = get_var(MANDALA_PUMP3);

    if (m_ers){
      turn_off_all_pumps();
    }else{

      if(m_ignition != m_ignitionOld){
        m_ignitionOld = m_ignition;
        if(!m_ignition){
          turn_off_all_pumps();
        }
      }

      if(m_algoritm != m_algoritmOld){
          m_algoritmOld = m_algoritm;
          if(m_algoritm){
            printf("manual fuel ON");
          } else {
            turn_off_all_pumps();
            printf("manual fuel OFF");
          }
      }

      if(!m_algoritm && m_ignition){
        fuel_auto_control();
      }else{
        fuel_manual_control();
      }
    }

    msg{0} = 0x31;
    msg{2} = 0x06;
    msg{1} = ADR_SENS1 + m_sensRequest; //choose sensor
    msg{3} = calcCrc(msg, 3);
    serial_write(PORT_RS485, msg, 4, serialmode:NODE);

    m_sensRequest++;
    if(m_sensRequest == 3)
      m_sensRequest = 0;
}
