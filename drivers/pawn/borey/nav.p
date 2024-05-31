//Borey NAV R4, version 1.0, 31.05.2024

const TASK_DELAY_MS = 10;

//ports id
const LOOKHERE_PORT_ID = 123;
const ESC_PORT_ID = 50;
const AGL_PORT_ID = 13;

//data size
const LOOKHERE_DATA_SIZE = 8;
const ESC_DATA_SIZE = 10;
const AGL_DATA_SIZE = 13;

//---------------------ESC-----------------------
new esc_rx_buffer{16} = {};

new m_temperature = 0;
new m_voltage = 0;
new m_current = 0;
new m_consumption = 0;
new m_rpm = 0;

//---------------------AGL-----------------------
#define AGL_CAN_ID  0x00090002
new idx_agl = f_agl;

//-------------------Safety----------------------
new bool:g_releaseParachute;
new idx_releaseParachute = f_userb_8;
new idx_cmdReleaseParachute = f_sw_sw1;

//---------------------TMP-----------------------
const TABLE_SIZE = 43;
const Float: V_CC = 2.998;
const Float: R_PUP = 3160.0;

new Float:TableTemp[] = [
    -55.0, -50.0, -45.0, -40.0, -35.0,
    -30.0, -25.0, -20.0, -15.0, -10.0,
     -5.0,   0.0,   5.0,  10.0,  15.0,
     20.0,  25.0,  30.0,  35.0,  40.0,
     45.0,  50.0,  55.0,  60.0,  65.0,
     70.0,  75.0,  80.0,  85.0,  90.0,
     95.0, 100.0, 105.0, 110.0, 115.0,
    120.0, 125.0, 130.0, 135.0, 140.0,
    145.0, 150.0, 155.0
];

new Float:TableRes[] = [
    963000.0, 670100.0, 471700.0, 336500.0, 242600.0,
    177000.0, 130400.0,  97070.0,  72930.0,  55330.0,
     42320.0,  32650.0,  25390.0,  19900.0,  15710.0,
     12490.0,  10000.0,   8057.0,   6531.0,   5327.0,
      4369.0,   3603.0,   2986.0,   2488.0,   2083.0,
      1752.0,   1481.0,   1258.0,   1072.0,    917.7,
       788.5,    680.0,    588.6,    511.2,    445.4,
       389.3,    341.7,    300.9,    265.4,    234.8,
       208.3,    185.3,    165.3
];

new idx_raw_volt = f_VM30;
new idx_temp = f_radar_Vz;

new Float: v_t = 0.0;
new Float: r_t = 0.0;
new Float: i_t = 0.0;

new Float: temp = 0.0;
new Float: persent = 0.0;

//-----------------------------------------------
new g_time_look_here;
new g_time_common;
new g_time_safety;

packFloat(data{}, index, Float:v)
{
    for(new i = 0; i < 4; i++)
        data{index + i} = (v >> (8 * i)) & 0xFF;
}

sendLookHereCommands()
{
    new data{LOOKHERE_DATA_SIZE};

    packFloat(data, 0, get_var(f_camctr_yaw));
    packFloat(data, 4, get_var(f_camctr_pitch));

    new bool:result = serial_write(LOOKHERE_PORT_ID, data, LOOKHERE_DATA_SIZE, serialmode:NODE);
    if(!result)
        printf("sendLookHereCommands error")
}

save_data_to_mandala()
{
    //ESC
    set_var(f_ET, m_temperature, true);
    set_var(f_Vm, m_voltage / 100.0, true);
    set_var(f_user3, m_current / 100.0, true);  // not supported in Flycolor
    set_var(f_user4, m_consumption, true);      // not supported in Flycolor
    set_var(f_rpm, m_rpm * 100.0 / 7, true);

    //Temperature
    set_var(idx_temp, temp, true);
}

calc_temperature()
{
    new indexL = 0;
    new indexH = 0;
    v_t = get_var(idx_raw_volt);
    i_t = (V_CC-v_t)/R_PUP;

    if((v_t/963000.0)<i_t){
        r_t = v_t/i_t
    }else{
        r_t = 963000;
    }

    for(new i = 1; i < TABLE_SIZE; i++){
        if(r_t <= TableRes[i-1] && r_t >= TableRes[i]){
            indexH = i-1;
            indexL = i;
            break;
        }
    }

    if(indexL != indexH){
        persent = (r_t - TableRes[indexL]) / (TableRes[indexH] - TableRes[indexL]);
        temp = TableTemp[indexL] - persent*(TableTemp[indexL] - TableTemp[indexH]);
    }
}

safety_algorithm()
{
    new mode = get_var(f_mode);
    new bool: rel_ok = (get_var(f_ctrb_ers) && (mode == 8) && (get_var(f_vspeed) > -3)) || (mode == 0) || (mode == 6);

    if(!g_releaseParachute && get_var(idx_cmdReleaseParachute) && rel_ok){
        g_releaseParachute = true;
        set_var(idx_releaseParachute, 1.0, true);
        print("VM:Release on...\n");
    }else if (!g_releaseParachute && get_var(idx_cmdReleaseParachute)){
        set_var(idx_cmdReleaseParachute, 0.0, true);
        print("VM:Release block...\n");
    }else if (g_releaseParachute && !get_var(idx_cmdReleaseParachute)){
        g_releaseParachute = false;
        set_var(idx_releaseParachute, 0.0, true);
        print("VM:Release off...\n");
    }
}

main()
{
    new now = time();

    g_time_look_here = now;
    g_time_common = now;
    g_time_safety = now;

    g_releaseParachute = false;
    set_var(idx_releaseParachute, 0.0, true);
    set_var(f_sw_sw1, 0.0, true);

    serial_listen(ESC_PORT_ID, "@OnSerial_ESC");
    serial_listen(AGL_PORT_ID, "@OnSerial_AGL");
}

@OnTask()
{
    new now = time();

    if(now - g_time_look_here >= 50){
        g_time_look_here = now;
        if(get_var(f_cam_mode) == 4){
            sendLookHereCommands();
        }
    }

    if(now - g_time_common >= 200){
        g_time_common = now;

        calc_temperature();
        save_data_to_mandala();
    }

    if(now - g_time_safety >= 500){
        g_time_safety = now;
        safety_algorithm();
    }

    return TASK_DELAY_MS;
}

update_crc8(crc, crc_seed)
{
    new crc_u;
    crc_u = crc;
    crc_u ^= crc_seed;
    for(new i=0; i<8; i++)
        crc_u = (crc_u & 0x80) ? 0x7 ^ (crc_u << 1) : (crc_u << 1);
    return crc_u;
}

get_crc8(buf{}, crc_len)
{
    new crc = 0;
    for(new i=0; i<crc_len; i++)
        crc = update_crc8(buf{i}, crc);
    return crc & 0x000000ff;
}

forward @OnSerial_ESC(cnt)
@OnSerial_ESC(cnt)
{
    if(cnt != ESC_DATA_SIZE){
        return;
    }

    new frame_length = 0;

    for(new i = 0; i<cnt; i++) {
        esc_rx_buffer{frame_length++} = serial_byte(i);
    }

    if(get_crc8(esc_rx_buffer, ESC_DATA_SIZE-1) == esc_rx_buffer{ESC_DATA_SIZE-1}){
        m_temperature = serial_byte(0);
        m_voltage = (serial_byte(1) << 8) + serial_byte(2);
        m_current = (serial_byte(3) << 8) + serial_byte(4);
        m_consumption = (serial_byte(5) << 8) + serial_byte(6);
        m_rpm = (serial_byte(7) << 8) + serial_byte(8);
    }
}

forward @OnSerial_AGL(cnt)
@OnSerial_AGL(cnt)
{
    if(cnt!= AGL_DATA_SIZE){
        return
    }

    new can_id = serial_byte(0)
                + (serial_byte(1) << 8)
                + (serial_byte(2) << 16)
                + (serial_byte(3) << 24);


    if(can_id == AGL_CAN_ID){
    
        //TODO add filter for agl data
        new Float: agl = ((serial_byte(5)<<8) + serial_byte(6)) / 100.0;
        if(agl > 0.1 && agl < 50.0){
            set_var(idx_agl, agl, true);
        }
    }
}
