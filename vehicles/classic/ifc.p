#define ORDER_LITTLE_ENDIAN 0
#define ORDER_BIG_ENDIAN    1

#define PORT_ID_AUX         20
#define CAN_PACK_SIZE       13

//====================vesc====================
new idx_vesc_rpm =      f_VM1;
new idx_vesc_crt =      f_VM2;
new idx_vesc_dc =       f_VM3;
new idx_vesc_ft =       f_VM4;
new idx_vesc_mt =       f_VM5;
new idx_vesc_crtin =    f_VM6;

new Float: vesc_rpm =   0.0;
new Float: vesc_crt =   0.0;
new Float: vesc_dc =    0.0;
new Float: vesc_ft =    0.0;
new Float: vesc_mt =    0.0;
new Float: vesc_crtin = 0.0;

//====================mcell====================
#define MCELL_ID        256
#define MCELL_PACK1     MCELL_ID + 1
#define MCELL_PACK2     MCELL_ID + 2
#define MCELL_PACK3     MCELL_ID + 3
#define MCELL_PACK4     MCELL_ID + 4

new mcell_vbat =    0;
new mcell_tbat =    0;
new mcell_tpcb =    0;
new mcell_status =  0;
new cell[6] =       [0,];

new Float: mcell_min = 0.0;
new Float: mcell_max = 0.0;

//new mcell_upd =     false;

//====================uvhpu====================
#define UVH_PU_ID       128
#define UVH_PU_PACK1    UVH_PU_ID + 1
#define UVH_PU_PACK2    UVH_PU_ID + 2
#define UVH_PU_PACK3    UVH_PU_ID + 3
#define UVH_PU_PACK4    UVH_PU_ID + 4
#define UVH_PU_PACK5    UVH_PU_ID + 5
#define UVH_PU_PACK6    UVH_PU_ID + 6
#define UVH_PU_PACK7    UVH_PU_ID + 7

#define UVH_CMD_HEATER    UVH_PU_ID + 0x0A
#define UVH_CMD_THROTTLE  UVH_PU_ID + 0x0B

new Float: uvhpu_vbat =         0.0;
new Float: uvhpu_ibat =         0.0;
new Float: uvhpu_imon =         0.0;

new Float: uvhpu_vout =         0.0;
new Float: uvhpu_tbat =         0.0;
new Float: uvhpu_pbat =         0.0;
new uvhpu_status =              0;

new Float: uvhpu_cbat =         0.0;
new Float: uvhpu_ebat =         0.0;

new Float: uvhpu_res_bat =      0.0;
new Float: uvhpu_v_res =        0.0;

new Float: uvhpu_ibat_filt =    0.0;
new Float: uvhpu_vbat_filt =    0.0;

new Float: uvhpu_cbat_res =     0.0;
new Float: uvhpu_ebat_res =     0.0;

new uvhpu_life_cycles =         0;
new Float: uvhpu_cbat_mod =     0.0;
new Float: uvhpu_h_pwr =        0.0;

//new uvhpu_upd =                 false;

//====================mppt====================
#define CANAS_POWER_ON       1500
#define CANAS_SAVE_POWER_ON  1550

#define CANAS_KMPPT_ADDR       1510
#define CANAS_SAVE_KMPPT_ADDR  1560

#define CANAS_VOLT_IN   1600
#define CANAS_VOLT_OUT  1601
#define CANAS_CURR_IN   1602
#define CANAS_CURR_OUT  1603
#define CANAS_TEMP      1604

#define k_mppt 0.7
#define number_mppt 1
new Float:KMPPT_ARRAY[number_mppt] = [k_mppt,];

new bool: mppt_power_state;

new Float: mppt_in_volt;
new Float: mppt_out_volt;
new Float: mppt_out_curr;
new Float: mppt_in_curr;
new Float: mppt_temp;

//============================================

const mppt_timeout = 100;
const cmd_timeout =  1000;
const vesc_timeout = 100;
const save_timeout = 200;

new mppt_timer = 0;
new cmd_timer =  0;
new vesc_timer = 0;
new save_timer = 0;
//============================================
/*
toChar(b0)
{
    if (b0 & 0x80)
        b0 = (-1)*(0x100 - b0);
    return b0;
}
*/

toShort(data{}, index, order)
{
    new val;

    if (order == ORDER_BIG_ENDIAN)
        val = data{index+1} + (data{index} << 8);
    else
        val = data{index} + (data{index+1} << 8);

    if (val & 0x8000)
        val = (-1)*(0x10000 - val);

    return val;
}

toUShort(data{}, index, order)
{
    if (order == ORDER_BIG_ENDIAN)
        return data{index+1} + (data{index} << 8);
    else
        return data{index} + (data{index+1} << 8);
}

Float:toFloat(data{}, index, order)
{
    new Float:value[1];
    if (order == ORDER_BIG_ENDIAN) {
        memcpy(value, data, 0, 4);
    }
    else {
        new temp{4};
        temp{3} = data{index++};
        temp{2} = data{index++};
        temp{1} = data{index++};
        temp{0} = data{index++};
        memcpy(value, temp, 0, 4);
    }

    return value[0];
}

serializeFloat(data{}, index, Float: value)
{
  for(new i = 0; i < 4; i++) {
    new shift = 8 * i;
    data{index + i} = (value >> shift) & 0xFF;
  }
}
/*
toInt(data{}, index, order)
{
    new value[1];
    if (order == ORDER_BIG_ENDIAN) {
        memcpy(value, data, 0, 4);
    }
    else {
        new temp{4};
        temp{0} = data{index + 3};
        temp{1} = data{index + 2};
        temp{2} = data{index + 1};
        temp{3} = data{index};
        memcpy(value, temp, 0, 4);
    }

    return value[0];
}
*/

sendCmdToCan(id, cmd{}, n)
{
    new msg{CAN_PACK_SIZE};

    msg{0} = id;           //ID_0_7
    msg{1} = id >> 8;      //ID_8_15
    msg{2} = id >> 16;     //ID_16_23
    msg{3} = id >> 24;     //ID_24_31
    msg{4} = n;            //IDE (bit 7): 1=ext,0=std; DLC (bit 0-3): data length
    if (id > 0x7FF)
        msg{4} = msg{4} | 0x80;

    for(new i = 0 ; i < n; i++) {
        msg{5+i} = cmd{i};
    }
    serial_write(PORT_ID_AUX, msg,  5 + n, serialmode:NODE);
}
//------------------------------------------------------------------------------------------
main()
{
    wait (100);
    serial_listen(PORT_ID_AUX, "@on_can_aux");

    mppt_power_state = false;

    new now = time();

    mppt_timer = now;
    cmd_timer = now;
    vesc_timer = now;
    save_timer = now;
}

@OnTask()
{
    new now = time();

    //mppt power on/off
    new bool: m_power_ignition = (get_var(f_power_ignition) > 0);
    if (mppt_power_state != m_power_ignition) {
        mppt_power_state = m_power_ignition;
        for(new i=1; i<number_mppt+1; i++) {
            mppt_power_on(CANAS_POWER_ON + i, mppt_power_state)
        }
    }

    //mppt configuration
    if (now - mppt_timer > mppt_timeout) {
        mppt_timer = now;

        //new n_mppt = get_var(f_cam_pitch);
        new n_mppt = 1.0;
        if (n_mppt>=1.0 && n_mppt<=5.0) {
        new Float: new_k_mppt = get_var(f_cam_roll);
            if (KMPPT_ARRAY[n_mppt-1] != new_k_mppt) {
                KMPPT_ARRAY[n_mppt-1] = new_k_mppt;
                mppt_set_k(CANAS_KMPPT_ADDR + n_mppt, new_k_mppt);
           }
        }

    }

    if (now - cmd_timer > cmd_timeout) {
        cmd_timer = now;


        new msg{8};
        //new hHeater = get_var(f_platform_Vdown) * 100.0;
        //msg{0} = hHeater;
        //msg{1} = hHeater >> 8;
        //sendCmdToCan(UVH_CMD_HEATER, msg, 2);


        new throttle = get_var(f_platform_Vdown) * 100.0;
        msg{0} = throttle;
        msg{1} = throttle >> 8;
        sendCmdToCan(UVH_CMD_THROTTLE, msg, 2);

    }

    if (now - vesc_timer > vesc_timeout) {
        vesc_timer = now;
        forwardVESCPackage();
    }

    if (now - save_timer > save_timeout) {
        save_timer = now;
        save_data_to_mandala();
    }

    return 20;
}
//------------------------------------------------------------------------------------------
forwardVESCPackage()
{
    vesc_rpm = get_var(idx_vesc_rpm);
    vesc_crt = get_var(idx_vesc_crt);
    vesc_dc = get_var(idx_vesc_dc);
    vesc_ft = get_var(idx_vesc_ft);
    vesc_mt = get_var(idx_vesc_mt);
    vesc_crtin = get_var(idx_vesc_crtin);
}

forward @vesc()
@vesc()
{
    printf("rpm: %.2f\n", vesc_rpm);
    printf("crt: %.2f\n", vesc_crt);
    printf("dt: %.2f\n", vesc_dc);
    printf("ft: %.2f\n", vesc_ft);
    printf("mt: %.2f\n", vesc_mt);
    printf("crtin: %.2f\n", vesc_crtin);
    printf("---\n");
}
//------------------------------------------------------------------------------------------
processMCELLPackage(can_id, data{})
{
    switch (can_id) {
    case MCELL_PACK1: {
        mcell_vbat = toShort(data, 0, ORDER_LITTLE_ENDIAN);
        mcell_tbat = toShort(data, 2, ORDER_LITTLE_ENDIAN);
        mcell_tpcb = toShort(data, 4, ORDER_LITTLE_ENDIAN);
        mcell_status = data{6};
    }
    case MCELL_PACK2: {
        cell[0] = toShort(data, 0, ORDER_LITTLE_ENDIAN);
        cell[1] = toShort(data, 2, ORDER_LITTLE_ENDIAN);
        cell[2] = toShort(data, 4, ORDER_LITTLE_ENDIAN);
        cell[3] = toShort(data, 6, ORDER_LITTLE_ENDIAN);
    }
    case MCELL_PACK3: {
        cell[4] = toShort(data, 0, ORDER_LITTLE_ENDIAN);
        cell[5] = toShort(data, 2, ORDER_LITTLE_ENDIAN);
        //cell[6] = toShort(data, 4, ORDER_LITTLE_ENDIAN);
        //cell[7] = toShort(data, 6, ORDER_LITTLE_ENDIAN);
    }
    }

    mcell_min_max();
}

mcell_min_max()
{
    new min = cell[0];
    new max = cell[0];

    for (new i=1; i<6; i++) {
        if(cell[i] < min) {
           min = cell[i];
        }

        if(cell[i] > max) {
            max = cell[i];
        }
    }

    mcell_min = min / 1000.0;
    mcell_max = max / 1000.0;

}

forward @mcell()
@mcell()
{
    printf("v_bat: %.2f\n", mcell_vbat / 100.0);
    printf("t_bat: %.2f\n", mcell_tbat / 100.0);
    printf("t_pcb: %.2f\n", mcell_tpcb / 100.0);
    printf("state: %d\n", mcell_status);

    printf("C[1]: %.2f\n", cell[0] / 1000.0);
    printf("C[2]: %.2f\n", cell[1] / 1000.0);
    printf("C[3]: %.2f\n", cell[2] / 1000.0);
    printf("C[4]: %.2f\n", cell[3] / 1000.0);

    printf("C[5]: %.2f\n", cell[4] / 1000.0);
    printf("C[6]: %.2f\n", cell[5] / 1000.0);

    printf("v_min: %.2f\n", mcell_min);
    printf("v_max: %.2f\n", mcell_max);

    printf("---\n");
}
//------------------------------------------------------------------------------------------
processUVHPUPackage(can_id, data{})
{
    switch (can_id) {
    case UVH_PU_PACK1: {
        uvhpu_vbat = toShort(data, 0, ORDER_LITTLE_ENDIAN)/100.0;
        uvhpu_ibat = toFloat(data, 2, ORDER_LITTLE_ENDIAN);
        uvhpu_imon = toShort(data, 6, ORDER_LITTLE_ENDIAN)/100.0;
    }
    case UVH_PU_PACK2: {
        uvhpu_vout = toShort(data, 0, ORDER_LITTLE_ENDIAN)/100.0;
        uvhpu_tbat = toShort(data, 2, ORDER_LITTLE_ENDIAN)/100.0;
        uvhpu_pbat = toShort(data, 4, ORDER_LITTLE_ENDIAN);
        uvhpu_status = data{6};
    }
    case UVH_PU_PACK3: {
        uvhpu_cbat = toFloat(data, 0, ORDER_LITTLE_ENDIAN);
        uvhpu_ebat = toFloat(data, 4, ORDER_LITTLE_ENDIAN);
    }
    case UVH_PU_PACK4: {
        uvhpu_res_bat = toFloat(data, 0, ORDER_LITTLE_ENDIAN);
        uvhpu_v_res = toFloat(data, 4, ORDER_LITTLE_ENDIAN);
    }
    case UVH_PU_PACK5: {
        uvhpu_ibat_filt = toFloat(data, 0, ORDER_LITTLE_ENDIAN);
        uvhpu_vbat_filt = toFloat(data, 4, ORDER_LITTLE_ENDIAN);
    }
    case UVH_PU_PACK6: {
        uvhpu_cbat_res = toFloat(data, 0, ORDER_LITTLE_ENDIAN);
        uvhpu_ebat_res = toFloat(data, 4, ORDER_LITTLE_ENDIAN);
    }
    case UVH_PU_PACK7: {
        uvhpu_life_cycles = toUShort(data, 0, ORDER_LITTLE_ENDIAN);
        uvhpu_cbat_mod = toFloat(data, 2, ORDER_LITTLE_ENDIAN);
        uvhpu_h_pwr = toShort(data, 6, ORDER_LITTLE_ENDIAN)/100.0;
    }
    }
}

forward @uvhpu()
@uvhpu()
{
    printf("vbat: %.2f\n", uvhpu_vbat);
    printf("ibat: %.2f\n", uvhpu_ibat);
    printf("imon: %.2f\n", uvhpu_imon);

    printf("vout: %.2f\n", uvhpu_vout);
    printf("tbat: %.2f\n", uvhpu_tbat);
    printf("pbat: %.2f\n",uvhpu_pbat);
    printf("status: %d\n", uvhpu_status);

    printf("cbat: %.2f\n", uvhpu_cbat);
    printf("ebat: %.2f\n", uvhpu_ebat);

    printf("res_bar: %.2f\n", uvhpu_res_bat);
    printf("v_res: %.2f\n", uvhpu_v_res);

    printf("ibat_filt: %.2f\n", uvhpu_ibat_filt);
    printf("vbat_filt: %.2f\n", uvhpu_vbat_filt);

    printf("cbat_res: %.2f\n", uvhpu_cbat_res);
    printf("ebat_res: %.2f\n", uvhpu_ebat_res);

    printf("life_cycles: %d\n", uvhpu_life_cycles);
    printf("cbat_mod: %.2f\n", uvhpu_cbat_mod);
    printf("h_pwr: %.2f\n", uvhpu_h_pwr);

    printf("---\n");
}
//------------------------------------------------------------------------------------------
processMPPTPackage(can_id, data{})
{
    switch (can_id) {
        case (CANAS_VOLT_IN):   { mppt_in_volt  = toFloat(data, 4, ORDER_LITTLE_ENDIAN); }
        case (CANAS_VOLT_OUT):  { mppt_out_volt = toFloat(data, 4, ORDER_LITTLE_ENDIAN); }
        case (CANAS_CURR_IN):   { mppt_in_curr = toFloat(data, 4, ORDER_LITTLE_ENDIAN); }
        case (CANAS_CURR_OUT):  { mppt_out_curr = toFloat(data, 4, ORDER_LITTLE_ENDIAN); }
        case (CANAS_TEMP):      { mppt_temp = toFloat(data, 4, ORDER_LITTLE_ENDIAN); }
    }
}

forward @mppt()
@mppt()
{
    printf("in_volt: %.2f\n", mppt_in_volt);
    printf("out_volt:%.2f\n", mppt_out_volt);
    printf("in_curr: %.2f\n", mppt_in_curr);
    printf("out_curr: %.2f\n", mppt_out_curr);
    printf("temp: %.2f\n", mppt_temp);

    printf("---\n");
}

mppt_power_on(id, value)
{
    new data{13} = "";
    data{0} = id & 0xFF;
    data{1} = (id >> 8 ) & 0xFF;
    data{2} = (id >> 16) & 0xFF;
    data{3} = (id >> 24) & 0xFF;
    data{4} = 1; // DLC
    data{5} = value;
    new bool:result = serial_write(PORT_ID_AUX, data,  6, serialmode:NODE);
    if(!result)
        print("MPPT: power on error...\n");
}

mppt_set_k(id, Float: value)
{
    new data{13} = "";
    data{0} = id & 0xFF;
    data{1} = (id >> 8 ) & 0xFF;
    data{2} = (id >> 16) & 0xFF;
    data{3} = (id >> 24) & 0xFF;
    data{4} = 4; // DLC
    serializeFloat(data, 5, value);
    new bool:result = serial_write(PORT_ID_AUX, data,  9, serialmode:NODE);
    if(!result)
        print("MPPT: set k error...\n");
}
//------------------------------------------------------------------------------------------
save_data_to_mandala()
{
    //vesc
    set_var(f_rpm, vesc_rpm, true);
    set_var(f_user4, vesc_dc, true);
    set_var(f_radar_Vx, vesc_ft, true);
    set_var(f_radar_Vy, vesc_mt, true);
    set_var(f_radar_Vz, vesc_crtin, true);

    //mcell
    set_var(f_radar_dx, mcell_vbat / 100.0, true);
    set_var(f_radar_dy, mcell_tbat / 100.0, true);

    set_var(f_platform_Vnorth, mcell_min, true);
    set_var(f_platform_Veast, mcell_max, true);


    //uvhpu
    set_var(f_platform_hmsl, uvhpu_tbat, true);
    set_var(f_ls_spd, uvhpu_res_bat, true);
    set_var(f_radar_dz, uvhpu_ibat_filt, true);

    //mppt
    set_var(f_ls_roll, mppt_in_volt, true);
    set_var(f_ls_pitch, mppt_out_volt, true);
    set_var(f_ls_cp, mppt_out_volt * mppt_out_curr, true);
    set_var(f_platform_hdg, mppt_temp, true);
}

forward @on_can_aux(cnt)
@on_can_aux(cnt)
{
    new can_data{8};

    if (cnt > CAN_PACK_SIZE)
        return;

    new can_id =   serial_byte(0)
                + (serial_byte(1) << 8)
                + (serial_byte(2) << 16)
                + (serial_byte(3) << 24);

    //printf("CAN_ID = %d", can_id);

    const can_data_pos = 5;
    new msg_cnt = serial_byte(4) & 0x0F;

    for (new i = 0; i < msg_cnt; i++) {
        can_data{i} = serial_byte(can_data_pos+i);
    }

    switch (can_id & 0xFFFF) {
        case MCELL_PACK1..MCELL_PACK4: {
            processMCELLPackage(can_id, can_data);
        }
        case UVH_PU_PACK1..UVH_PU_PACK7: {
            processUVHPUPackage(can_id, can_data);
        }
        case CANAS_VOLT_IN..CANAS_TEMP: {
            processMPPTPackage(can_id, can_data);
        }
        case (CANAS_SAVE_POWER_ON + 1)..(CANAS_SAVE_POWER_ON + 5): {
            printf("mppt_%d pwr_on = %d \n", can_id - CANAS_SAVE_POWER_ON, serial_byte(5));
        }
        case (CANAS_SAVE_KMPPT_ADDR + 1)..(CANAS_SAVE_KMPPT_ADDR + 5): {
            printf("mppt_%d k = %.3f", can_id - CANAS_SAVE_KMPPT_ADDR, toFloat(can_data, 0, ORDER_LITTLE_ENDIAN));
        }
    }
}