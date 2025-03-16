#define NODE_LEFT               TRUE            //L-TRUE R-comment
//#define NODE_RIGHT              TRUE            //R-TRUE L-comment


#if defined NODE_LEFT
new txt_dev{1} = "L"                            //L
#define NODE_ID                 3               //L=1
#define MSG1_ID                 0               //L=0
#define MSG2_ID                 1               //L=1
#define MSG3_ID                 2               //L=2
#define NODE_WING_ID            1               //L=1
#define NODE_WING_REVERSE_ID    2               //R=2
#define PORT_ID_CAN_AUX         20              //L=20

#define CMD_PU1_H               10
#define CMD_PU2_H               11
#define CMD_SP1_T               14
#define CMD_SP2_T               15
#define CMD_SP1_K               18
#define CMD_SP2_K               19
#endif

#if defined NODE_RIGHT
new txt_dev{1} = "R"                            //R
#define NODE_ID                 4               //R=2
#define MSG1_ID                 3               //R=3
#define MSG2_ID                 4               //R=4
#define MSG3_ID                 5               //R=5
#define NODE_WING_ID            2               //R=2
#define NODE_WING_REVERSE_ID    1               //R=1
#define PORT_ID_CAN_AUX         70              //R=70

#define CMD_PU1_H               12
#define CMD_PU2_H               13

#define CMD_SP1_T               16
#define CMD_SP2_T               17
#define CMD_SP1_K               20
#define CMD_SP2_K               21
#endif


#define TASK_DELAY_MS           20
//--------------------ports--------------------
#define PORT_ID_GCU_W           40
#define PORT_ID_GCU_R           41
#define PORT_ID_TLM_SYNC        42
#define PORT_ID_WING            43
#define PORT_ID_MHX             44


#define CAN_PACK_SIZE           13
#define WING_PACK_SIZE          6
#define CNT_DEV                 2
#define MSG7_ID                 6               //for nav-l and nav-r wing


//--------------------uvhpu--------------------
#define UVHPU_ID                128
#define UVHPU_SHIFT             20
#define UVHPU_PACK1             UVHPU_ID + 1
#define UVHPU_PACK2             UVHPU_ID + 2
#define UVHPU_PACK3             UVHPU_ID + 3
#define UVHPU_PACK4             UVHPU_ID + 4
#define UVHPU_PACK5             UVHPU_ID + 5
#define UVHPU_PACK6             UVHPU_ID + 6
#define UVHPU_PACK7             UVHPU_ID + 7

#define UVHPU_CMD_HEATER        UVHPU_ID + 10
#define UVHPU_CMD_PUMP          UVHPU_ID + 11
#define UVHPU_CMD_PON           UVHPU_ID + 12
#define UVHPU_CMD_RB            UVHPU_ID + 13

//data
new uvhpu_vbat[CNT_DEV] = [0,];
new Float: uvhpu_ibat[CNT_DEV] = [0.0,];
new uvhpu_imon[CNT_DEV] = [0,];

new uvhpu_vout[CNT_DEV] = [0,];
new uvhpu_tbat[CNT_DEV] = [0,];
new uvhpu_pbat[CNT_DEV] = [0,];
new uvhpu_status[CNT_DEV] = [0,];

new Float: uvhpu_cbat[CNT_DEV] = [0.0,];
new Float: uvhpu_ebat[CNT_DEV] = [0.0,];

new Float: uvhpu_res_bat[CNT_DEV] = [0.0,];
new Float: uvhpu_v_res[CNT_DEV] = [0.0,];

new Float: uvhpu_ibat_filt[CNT_DEV] = [0.0,];
new Float: uvhpu_vbat_filt[CNT_DEV] = [0.0,];

new Float: uvhpu_cbat_res[CNT_DEV] = [0.0,];
new Float: uvhpu_ebat_res[CNT_DEV] = [0.0,];

new uvhpu_life_cycles[CNT_DEV] = [0,];
new Float: uvhpu_cbat_mod[CNT_DEV] = [0.0,];
new uvhpu_h_pwr[CNT_DEV] = [0,];

//--------------------mcell--------------------
#define MCELL_ID                256
#define MCELL_SHIFT             20
#define MCELL_PACK1             MCELL_ID + 1
#define MCELL_PACK2             MCELL_ID + 2
#define MCELL_PACK3             MCELL_ID + 3
#define MCELL_PACK4             MCELL_ID + 4

//data
new mcell_vbat[CNT_DEV] = [0,];
new mcell_tbat[CNT_DEV] = [0,];
new mcell_tpcb[CNT_DEV] = [0,];
new mcell_status[CNT_DEV] = [0,];
new cell[CNT_DEV][6];

new Float: mcell_min[CNT_DEV] = [0.0,];
new Float: mcell_max[CNT_DEV] = [0.0,];

//--------------------mppt--------------------
#define MPPT_ID                 512
#define MPPT_SHIFT              20
#define MPPT_PACK1              MPPT_ID + 1
#define MPPT_PACK2              MPPT_ID + 2
#define MPPT_SAVE_POWER_ON      MPPT_ID + 6
#define MPPT_SAVE_K             MPPT_ID + 7

#define MPPT_CMD_POWER_ON       MPPT_ID + 10
#define MPPT_CMD_K              MPPT_ID + 11
#define MPPT_CMD_TARGET_VOLT    MPPT_ID + 12

//data
new mppt_vin[CNT_DEV] = [0,];
new mppt_vout[CNT_DEV] = [0,];
new mppt_temp[CNT_DEV] = [0,];
new mppt_status[CNT_DEV] = [0,];
new Float: mppt_cin[CNT_DEV] = [0.0,];
new Float: mppt_cout[CNT_DEV] = [0.0,];

new mppt_target_volt[CNT_DEV] = [100,100];

new mppt_power_state = false;
//--------------------vesc--------------------
#define VESC_ID             0x24    //36

const idx_vesc_rpm =        f_VM1;
const idx_vesc_crt =        f_VM2;
const idx_vesc_dc =         f_VM3;
const idx_vesc_ft =         f_VM4;
const idx_vesc_mt =         f_VM5;
const idx_vesc_crtin =      f_VM6;

new Float: vesc_rpm =       0.0;
new Float: vesc_crt =       0.0;
new Float: vesc_dc =        0.0;
new Float: vesc_ft =        0.0;
new Float: vesc_mt =        0.0;
new Float: vesc_crtin =     0.0;

new Float: last_vesc_rpm =  0.0;

//====================volz====================
const idx_volz_sweep_pos =  f_VM10;
const idx_volz_elv_pos =    f_VM11;
const idx_volz_rud_pos =    f_VM12;

const idx_volz_sweep_temp = f_VM13;
const idx_volz_elv_temp =   f_VM14;
const idx_volz_rud_temp =   f_VM15;

new volz_pos[4] = [0,];
new volz_temp[4] = [0,];

//====================pump====================
const idx_fb_pump =  f_VM20;

new fb_pump = 0;

//============================================
new g_tpUvhpuResponse[CNT_DEV] = [0,];
new g_tpMcellResponse[CNT_DEV] = [0,];
new g_tpMpptResponse[CNT_DEV] = [0,];
new g_tpVescResponse = 0;
new g_tpVolzResponse[4] = [0,];

const DEVICE_TIMEOUT = 3000;

new g_responseValues[11] = [0,];        //uvhpu1, mcell1, mppt1, uvhpu2, mcell2, mppt2, vesc, volz[4]

new size_error_gcu = 0;
new size_error_sync = 0;
new size_error_wing = 0;

//===================Telemetry================
const TELEMETRY_DELAY = 40;

const MSG1_SIZE = 1 + 49 + 1;           //header + data + crc
const MSG2_SIZE = 1 + 49 + 1;           //header + data + crc
const MSG3_SIZE = 1 + 18 + 1;           //header + data + crc

const PACK_SIZE_MAX = 64;               //max pack size
new g_telemetrySize = 0;
new g_telemetryPack{PACK_SIZE_MAX};

new g_tpTelemetryMsg = 0;
new g_telemetrySyncCurrentMsgId = -1;

new bool:g_needSendTelemetry = false;


//===================MandalaVar===============
#if defined NODE_LEFT
const idx_pu1_ibat_flt =    f_platform_lat;
const idx_pu2_ibat_flt =    f_platform_lon;
const idx_mc1_vbat =        f_platform_Vnorth;
const idx_mc2_vbat =        f_Vp;
const idx_vs_rpm =          f_rpm;
const idx_vs_ft =           f_ils_heading;
const idx_vs_mt =           f_ET;
const idx_vs_crtin =        f_turret_pitch;
const idx_wing_heat =       f_userb_1;
#endif

#if defined NODE_RIGHT
const idx_pu3_ibat_flt =    f_platform_hmsl;
const idx_pu4_ibat_flt =    f_platform_hdg;
const idx_pu4_tbat =        f_range;
const idx_mc3_vbat =        f_Vm;
const idx_mc4_vbat =        f_platform_Veast;
const idx_vs_rpm =          f_platform_Vdown;
const idx_vs_ft =           f_ils_altitude;
const idx_vs_mt =           f_OT;
const idx_vs_crtin =        f_turret_heading;
const idx_wing_heat =       f_userb_3;
#endif

//===================Wing Var=================
new wing_nav_temp = 0;
new wing_nav_heat_state = 0;

//===================OnTask===================
const forward_timeout = 100;
const save_timeout = 200;

new forward_timer = 0;
new save_timer = 0;
//============================================

main()
{
    wait (100);

    new now = time();
    forward_timer = now;
    save_timer = now;

    serial_listen(PORT_ID_CAN_AUX,  "@canAuxHandler");
    serial_listen(PORT_ID_GCU_R,    "@gcuHandler");
    serial_listen(PORT_ID_TLM_SYNC, "@tlmSyncHandler");
    serial_listen(PORT_ID_WING,     "@wingHandler");
}

@OnTask()
{
    new now = time();

    //msg1
    checkDeviceOnline(now, g_tpUvhpuResponse[0], 0)
    checkDeviceOnline(now, g_tpMcellResponse[0], 1)
    checkDeviceOnline(now, g_tpMpptResponse[0], 2)

    //msg2
    checkDeviceOnline(now, g_tpUvhpuResponse[1], 3)
    checkDeviceOnline(now, g_tpMcellResponse[1], 4)
    checkDeviceOnline(now, g_tpMpptResponse[1], 5)

    //msg3
    checkDeviceOnline(now, g_tpVescResponse, 6)
    checkDeviceOnline(now, g_tpVolzResponse[0], 7)
    checkDeviceOnline(now, g_tpVolzResponse[1], 8)
    checkDeviceOnline(now, g_tpVolzResponse[2], 9)
    checkDeviceOnline(now, g_tpVolzResponse[3], 10)

    //mppt power on/off
    new m_power_ignition = get_var(f_power_ignition);
    if (mppt_power_state != m_power_ignition) {
        mppt_power_state = m_power_ignition;
        mppt_power_on(0, mppt_power_state);
        mppt_power_on(1, mppt_power_state);
    }

    if (now - forward_timer > forward_timeout) {
        forward_timer = now;
        forwardPackage();
    }

    if (now - save_timer > save_timeout) {
        save_timer = now;
        save_data_to_mandala();

        set_target_volt(0, mppt_target_volt[0]);
        set_target_volt(1, mppt_target_volt[1]);
    }

    if (now - g_tpTelemetryMsg > TELEMETRY_DELAY && g_needSendTelemetry) {
        sendTelemetry();
    }

    return TASK_DELAY_MS;
}
//------------------------------------------------------------------------------------------
packByte(data{}, index, byte)
{
    data{index} = byte;
}

pack2Bytes(data{}, index, bytes)
{
    data{index} = (bytes >> (0 * 8)) & 0xFF;
    data{index + 1} = (bytes >> (1 * 8)) & 0xFF;
}

pack4Bytes(data{}, index, bytes)
{
    data{index} = (bytes >> (0 * 8)) & 0xFF;
    data{index + 1} = (bytes >> (1 * 8)) & 0xFF;
    data{index + 2} = (bytes >> (2 * 8)) & 0xFF;
    data{index + 3} = (bytes >> (3 * 8)) & 0xFF;
}

packFloat(data{}, index, Float:value)
{
    for (new i = 0; i < 4; i++)
        data{index + i} = (value >> (8 * i)) & 0xFF;
}

unpackInt2(data{}, index)
{
    new val = data{index} + (data{index+1} << 8);
    if (val & 0x8000)
        val = (-1)*(0x10000 - val);
    return val;
}

unpackUInt2(data{}, index)
{
    return data{index} + (data{index+1} << 8);
}
/*
unpackInt(data{}, index)
{
    new temp{4};
    new value[1];
    temp{3} = data{index++};
    temp{2} = data{index++};
    temp{1} = data{index++};
    temp{0} = data{index++};
    memcpy(value, temp, 0, 4);

    return value[0];
}
*/
Float:unpackFloat(data{}, index)
{
    new temp{4};
    new Float:value[1];
    temp{3} = data{index++};
    temp{2} = data{index++};
    temp{1} = data{index++};
    temp{0} = data{index++};
    memcpy(value, temp, 0.0, 4.0);
    return value[0];
}

Float:limit(Float: value, Float: min, Float: max)
{
    if(value < min) {
        return min;
    }

    if(value > max) {
      return max;
    }

    return value;
}

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

    for (new i = 0 ; i < n; i++) {
        msg{5+i} = cmd{i};
    }
    serial_write(PORT_ID_CAN_AUX, msg,  5 + n, serialmode:NODE);
}

checkDeviceOnline(timeNow, tpLastResponse, var)
{
    if (timeNow - tpLastResponse > DEVICE_TIMEOUT) {
        g_responseValues[var] = 0;
    } else {
        g_responseValues[var] = 1;
    }
}

packMsg1Msg2(id)
{
    new flags = 0;
    flags |= g_responseValues[id * 3 + 0] << 0;
    flags |= g_responseValues[id * 3 + 1] << 1;
    flags |= g_responseValues[id * 3 + 2] << 2;
    packByte(g_telemetryPack, 1, flags);

    //uvhpu
    pack2Bytes(g_telemetryPack, 2, uvhpu_vout[id]);
    pack2Bytes(g_telemetryPack, 4, uvhpu_tbat[id]);
    packByte(g_telemetryPack,   6, uvhpu_status[id]);
    pack4Bytes(g_telemetryPack, 7, floatint(uvhpu_cbat[id]));
    pack4Bytes(g_telemetryPack, 11, floatint(uvhpu_ebat[id] * 100));
    pack2Bytes(g_telemetryPack, 15, floatint(uvhpu_res_bat[id] * 1000.0));
    pack2Bytes(g_telemetryPack, 17, floatint(uvhpu_v_res[id] * 100.0));
    pack2Bytes(g_telemetryPack, 19, floatint(uvhpu_ibat_filt[id] * 100.0));
    pack2Bytes(g_telemetryPack, 21, floatint(uvhpu_h_pwr[id] * 100.0));

    //mcell
    pack2Bytes(g_telemetryPack, 23, mcell_vbat[id]);
    packByte(g_telemetryPack,   25, mcell_tbat[id] / 100.0);
    packByte(g_telemetryPack,   26, mcell_tpcb[id] / 100.0);
    packByte(g_telemetryPack,   27, mcell_status[id]);
    pack2Bytes(g_telemetryPack, 28, cell[id][0]);
    pack2Bytes(g_telemetryPack, 30, cell[id][1]);
    pack2Bytes(g_telemetryPack, 32, cell[id][2]);
    pack2Bytes(g_telemetryPack, 34, cell[id][3]);
    pack2Bytes(g_telemetryPack, 36, cell[id][4]);
    pack2Bytes(g_telemetryPack, 38, cell[id][5]);

    //mppt
    pack2Bytes(g_telemetryPack, 40, mppt_vin[id]);
    pack2Bytes(g_telemetryPack, 42, mppt_vout[id]);
    packByte(g_telemetryPack,   44, mppt_temp[id] / 100.0);
    packByte(g_telemetryPack,   45, mppt_status[id]);
    pack2Bytes(g_telemetryPack, 46, floatint(mppt_cin[id] * 100.0));
    pack2Bytes(g_telemetryPack, 48, floatint(mppt_cout[id] * 100.0));
}

fillTelemetryPack()
{
    //HEADER
    packByte(g_telemetryPack, 0, (g_telemetrySyncCurrentMsgId | NODE_ID<<4 & 0xF0));

    if (g_telemetrySyncCurrentMsgId == MSG1_ID) {
        g_telemetrySize = MSG1_SIZE;
        packMsg1Msg2(0);
    } else if (g_telemetrySyncCurrentMsgId == MSG2_ID) {
        g_telemetrySize = MSG2_SIZE;
        packMsg1Msg2(1);
    } else if (g_telemetrySyncCurrentMsgId == MSG3_ID) {
        g_telemetrySize = MSG3_SIZE;

        new flags = 0;
        flags |= g_responseValues[6] << 0;
        flags |= g_responseValues[7] << 1;
        flags |= g_responseValues[8] << 2;
        flags |= g_responseValues[9] << 3;
        flags |= g_responseValues[10] << 4;
        packByte(g_telemetryPack, 1, flags);
        pack2Bytes(g_telemetryPack, 2, floatint(vesc_crt * 100.0));
        packByte(g_telemetryPack,   4, floatint(vesc_dc));
        packByte(g_telemetryPack,   5, floatint(vesc_ft));
        packByte(g_telemetryPack,   6, floatint(vesc_mt));
        pack2Bytes(g_telemetryPack, 7, vesc_crtin * 100.0);
        packByte(g_telemetryPack,   9, floatint(volz_pos[0]));
        packByte(g_telemetryPack,   10, floatint(volz_temp[0]));
        packByte(g_telemetryPack,   11, floatint(volz_pos[1]));
        packByte(g_telemetryPack,   12, floatint(volz_temp[1]));
        packByte(g_telemetryPack,   13, floatint(volz_pos[2]));
        packByte(g_telemetryPack,   14, floatint(volz_temp[2]));
        packByte(g_telemetryPack,   15, floatint(volz_pos[3]));
        packByte(g_telemetryPack,   16, floatint(volz_temp[3]));
        packByte(g_telemetryPack,   17, floatint(wing_nav_temp));
        packByte(g_telemetryPack,   18, fb_pump / 100.0);
    }

    //CRC
    new crc = calcTelemetryCrc(g_telemetryPack, g_telemetrySize - 1);
    packByte(g_telemetryPack, g_telemetrySize - 1, crc);
}

sendTelemetry()
{
    serial_write(PORT_ID_GCU_W, g_telemetryPack, g_telemetrySize, serialmode:GCU);
    g_needSendTelemetry = false;
    g_telemetrySyncCurrentMsgId = -1;
}

calcTelemetryCrc(data{}, size)
{
    new crc = 0xFF;
    for(new i = 0; i < size; i++) {
        crc ^= data{i};
        for(new j = 0; j < 8; j++)
            crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
    }
    return crc;
}
//------------------------------------------------------------------------------------------
processUVHPUPackage(can_id, uvhpu_id, data{})
{
    switch (can_id) {
        case (UVHPU_PACK1): {
            uvhpu_vbat[uvhpu_id] = unpackInt2(data, 0);
            uvhpu_ibat[uvhpu_id] = unpackFloat(data, 2);
            uvhpu_imon[uvhpu_id] = unpackInt2(data, 6);
        }
        case (UVHPU_PACK2): {
            uvhpu_vout[uvhpu_id] = unpackInt2(data, 0);
            uvhpu_tbat[uvhpu_id] = unpackInt2(data, 2);
            uvhpu_pbat[uvhpu_id] = unpackInt2(data, 4);
            uvhpu_status[uvhpu_id] = data{6};
        }
        case (UVHPU_PACK3): {
            uvhpu_cbat[uvhpu_id] = unpackFloat(data, 0);
            uvhpu_ebat[uvhpu_id] = unpackFloat(data, 4);
        }
        case (UVHPU_PACK4): {
            uvhpu_res_bat[uvhpu_id] = unpackFloat(data, 0);
            uvhpu_v_res[uvhpu_id] = unpackFloat(data, 4);
        }
        case (UVHPU_PACK5): {
            uvhpu_ibat_filt[uvhpu_id] = unpackFloat(data, 0);
            uvhpu_vbat_filt[uvhpu_id] = unpackFloat(data, 4);
        }
        case (UVHPU_PACK6): {
            uvhpu_cbat_res[uvhpu_id] = unpackFloat(data, 0);
            uvhpu_ebat_res[uvhpu_id] = unpackFloat(data, 4);
        }
        case (UVHPU_PACK7): {
            uvhpu_life_cycles[uvhpu_id] = unpackUInt2(data, 0);
            uvhpu_cbat_mod[uvhpu_id] = unpackFloat(data, 2);
            uvhpu_h_pwr[uvhpu_id] = unpackInt2(data, 6)/100.0;
        }
    }
}

send_cmd_heater(id, value)
{
    new msg{2};
    value *= 100;
    msg{0} = value;
    msg{1} = value >> 8;
    sendCmdToCan(UVHPU_CMD_HEATER + id * UVHPU_SHIFT, msg, 2);
}
//------------------------------------------------------------------------------------------
processMCELLPackage(can_id, mcell_id, data{})
{
    switch (can_id) {
        case MCELL_PACK1: {
            mcell_vbat[mcell_id] = unpackInt2(data, 0);
            mcell_tbat[mcell_id] = unpackInt2(data, 2);
            mcell_tpcb[mcell_id] = unpackInt2(data, 4);
            mcell_status[mcell_id] = data{6};
        }
        case MCELL_PACK2: {
            cell[mcell_id][0] = unpackInt2(data, 0);
            cell[mcell_id][1] = unpackInt2(data, 2);
            cell[mcell_id][2] = unpackInt2(data, 4);
            cell[mcell_id][3] = unpackInt2(data, 6);
        }
        case MCELL_PACK3: {
            cell[mcell_id][4] = unpackInt2(data, 0);
            cell[mcell_id][5] = unpackInt2(data, 2);
            //cell[0][6] = unpackInt2(data, 4);
            //cell[0][7] = unpackInt2(data, 6);
        }
    }
    mcell_min_max(mcell_id);
}

mcell_min_max(mcell_id)
{
    new min = cell[mcell_id][0];
    new max = cell[mcell_id][0];

    for (new i=1; i<6; i++) {
        if(cell[mcell_id][i] < min) {
           min = cell[mcell_id][i];
        }

        if(cell[mcell_id][i] > max) {
            max = cell[mcell_id][i];
        }
    }

    mcell_min[mcell_id] = min / 1000.0;
    mcell_max[mcell_id] = max / 1000.0;
}
//------------------------------------------------------------------------------------------
processMPPTPackage(can_id, mppt_id, data{})
{
    switch (can_id) {
        case (MPPT_PACK1): {
            mppt_vin[mppt_id]  = unpackInt2(data, 0);
            mppt_vout[mppt_id] = unpackInt2(data, 2);
            mppt_temp[mppt_id] = unpackInt2(data, 4);
            mppt_status[mppt_id] = data{6};
        }
        case (MPPT_PACK2): {
            mppt_cin[mppt_id]   = unpackFloat(data, 0);
            mppt_cout[mppt_id]  = unpackFloat(data, 4);
        }
    }
}

mppt_power_on(id, value)
{
    new msg{1};
    msg{0} = value;
    sendCmdToCan(MPPT_CMD_POWER_ON + id * MPPT_SHIFT, msg, 1);
}

set_mppt_coefficient(id, Float: value)
{
    new msg{4};
    packFloat(msg, 0, value);
    sendCmdToCan(MPPT_CMD_K + id * MPPT_SHIFT, msg, 4);
}

set_target_volt(id, value)
{
    new msg{2};
    value *= 100;
    msg{0} = value;
    msg{1} = value >> 8;
    sendCmdToCan(MPPT_CMD_TARGET_VOLT + id * MPPT_SHIFT, msg, 2);
}
//------------------------------------------------------------------------------------------
forwardPackage()
{
    //vesc
    vesc_rpm = get_var(idx_vesc_rpm);
    vesc_crt = get_var(idx_vesc_crt);
    vesc_dc = get_var(idx_vesc_dc);
    vesc_ft = get_var(idx_vesc_ft);
    vesc_mt = get_var(idx_vesc_mt);
    vesc_crtin = get_var(idx_vesc_crtin);

    //volz
    volz_pos[1] = get_var(idx_volz_sweep_pos);
    volz_pos[2] = get_var(idx_volz_elv_pos);
    volz_pos[3] = get_var(idx_volz_rud_pos);

    volz_temp[1] = get_var(idx_volz_sweep_temp);
    volz_temp[2] = get_var(idx_volz_elv_temp);
    volz_temp[3] = get_var(idx_volz_rud_temp);

    //pump
    fb_pump = get_var(idx_fb_pump);
}
//------------------------------------------------------------------------------------------
send_tlm_mhx()
{
    new data{8};

    //HEADER
    packByte(data, 0, (NODE_ID << 4 & 0xF0));

    //Data
    new Float: mcell_pwr = mcell_vbat[0]/100.0 * uvhpu_ibat_filt[0] + mcell_vbat[1]/100.0 * uvhpu_ibat_filt[1];
    new Float: mppt_pwr = mppt_vout[0]/100.0 * mppt_cout[0] + mppt_vout[1]/100.0 * mppt_cout[1];
    new Float: vesc_pwr = mcell_vbat[1]/100.0 * vesc_crtin;

    pack2Bytes(data, 1, floatint(mcell_pwr));
    pack2Bytes(data, 3, floatint(mppt_pwr));
    pack2Bytes(data, 5, floatint(vesc_pwr));

    //CRC
    new crc = calcTelemetryCrc(data, 7);
    packByte(data, 7, crc);

    serial_write(PORT_ID_MHX, data, 8, serialmode:LAN);
}

save_data_to_mandala()
{
    //vesc
    if (vesc_rpm > 50 && vesc_rpm == last_vesc_rpm) {
        vesc_rpm += 5;
    }
    last_vesc_rpm = vesc_rpm;
    set_var(idx_vs_rpm, vesc_rpm, true);
    set_var(idx_vs_ft, vesc_ft, true);
    set_var(idx_vs_mt, vesc_mt+127, true);
    set_var(idx_vs_crtin, vesc_crtin, true);

    //wing
    set_var(idx_wing_heat, wing_nav_heat_state, true);

#if defined NODE_LEFT
    set_var(idx_pu1_ibat_flt, uvhpu_ibat_filt[0], true);
    set_var(idx_pu2_ibat_flt, uvhpu_ibat_filt[1], true);

    set_var(idx_mc1_vbat, mcell_vbat[0]/100.0, true);
    set_var(idx_mc2_vbat, mcell_vbat[1]/100.0, true);
#endif

#if defined NODE_RIGHT
    set_var(idx_pu3_ibat_flt, uvhpu_ibat_filt[0], true);
    set_var(idx_pu4_ibat_flt, uvhpu_ibat_filt[1], true);

    set_var(idx_mc3_vbat, mcell_vbat[0]/100.0, true);
    set_var(idx_mc4_vbat, mcell_vbat[1]/100.0, true);
#endif

    send_tlm_mhx();
}

forward @canAuxHandler(cnt)
@canAuxHandler(cnt)
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

    new now = time();

    if (can_id & 0xFF == VESC_ID) {
        g_tpVescResponse = now;
    }

    switch (can_id & 0xFFFF) {
        case UVHPU_PACK1..UVHPU_PACK7: {
            g_tpUvhpuResponse[0] = now;
            processUVHPUPackage(can_id, 0, can_data);
        }
        case (UVHPU_PACK1 + 1 * UVHPU_SHIFT)..(UVHPU_PACK7 + 1 * UVHPU_SHIFT): {
            g_tpUvhpuResponse[1] = now;
            can_id -= 1 * UVHPU_SHIFT
            processUVHPUPackage(can_id, 1, can_data);
        }
        case MCELL_PACK1..MCELL_PACK4: {
            g_tpMcellResponse[0] = now;
            processMCELLPackage(can_id, 0, can_data);
        }
        case (MCELL_PACK1 + 1 * MCELL_SHIFT)..(MCELL_PACK4 + 1 * MCELL_SHIFT): {
            g_tpMcellResponse[1] = now;
            can_id -= 1 * MCELL_SHIFT;
            processMCELLPackage(can_id, 1, can_data);
        }
        case MPPT_PACK1..MPPT_PACK2: {
            g_tpMpptResponse[0] = now;
            processMPPTPackage(can_id, 0, can_data);
        }
        case (MPPT_PACK1 + 1 * MPPT_SHIFT)..(MPPT_PACK2 + 1 * MPPT_SHIFT): {
            g_tpMpptResponse[1] = now;
            can_id -= 1 * MPPT_SHIFT
            processMPPTPackage(can_id, 1, can_data);
        }
        case MPPT_SAVE_POWER_ON, (MPPT_SAVE_POWER_ON + 1 * MPPT_SHIFT): {
            new n_mppt = (can_id - MPPT_SAVE_POWER_ON) / MPPT_SHIFT + 1;
            printf("SP-%s-%d pwr:%d\n", txt_dev, n_mppt, serial_byte(5));
        }
        case MPPT_SAVE_K, (MPPT_SAVE_K + 1 * MPPT_SHIFT): {
            new n_mppt = (can_id - MPPT_SAVE_K) / MPPT_SHIFT + 1;
            printf("SP-%s-%d k:%.2f\n\n", txt_dev, n_mppt, unpackFloat(can_data, 0));
        }
    }
}
//------------------------------------------------------------------------------------------
forward @gcuHandler(cnt);
@gcuHandler(cnt)
{
    if (cnt != 2 && size_error_gcu++ < 3) {
        printf("Ifc-%s, wrong size(%d) GCU packet\n", txt_dev, cnt);
        return;
    }

    new cmd = serial_byte(0);
    switch (cmd & 0xFF) {
        case CMD_PU1_H: {
            send_cmd_heater(0, limit(serial_byte(1), 0, 30))
        }
        case CMD_PU2_H: {
            send_cmd_heater(1, limit(serial_byte(1), 0, 30))
        }
        case CMD_SP1_T: {
            mppt_target_volt[0] = limit(serial_byte(1), 90, 110);
        }
        case CMD_SP2_T: {
            mppt_target_volt[1] = limit(serial_byte(1), 90, 110);
        }
        case CMD_SP1_K: {
            new Float: mppt_k = serial_byte(1) / 100.0;
            set_mppt_coefficient(0, mppt_k);
        }
        case CMD_SP2_K: {
            new Float: mppt_k = serial_byte(1) / 100.0;
            set_mppt_coefficient(1, mppt_k);
        }
    }
}
//------------------------------------------------------------------------------------------
forward @tlmSyncHandler(cnt);
@tlmSyncHandler(cnt)
{
    if (cnt != 2 && size_error_sync++ < 3) {
        printf("Ifc-%s, wrong telemetry sync packet\n", txt_dev);
        return;
    }

    if (serial_byte(0) == NODE_ID) {
        new msg_id = serial_byte(1);
        if(msg_id == MSG1_ID || msg_id == MSG2_ID || msg_id == MSG3_ID) {
            g_telemetrySyncCurrentMsgId = msg_id;
            g_needSendTelemetry = true;
            g_tpTelemetryMsg = time();
            fillTelemetryPack();
        }
    }
}
//------------------------------------------------------------------------------------------
forward @wingHandler(cnt);
@wingHandler(cnt)
{
    if (cnt != WING_PACK_SIZE && size_error_wing++ < 3) {
        printf("Ifc-%s, wrong telemetry wing packet\n", txt_dev);
        return;
    }

    if(serial_byte(0) == (MSG7_ID | NODE_WING_ID<<4 & 0xF0)) {
        wing_nav_temp = serial_byte(1);       //nav temp
        wing_nav_heat_state = serial_byte(2); //nav temp heater state
        volz_pos[0] =   serial_byte(3);       //volz pos
        volz_temp[0] =  serial_byte(4);       //volz temp
    }
}
//==========================================================================================
forward @pu_on()
@pu_on()
{
    printf("PU-ON-%s:\n", txt_dev);
    new msg{1};
    msg{0} = 1;

    sendCmdToCan(UVHPU_CMD_PON, msg, 1);
    sendCmdToCan(UVHPU_CMD_PON + UVHPU_SHIFT, msg, 1);
}

forward @pu_off()
@pu_off()
{
    printf("PU-OFF-%s:\n", txt_dev);
    new msg{1};
    msg{0} = 0;

    sendCmdToCan(UVHPU_CMD_PON, msg, 1);
    sendCmdToCan(UVHPU_CMD_PON + UVHPU_SHIFT, msg, 1);
}
//------------------------------------------------------------------------------------------
forward @pu_rb()
@pu_rb()
{
    new msg{1};
    new val = limit(get_var(f_ils_RF) * 100.0, 0.0, 100.0);
    printf("PU-%s:%d\n", txt_dev, val);

    msg{0} = val;

    sendCmdToCan(UVHPU_CMD_RB, msg, 1);
    sendCmdToCan(UVHPU_CMD_RB + UVHPU_SHIFT, msg, 1);
}
//------------------------------------------------------------------------------------------
forward @reset_error()
@reset_error()
{
    printf("IFC-%s: gcu_size(%d)\n", txt_dev, size_error_gcu);
    printf("IFC-%s: sync_size(%d)\n", txt_dev, size_error_sync);
    printf("IFC-%s: wing_size(%d)\n", txt_dev, size_error_wing);

    size_error_gcu = 0;
    size_error_sync = 0;
    size_error_wing = 0;
}
//==========================================================================================
//Debug
print_uvhpu(id)
{
    printf("vbat: %.2f\n", uvhpu_vbat[id]/100.0);
    printf("ibat: %.2f\n", uvhpu_ibat[id]);
    printf("imon: %.2f\n", uvhpu_imon[id]/100.0);
    printf("vout: %.2f\n", uvhpu_vout[id]/100.0);
    printf("tbat: %.2f\n", uvhpu_tbat[id]/100.0);
    printf("pbat: %d\n", uvhpu_pbat[id]);
    printf("stat: %d\n", uvhpu_status[id]);
    printf("cbat: %.2f\n", uvhpu_cbat[id]);
    printf("ebat: %.2f\n", uvhpu_ebat[id]);
    printf("res_bar: %.2f\n", uvhpu_res_bat[id]);
    printf("v_res: %.2f\n", uvhpu_v_res[id]);
    printf("ibat_filt: %.2f\n", uvhpu_ibat_filt[id]);
    printf("vbat_filt: %.2f\n", uvhpu_vbat_filt[id]);
    printf("cbat_res: %.2f\n", uvhpu_cbat_res[id]);
    printf("ebat_res: %.2f\n", uvhpu_ebat_res[id]);
    printf("life_cycles: %d\n", uvhpu_life_cycles[id]);
    printf("cbat_mod: %.2f\n", uvhpu_cbat_mod[id]);
    printf("h_pwr: %.2f\n", uvhpu_h_pwr[id]/100.0);
}

#if defined NODE_LEFT
forward @uvhpu_1()
@uvhpu_1()
{
    printf("UVHPU-%s-1:\n", txt_dev);
    print_uvhpu(0);
}

forward @uvhpu_2()
@uvhpu_2()
{
    printf("UVHPU-%s-2:\n", txt_dev);
    print_uvhpu(1);
}
#endif

#if defined NODE_RIGHT
forward @uvhpu_3()
@uvhpu_3()
{
    printf("UVHPU-%s-3:\n", txt_dev);
    print_uvhpu(0);
}

forward @uvhpu_4()
@uvhpu_4()
{
    printf("UVHPU-%s-4:\n", txt_dev);
    print_uvhpu(1);
}
#endif
//------------------------------------------------------------------------------------------
print_mcell(id)
{
    printf("v_bat: %.2f\n", mcell_vbat[id]/100.0);
    printf("t_bat: %.2f\n", mcell_tbat[id]/100.0);
    printf("t_pcb: %.2f\n", mcell_tpcb[id]/100.0);
    printf("state: %d\n", mcell_status[id]);
    printf("C[1]: %.2f\n", cell[id][0]/1000.0);
    printf("C[2]: %.2f\n", cell[id][1]/1000.0);
    printf("C[3]: %.2f\n", cell[id][2]/1000.0);
    printf("C[4]: %.2f\n", cell[id][3]/1000.0);
    printf("C[5]: %.2f\n", cell[id][4]/1000.0);
    printf("C[6]: %.2f\n", cell[id][5]/1000.0);
    printf("C_min: %.2f\n", mcell_min[id]);
    printf("C_max: %.2f\n", mcell_max[id]);
}

#if defined NODE_LEFT
forward @mcell_1()
@mcell_1()
{
    printf("MCELL-%s-1:\n", txt_dev);
    print_mcell(0);
}

forward @mcell_2()
@mcell_2()
{
    printf("MCELL-%s-2:\n", txt_dev);
    print_mcell(1);
}
#endif

#if defined NODE_RIGHT
forward @mcell_3()
@mcell_3()
{
    printf("MCELL-%s-3:\n", txt_dev);
    print_mcell(0);
}

forward @mcell_4()
@mcell_4()
{
    printf("MCELL-%s-4:\n", txt_dev);
    print_mcell(1);
}
#endif

//------------------------------------------------------------------------------------------
print_mppt(id)
{
    printf("vin: %.2f\n", mppt_vin[id]/100.0);
    printf("vout:%.2f\n", mppt_vout[id]/100.0);
    printf("temp: %.2f\n", mppt_temp[id]/100.0);
    printf("status: %d\n", mppt_status[id]);
    printf("cin: %.2f\n", mppt_cin[id]);
    printf("cout: %.2f\n", mppt_cout[id]);
}

#if defined NODE_LEFT
forward @mppt_1()
@mppt_1()
{
    printf("MPPT-%s-1:\n", txt_dev);
    print_mppt(0);
}

forward @mppt_2()
@mppt_2()
{
    printf("MPPT-%s-2:\n", txt_dev);
    print_mppt(1);
}

forward @mppt1_rst()
@mppt1_rst()
{
    mppt_target_volt[0] = 100;
    printf("VOLT[%]: %d\n", mppt_target_volt[0]);
}

forward @mppt2_rst()
@mppt2_rst()
{
    mppt_target_volt[1] = 100;
    printf("VOLT[%]: %d\n", mppt_target_volt[1]);
}
#endif

#if defined NODE_RIGHT
forward @mppt_3()
@mppt_3()
{
    printf("MPPT-%s-3:\n", txt_dev);
    print_mppt(0);
}

forward @mppt_4()
@mppt_4()
{
    printf("MPPT-%s-4:\n", txt_dev);
    print_mppt(1);
}

forward @mppt3_rst()
@mppt3_rst()
{
    mppt_target_volt[0] = 100;
    printf("VOLT[%]: %d\n", mppt_target_volt[0]);
}

forward @mppt4_rst()
@mppt4_rst()
{
    mppt_target_volt[1] = 100;
    printf("VOLT[%]: %d\n", mppt_target_volt[1]);
}
#endif

forward @mppt_on()
@mppt_on()
{
    for (new i=0; i<CNT_DEV; i++) {
        mppt_power_on(i, 1)
    }
}

forward @mppt_off()
@mppt_off()
{
    for (new i=0; i<CNT_DEV; i++) {
        mppt_power_on(i, 0)
    }
}
//------------------------------------------------------------------------------------------
forward @vm_status()
@vm_status()
{
    printf("IFC-%s:Ok...\n", txt_dev);
}
