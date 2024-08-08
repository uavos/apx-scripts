#define NODE_LEFT               TRUE            //L-TRUE R-comment
//#define NODE_RIGHT              TRUE            //R-TRUE L-comment

#if defined NODE_LEFT
#define NODE_ID             1
#define MSG7_ID             6
new txt_dev{1} = "L"
#endif

#if defined NODE_RIGHT
#define NODE_ID             2
#define MSG7_ID             6
new txt_dev{1} = "R"
#endif

#define TASK_DELAY_MS       20
#define PORT_ID_WING        43

//---------------------------temperature----------------------
const idx_nav_temp =        f_RT;

//---------------------------volz-----------------------------
const idx_volz_ail_pos =    f_VM10;
const idx_volz_ail_temp =   f_VM11;

//---------------------------OnTask---------------------------
new schedule_tlm_timer  = 0;
const SCHEDULE_TLM_TIMEOUT = 100;

//---------------------------telemetry------------------------
const PACK_SIZE_MAX = 8;
new g_tlmSize = 1 + 3 + 1;   //header + data + crc
new g_wingTlmPack{PACK_SIZE_MAX};
new send_error = 0;

main()
{
    schedule_tlm_timer = time();
}

@OnTask()
{
    new now = time();

    if (now - schedule_tlm_timer > SCHEDULE_TLM_TIMEOUT) {
        schedule_tlm_timer = now;
        send_wing_telemetry_data();
    }

    return TASK_DELAY_MS;
}

packByte(data{}, index, byte)
{
    data{index} = byte;
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

send_wing_telemetry_data()
{
    //HEADER
    packByte(g_wingTlmPack, 0, (MSG7_ID | NODE_ID << 4 & 0xF0));
    packByte(g_wingTlmPack, 1, get_var(idx_nav_temp));        //nav temp
    packByte(g_wingTlmPack, 2, get_var(idx_volz_ail_pos));    //volz pos
    packByte(g_wingTlmPack, 3, get_var(idx_volz_ail_temp));   //volz temp

    new crc = calcTelemetryCrc(g_wingTlmPack, g_tlmSize - 1);
    packByte(g_wingTlmPack, g_tlmSize - 1, crc);

    new bool:result = serial_write(PORT_ID_WING, g_wingTlmPack, g_tlmSize, serialmode:LAN);

    if (!result && send_error++ < 5)
        printf("NAV-%s: send telemetry error...\n", txt_dev);
}

forward @reset_error()
@reset_error()
{
    printf("NAV-%s: reset(%d)\n", txt_dev, send_error);
    send_error = 0;
}