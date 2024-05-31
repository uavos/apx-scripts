// script for swcm
// implement bridge for debug data

const SWSM_ID = 33;

//-----------------------------------------------------------------------------
const DEV1_ID = 1;         //nav_l
const DEV1_PACK_SIZE = 16; //id(1) + data(14:8var) + crc(1)
const DEV2_ID = 2;         //nav_r
const DEV2_PACK_SIZE = 16; //id(1) + data(14:8var) + crc(1)
const DEV3_ID = 3;         //nav_c
const DEV3_PACK_SIZE = 16; //id(1) + data(14:8var) + crc(1)

const DEV4_ID = 4;         //ifc
const DEV4_PACK_SIZE = 6;  //id(1) + data(4:2var) + crc(1)

const DEV5_ID = 5;         //ghanta_l
const DEV5_PACK_SIZE = 4;  //id(1) + data(2:1var) + crc(1)
const DEV6_ID = 6;         //ghanta_r
const DEV6_PACK_SIZE = 4;  //id(1) + data(2:1var) + crc(1)
const DEV7_ID = 7;         //ghanta_work
const DEV7_PACK_SIZE = 52; //id(1) + data(50:25var) + crc(1)

//... add new periph device

const DEV_SIZE = 3
new DEV_ARRAY[DEV_SIZE] = [DEV1_ID, DEV2_ID, DEV7_ID];       //DEV1_ID, DEV2_ID, DEV3_ID, DEV4_ID, DEV5_ID, DEV6_ID,
new DEV_PACK_SIZE_ARRAY[DEV_SIZE] = [DEV1_PACK_SIZE, DEV2_PACK_SIZE, DEV7_PACK_SIZE];    // DEV3_PACK_SIZE, DEV4_PACK_SIZE, DEV5_PACK_SIZE, DEV6_PACK_SIZE,

//-----------------------------------------------------------------------------

const GCU_PORT_ID_W = 40;           //port for write to gcu
const GCU_PORT_ID_R = 41;           //port for read from gcu
const TELEMETRY_SWCM_PORT = 43;     //port for communication with moduls

//-----------------------------------------------------------------------------

const TELEMETRY_GET_REQUEST_WAIT_TIME = 50;     //period wait answer from slave
new g_tpTelemetryWait = 0;
const TELEMETRY_REQUEST_DELAY = 0;    		    //частота опроса датчиков
new g_tpTelemetryRequest = 0;

//variables
new bool:g_needGetTelemetry = false;
new g_DevIdCount = 6;   //счётчик опроса устройств (выставляется в предпоследнее устройство)

const PACK_SIZE = DEV7_PACK_SIZE;   //DEV1_PACK_SIZE-2+DEV2_PACK_SIZE-2+DEV3_PACK_SIZE-2+DEV4_PACK_SIZE-2+DEV5_PACK_SIZE-2+ DEV6_PACK_SIZE-2+

new g_telemetryPack{PACK_SIZE};
new g_telemetryPackCurrentIdx = 0;
const TELEMETRY_BYTES_PER_TICK = PACK_SIZE;//7;    //число байт передаваемое на наземку за итерацию
new g_curCountTelemetryPack = 1;
new bool:g_needSendTelemetry = false;

new bool:g_firstTelemetry = true;   //флаг первой отправки не синхронизированных пакетов
new g_timeLastTelemetry = 0;        //время начала последней отправки телеметрии
new g_timeNowTelemetry = 0;         //время начала текущей отправки телеметрии
const TELEMETRY_SEND_TIME = 200;    //время отправки данных в gcu

new g_telemetryCrcPack{PACK_SIZE};  //массив для проверки crc

new Float: let_float = 0.0;
//-----------------------------------------------------------------------------

main(){
  telemetryPackInit();
  serial_listen(TELEMETRY_SWCM_PORT, "@telemetrySwcmHandler");
  return 0
}

//===============================INTERRUPTS====================================

@OnTask()
{
    new timeNow = time();
    new userb_6 = get_var(f_sw_sw2);

    if(userb_6){
        if(g_needSendTelemetry)     //send to gcu
        {
            if(g_firstTelemetry || (!g_firstTelemetry && (timeNow - g_timeLastTelemetry >= TELEMETRY_SEND_TIME)) ){
                sendTelemetry();
                g_tpTelemetryRequest = timeNow;
            }
        }
        if(g_needGetTelemetry)
        {
            if(timeNow - g_tpTelemetryWait > TELEMETRY_GET_REQUEST_WAIT_TIME)	//время ответа истекло
            {
                if(DEV_ARRAY[g_DevIdCount] == DEV7_ID){
                    telemetryPackFill(g_curCountTelemetryPack, DEV_PACK_SIZE_ARRAY[g_DevIdCount] - 1, 0xFF);
                    g_curCountTelemetryPack += DEV_PACK_SIZE_ARRAY[g_DevIdCount] - 2;

                    if(g_DevIdCount == DEV_SIZE-1) {
                        packByte(PACK_SIZE - 1, calcTelemetryCrc());    //crc
                        g_needSendTelemetry = true;
                    }
                }

                g_needGetTelemetry = false;
                g_tpTelemetryRequest = timeNow;
            }
        }
        else if(timeNow - g_tpTelemetryRequest > TELEMETRY_REQUEST_DELAY)   //wait and send request to slave
        {
            sendTelemetryRequestData();
            g_needGetTelemetry = true;
            g_tpTelemetryWait = timeNow;
        }
    }
    else { g_firstTelemetry = true; }

    return 10;
}




// Handler for get data from device
forward @telemetrySwcmHandler(cnt);
@telemetrySwcmHandler(cnt)      //cnt <= 10 !!!!!!!!!
{
    if(g_needGetTelemetry && serial_byte(0) == DEV_ARRAY[g_DevIdCount])
    {
        if(cnt == DEV_PACK_SIZE_ARRAY[g_DevIdCount]){
            for(new k = 0; k < cnt-1; k++)
                g_telemetryCrcPack{k} = serial_byte(k)

            if(serial_byte(cnt-1) == checkTelemetryCrc(cnt)%256)
            {
                //GHANTA_WORK
                if(serial_byte(0) == DEV7_ID)
                {
                  for(new j = 1; j < DEV_PACK_SIZE_ARRAY[g_DevIdCount] - 1; j++){
                    g_telemetryPack{g_curCountTelemetryPack} = serial_byte(j);
                    g_curCountTelemetryPack++;
                  }
                }
                //NAV_L
                else if(serial_byte(0) == DEV1_ID)
                {
                    sFloatToFloat(serial_byte(1), serial_byte(2));
                    set_var(f_user5, let_float, true);  //curr_in

                    sFloatToFloat(serial_byte(3), serial_byte(4));
                    set_var(f_platform_Vnorth, let_float, true);    //V_in

                    set_var(f_user2, doubleByteToInt(serial_byte(5), serial_byte(6)), true);    //rpm
                    set_var(f_radar_dx, serial_byte(7), true);  //err

                    sFloatToFloat(serial_byte(9), serial_byte(10));
                    let_float=(let_float+127)/10.0;
                    set_var(f_Ip, let_float, true);         //tmp esc

                    sFloatToFloat(serial_byte(11), serial_byte(12));
                    let_float=let_float+127;
                    set_var(f_ET, let_float, true);         //tmp mtr

                    sFloatToFloat(serial_byte(13), serial_byte(14));
                    set_var(f_platform_lat, let_float, true);    //Duty left
                }
                //NAV_R
                else if(serial_byte(0) == DEV2_ID)
                {
                    sFloatToFloat(serial_byte(1), serial_byte(2));
                    set_var(f_range, let_float, true);  //curr_in

                    sFloatToFloat(serial_byte(3), serial_byte(4));
                    set_var(f_platform_Vdown, let_float, true);    //V_in

                    set_var(f_user3, doubleByteToInt(serial_byte(5), serial_byte(6)), true);    //rpm
                    set_var(f_radar_dz, serial_byte(7), true);  //err

                    sFloatToFloat(serial_byte(9), serial_byte(10));
                    let_float=(let_float+127)/10.0;
                    set_var(f_Is, let_float, true);        //tmp esc

                    sFloatToFloat(serial_byte(11), serial_byte(12));
                    let_float=let_float+127;
                    set_var(f_OT, let_float, true);       //tmp mtr

                    sFloatToFloat(serial_byte(13), serial_byte(14));
                    set_var(f_platform_lon, let_float, true);    //Duty right
                }

                if(g_DevIdCount == DEV_SIZE-1) {
                    packByte(PACK_SIZE - 1, calcTelemetryCrc());    //crc
                    g_needSendTelemetry = true;
                }

                g_needGetTelemetry = false;
                g_tpTelemetryRequest = time(); //get data from lan and sync for get data from next device
            }
        }
    }
}

//===============================FUNCTIONS=====================================

sendTelemetryRequestData()
{
    new data{2};

    if(++g_DevIdCount >= DEV_SIZE){
        g_DevIdCount = 0;
        g_curCountTelemetryPack = 1;
    }

    data{0} = DEV_ARRAY[g_DevIdCount];
    data{1} = DEV_PACK_SIZE_ARRAY[g_DevIdCount];
    serial_write(TELEMETRY_SWCM_PORT, data, 2, serialmode:LAN);
    //printf("HWCM: %d, %d", data{0}, data{1});
}

sendTelemetry()
{
    new max = g_telemetryPackCurrentIdx + TELEMETRY_BYTES_PER_TICK;

    if(max > PACK_SIZE)
        max = PACK_SIZE;

    new size = max - g_telemetryPackCurrentIdx;
    new data{TELEMETRY_BYTES_PER_TICK};
    for(new i = 0; i < size; i++)
        data{i} = g_telemetryPack{g_telemetryPackCurrentIdx + i};

    serial_write(GCU_PORT_ID_W, data, size, serialmode:GCU);
//    printf("h %d %d %d %d %d %d", data{0}, data{1}, data{2}, data{3}, data{4}, data{5});
    if(g_telemetryPackCurrentIdx == 0) g_timeNowTelemetry = time();

    g_telemetryPackCurrentIdx += size;

    if(max == PACK_SIZE)
    {
        g_telemetryPackCurrentIdx = 0;
        g_needSendTelemetry = false;
        g_firstTelemetry = false;//устанавливаем флаг первой отправки
        g_timeLastTelemetry = g_timeNowTelemetry;
    }
}

//=============================================================================

checkTelemetryCrc(len)
{
    new crc = 0xFF;

    for(new i = 0; i < len - 1; i++)
    {
        crc ^= g_telemetryCrcPack{i};

        for(new j = 0; j < len-2; j++)
            crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
    }

    return crc;
}

calcTelemetryCrc()
{
    new crc = 0xFF;

    for(new i = 0; i < PACK_SIZE - 1; i++)
    {
        crc ^= g_telemetryPack{i};

        for(new j = 0; j < PACK_SIZE - 2; j++)
            crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
    }

    return crc;
}

packByte(index, byte)
{
    g_telemetryPack{index} = byte;
}

pack2Bytes(index, bytes)
{
    g_telemetryPack{index} = (bytes >> (0 * 8)) & 0xFF;
    g_telemetryPack{index + 1} = (bytes >> (1 * 8)) & 0xFF;
}

telemetryPackFill(begin, end, byte)
{
    for(new i = begin; i < begin+end; i++)
        packByte(i, byte);
}

telemetryPackInit()
{
    packByte(0, SWSM_ID);
    telemetryPackFill(1, PACK_SIZE - 1, 0xFF);
}

sFloatToFloat(i, j)
{
    new Float: sign = 1.0;
    new Float: _i = i
    new Float: _j = j

    if(i > 128){ sign = -1.0; _i = 256-i; }
    if(j > 128){ sign = -1.0; _j = 256-j; }
    let_float = (_i+(_j/100.0))*sign;
}

doubleByteToInt(i, j)
{
    new Int:var_i = i
    new Int:var_j = j*256
    return var_i+var_j
}

//==================================END========================================
