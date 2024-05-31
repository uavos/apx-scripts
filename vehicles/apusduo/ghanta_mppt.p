/// MPPT
#define PORT_ID_ECU           123
#define CAN_PACK_SIZE         13
#define CAN_PAYLOAD_SIZE       8

#define CANAS_POWER_ON_ADDR       1500
#define CANAS_SAVE_POWER_ON_ADDR  1550

#define CANAS_Kmppt_ADDR       1510
#define CANAS_SAVE_Kmppt_ADDR  1560

#define CANAS_VOLTAGE_IN_ID  1600
#define CANAS_VOLTAGE_OUT_ID 1601
#define CANAS_CURRENT_IN_ID  1602
#define CANAS_CURRENT_OUT_ID 1603
#define CANAS_TEMPERATURE_ID 1604

#define STARTED_Kmppt 0.7

new err_cnt  = 0
new data_cnt = 0
new data{13} = ""
// --------------------------------------------------
// Набор переменных с mppt для телеметрии, которые
//   можно прокинуть через переменные мандалы
//---------------------------------------------------
new Float: mppt1_in_current;
new Float: mppt1_in_voltage;
new Float: mppt1_out_voltage;
new Float: mppt1_out_current;
new Float: mppt1_out_power;
new Float: mppt1_temperature;

new Float: mppt2_in_current;
new Float: mppt2_in_voltage;
new Float: mppt2_out_voltage;
new Float: mppt2_out_current;
new Float: mppt2_out_power;
new Float: mppt2_temperature;

new Float: mppt3_in_current;
new Float: mppt3_in_voltage;
new Float: mppt3_out_voltage;
new Float: mppt3_out_current;
new Float: mppt3_out_power;
new Float: mppt3_temperature;

new Float: mppt4_in_current;
new Float: mppt4_in_voltage;
new Float: mppt4_out_voltage;
new Float: mppt4_out_current;
new Float: mppt4_out_power;
new Float: mppt4_temperature;

new Float: mppt5_in_current;
new Float: mppt5_in_voltage;
new Float: mppt5_out_voltage;
new Float: mppt5_out_current;
new Float: mppt5_out_power;
new Float: mppt5_temperature;

new Float: mppt_out_pawer=0.0;

//=============================================================================
//FOR DEBUG DATA
const DEVICE_ID = 7;
const TELEMETRY_SWCM_PORT = 43;     //port for communication with moduls

const TEL_PACK_SIZE = 52;
new pack_len = 0;
new g_telemetryPack{TEL_PACK_SIZE};
new bool:g_needSendTelemetry = false;
//=============================================================================

new cntStarterRequest = 0;

new Float: state_power_on = 0;

#define COUNT_MMPT 5
new Float:KMPPT_ARRAY[5];

main()
{
    serial_listen(PORT_ID_ECU,"@OnSerialECU");
    serial_listen(TELEMETRY_SWCM_PORT, "@telemetrySwcmHandler");

    KMPPT_ARRAY[0] = STARTED_Kmppt;
    KMPPT_ARRAY[1]= STARTED_Kmppt;
    KMPPT_ARRAY[2] = STARTED_Kmppt;
    KMPPT_ARRAY[3] = STARTED_Kmppt;
    KMPPT_ARRAY[4] = STARTED_Kmppt;
    set_var(f_cam_roll, STARTED_Kmppt, true);
}


forward @OnTask()
@OnTask()
{
    if(cntStarterRequest++ >= 20)
    {
        cntStarterRequest = 0;
        if(state_power_on != get_var(f_power_ignition))
        {
            const number_dev_mppt = 5;
            new id = CANAS_POWER_ON_ADDR;
            for(new i=1; i<number_dev_mppt+1; i++) {
                id = CANAS_POWER_ON_ADDR + i;
                setPowerOn(id, get_var(f_power_ignition))
            }

            //Отладка МППТ 2 (включение)
            //id = CANAS_POWER_ON_ADDR + 2;
            //setPowerOn(id, get_var(f_power_ignition))

            state_power_on = get_var(f_power_ignition);
            printf("new state power");
        }

        new n_mppt = get_var(f_cam_pitch);
        if(n_mppt>=1.0 && n_mppt<=5.0)
        {
           if(KMPPT_ARRAY[n_mppt-1] != get_var(f_cam_roll))
           {
              new id = CANAS_Kmppt_ADDR + n_mppt;
              KMPPT_ARRAY[n_mppt-1] = get_var(f_cam_roll);
              set_Kmppt(id, KMPPT_ARRAY[n_mppt-1]);
              //printf("set kmppt id=%d k=%f", id, KMPPT_ARRAY[n_mppt-1]);
           }
        }

        mppt1_out_power = mppt1_out_voltage*mppt1_out_current;
        mppt2_out_power = mppt2_out_voltage*mppt2_out_current;
        mppt3_out_power = mppt3_out_voltage*mppt3_out_current;
        mppt4_out_power = mppt4_out_voltage*mppt4_out_current;
        mppt5_out_power = mppt5_out_voltage*mppt5_out_current;

        mppt_out_pawer=mppt1_out_power+mppt2_out_power+mppt3_out_power+mppt4_out_power+mppt5_out_power;

        set_var(f_platform_hdg, mppt_out_pawer, true)   // мощность mppt
        set_var(f_Vs, mppt5_out_voltage, true);         // напряжение левого аккумулятора
        set_var(f_Vp, mppt4_out_voltage, true);         // напряжение правого аккумулятора

        new Float: agl = get_var(f_VM32);
        if (agl > 0.1 && agl < 40) {
          set_var(f_agl, agl, true);
        }
    }

    telemetryProcess();
    return 5;
}


forward @OnSerialECU(cnt)
@OnSerialECU(cnt)
{
  if(cnt != (serial_byte(4)+5)) {
    set_var(f_Im, err_cnt++, true);
    if(err_cnt>250){err_cnt=0;}
    return;
  }

  new can_id =   (serial_byte(3) << 24)
               + (serial_byte(2) << 16)
               + (serial_byte(1) << 8)
               +  serial_byte(0);

  switch(can_id) {
  case (CANAS_VOLTAGE_IN_ID):    { mppt1_in_voltage  = getFloatValueFromMessage(9) }
  case (CANAS_VOLTAGE_OUT_ID):   { mppt1_out_voltage = getFloatValueFromMessage(9) }
  case (CANAS_CURRENT_OUT_ID):   { mppt1_out_current = getFloatValueFromMessage(9) }
  case (CANAS_CURRENT_IN_ID):    { mppt1_in_current  = getFloatValueFromMessage(9) }
  case (CANAS_TEMPERATURE_ID):   { mppt1_temperature = getFloatValueFromMessage(9) }

  case (CANAS_VOLTAGE_IN_ID +1*40): { mppt2_in_voltage  = getFloatValueFromMessage(9) }
  case (CANAS_VOLTAGE_OUT_ID+1*40): { mppt2_out_voltage = getFloatValueFromMessage(9) }
  case (CANAS_CURRENT_OUT_ID+1*40): { mppt2_out_current = getFloatValueFromMessage(9) }
  case (CANAS_CURRENT_IN_ID +1*40): { mppt2_in_current  = getFloatValueFromMessage(9) }
  case (CANAS_TEMPERATURE_ID+1*40): { mppt2_temperature = getFloatValueFromMessage(9) }

  case (CANAS_VOLTAGE_IN_ID +2*40): { mppt3_in_voltage  = getFloatValueFromMessage(9) }
  case (CANAS_VOLTAGE_OUT_ID+2*40): { mppt3_out_voltage = getFloatValueFromMessage(9) }
  case (CANAS_CURRENT_OUT_ID+2*40): { mppt3_out_current = getFloatValueFromMessage(9) }// set_var(f_range, mppt3_out_current, true); }
  case (CANAS_CURRENT_IN_ID +2*40): { mppt3_in_current  = getFloatValueFromMessage(9) }
  case (CANAS_TEMPERATURE_ID+2*40): { mppt3_temperature = getFloatValueFromMessage(9) }

  case (CANAS_VOLTAGE_IN_ID +3*40): { mppt4_in_voltage  = getFloatValueFromMessage(9) }
  case (CANAS_VOLTAGE_OUT_ID+3*40): { mppt4_out_voltage = getFloatValueFromMessage(9) }
  case (CANAS_CURRENT_OUT_ID+3*40): { mppt4_out_current = getFloatValueFromMessage(9) }
  case (CANAS_CURRENT_IN_ID +3*40): { mppt4_in_current  = getFloatValueFromMessage(9) }
  case (CANAS_TEMPERATURE_ID+3*40): { mppt4_temperature = getFloatValueFromMessage(9) }

  case (CANAS_VOLTAGE_IN_ID +4*40): { mppt5_in_voltage  = getFloatValueFromMessage(9) }
  case (CANAS_VOLTAGE_OUT_ID+4*40): { mppt5_out_voltage = getFloatValueFromMessage(9) }
  case (CANAS_CURRENT_OUT_ID+4*40): { mppt5_out_current = getFloatValueFromMessage(9) }
  case (CANAS_CURRENT_IN_ID +4*40): { mppt5_in_current  = getFloatValueFromMessage(9) }
  case (CANAS_TEMPERATURE_ID+4*40): { mppt5_temperature = getFloatValueFromMessage(9) }

  case (CANAS_SAVE_POWER_ON_ADDR+1)..(CANAS_SAVE_POWER_ON_ADDR+5):
      printf("mppt_%d power_on=%d", can_id - CANAS_SAVE_POWER_ON_ADDR, serial_byte(5));
  case (CANAS_SAVE_Kmppt_ADDR+1)..(CANAS_SAVE_Kmppt_ADDR+5):
      printf("mppt_%d Kmppt=%f", can_id - CANAS_SAVE_Kmppt_ADDR, getFloatValueFromMessage(5));
  }
  data_cnt++;
}

serializeData(data{}, index, value)
{
  for(new i = 0; i < 4; i++) {
    new shift = 8 * i;
    data{index + i} = (value >> shift) & 0xFF;
  }
}

serializeFloat(data{}, index, Float: value)
{
  for(new i = 0; i < 4; i++) {
    new shift = 8 * i;
    data{index + i} = (value >> shift) & 0xFF;
  }
}

Float: getFloatValueFromMessage(idx)
{
  new data{4};
  new Float:value[1];
  data{3} = serial_byte(idx++);
  data{2} = serial_byte(idx++);
  data{1} = serial_byte(idx++);
  data{0} = serial_byte(idx++);
  memcpy(value, data, 0, 4);
  return value[0];
}

setPowerOn(id, value)
{
  data{0} =  id & 0xFF;
  data{1} =  (id >> 8 ) & 0xFF;
  data{2} =  (id >> 16) & 0xFF;
  data{3} =  (id >> 24) & 0xFF;
  data{4} =  1; // DLC
  data{5} =  value;
  serial_write(PORT_ID_ECU, data,  6, serialmode:NODE);
}

set_Kmppt(id, Float: value)
{
  data{0} =  id & 0xFF;
  data{1} =  (id >> 8 ) & 0xFF;
  data{2} =  (id >> 16) & 0xFF;
  data{3} =  (id >> 24) & 0xFF;
  data{4} =  4; // DLC
  serializeFloat(data, 5, value);
  serial_write(PORT_ID_ECU, data,  9, serialmode:NODE);
}

//============================================================
forward @telemetrySwcmHandler(cnt);
@telemetrySwcmHandler(cnt)
{
    if(cnt == 2){
        if(serial_byte(0) == DEVICE_ID){
            pack_len = serial_byte(1);
            g_needSendTelemetry = true;
        }
        else{
            g_needSendTelemetry = false;
        }
    }
}

telemetryProcess()
{
    if(g_needSendTelemetry)
    {
        g_needSendTelemetry = false;
        createTelemetryPack();
        serial_write(TELEMETRY_SWCM_PORT, g_telemetryPack, pack_len, serialmode:LAN);
    }
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

pack2BytesSFloat(index, Float:bytes)
{
    new sign = 1;
    if(bytes < 0){ sign = -1; bytes = bytes*sign;}

    new i = floatround(bytes, floatround_floor);
    new j = floatround((bytes+0.001-i)*100, floatround_floor);
    i*=sign;
    j*=sign;

    g_telemetryPack{index} = i;
    g_telemetryPack{index+1} = j;
}

calcTelemetryCrc()
{
    new crc = 0xFF;

    for(new i = 0; i < pack_len-1; i++)
    {
        crc ^= g_telemetryPack{i};

        for(new j = 0; j < pack_len-2; j++)
            crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
    }

    return crc;
}

createTelemetryPack()
{
  packByte(0, DEVICE_ID);

  if(pack_len > 1) pack2BytesSFloat(1, mppt1_in_current);
  if(pack_len > 3) pack2BytesSFloat(3, mppt1_in_voltage);
  if(pack_len > 7) pack2BytesSFloat(5, mppt1_out_current);
  if(pack_len > 5) pack2BytesSFloat(7, mppt1_out_voltage);
  if(pack_len > 9) pack2BytesSFloat(9, mppt1_temperature);

  if(pack_len > 11) pack2BytesSFloat(11, mppt2_in_current);
  if(pack_len > 13) pack2BytesSFloat(13, mppt2_in_voltage);
  if(pack_len > 17) pack2BytesSFloat(15, mppt2_out_current);
  if(pack_len > 15) pack2BytesSFloat(17, mppt2_out_voltage);
  if(pack_len > 19) pack2BytesSFloat(19, mppt2_temperature);

  if(pack_len > 21) pack2BytesSFloat(21, mppt3_in_current);
  if(pack_len > 23) pack2BytesSFloat(23, mppt3_in_voltage);
  if(pack_len > 27) pack2BytesSFloat(25, mppt3_out_current);
  if(pack_len > 25) pack2BytesSFloat(27, mppt3_out_voltage);
  if(pack_len > 29) pack2BytesSFloat(29, mppt3_temperature);

  if(pack_len > 31) pack2BytesSFloat(31, mppt4_in_current);
  if(pack_len > 33) pack2BytesSFloat(33, mppt4_in_voltage);
  if(pack_len > 37) pack2BytesSFloat(35, mppt4_out_current);
  if(pack_len > 35) pack2BytesSFloat(37, mppt4_out_voltage);
  if(pack_len > 39) pack2BytesSFloat(39, mppt4_temperature);

  if(pack_len > 41) pack2BytesSFloat(41, mppt5_in_current);
  if(pack_len > 43) pack2BytesSFloat(43, mppt5_in_voltage);
  if(pack_len > 47) pack2BytesSFloat(45, mppt5_out_current);
  if(pack_len > 45) pack2BytesSFloat(47, mppt5_out_voltage);
  if(pack_len > 49) pack2BytesSFloat(49, mppt5_temperature);

  //printf("%f", mppt3_out_current)

  packByte(pack_len-1, calcTelemetryCrc());
}
//============================================================
