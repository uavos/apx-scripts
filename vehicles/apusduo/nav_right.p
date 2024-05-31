 // ESC
#define GET_VALUES 4
#define SET_DUTY   5
#define SET_CURRENT 6
#define SET_CURRENT_BRAKE 7
#define ALIVE 30

#define PORT_ID 10  // ch

#define DATA 5

// For VESC
#define PACKED_SIZE 78
///

//FOR DEBUG DATA
const DEVICE_ID = 2;
const TELEMETRY_SWCM_PORT = 43;     //port for communication with moduls

const TEL_PACK_SIZE = 16;
new pack_len = 0;
new g_telemetryPack{TEL_PACK_SIZE};
new bool:g_needSendTelemetry = false;
//=============================================================================
new Float: current_in = 0.0;
new Float: mot_current = 0.0;
new Float: voltage_in = 0.0;
new Float: rpm = 0.0;           //uint16_t
new Float: ch_throttle = 0.0;
new fault_code = 0;
new crc_error = 0;
new all_error = 0;              //все ошибки +1 или +10
new temp_mos = 0;
new temp_motor = 0;
new Float: duty_motor = 0.0;
new tmp = 0;

new cntStarterRequest = 0;

/*****************************/
#define CURRENT_MAX -50.0
#define START_CURRENT_BRAKE 0.5
#define CURRENT_BRAKE 0.7           //ток рекуперации
#define THR_CHANGE_RATE 15          //15% изменение throttle в сек (рассчитываем на сколько может измениться throttle за одну итерацию)
#define BRAKE_CHANGE_RATE 0.1       //0.1 ампер нарастание тока за одну итерацию
#define BRAKE_MAX 5                 //максимальный ток складывания винта
/*****************************/

//======speed throttle=======
new last_time_throttle=0;           //предыдущее время вызова скрипта, нужно для отслеживания изменения trottle
new Float: air_brk=0.0;
new Float: prev_throttle=0.0;       //значение throttle на предыдущей итерации
new Float: delta_throttle=0.0;      //изменение throttle за одну итерацию
new Float: output_throttle=0.0;     //выходное значение throttle
new Float: c_brake=0.0;             //ток складывания винта изменяется от START_CURRENT_BRAKE до 5
//===========================

new crc16_tab[] = [ 0x0000, 0x1021, 0x2042, 0x3063, 0x4084,
        0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
        0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7,
        0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a,
        0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
        0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719,
        0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7,
        0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948,
        0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50,
        0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b,
        0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
        0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97,
        0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe,
        0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca,
        0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
        0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d,
        0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214,
        0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,
        0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3,
        0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d,
        0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806,
        0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e,
        0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1,
        0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
        0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0,
        0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
        0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0]; // write non table crc - method

//=============================================================================


main()
{
    last_time_throttle = time();
    serial_listen(PORT_ID, "@OnSerial");
    serial_listen(TELEMETRY_SWCM_PORT, "@telemetrySwcmHandler");
}

@OnTask()
{
    new timeNow = time();
    if(cntStarterRequest++ >= 2){
        cntStarterRequest = 0;

        air_brk=get_var(f_ctr_airbrk);

        //====speed throttle====
        new Float: d_time=(timeNow-last_time_throttle)/1000.0;        //мсек -> сек
        delta_throttle=(THR_CHANGE_RATE/100.0)*d_time;

        if(air_brk==0){
            output_throttle=get_ch(3);              //значение throttle с автопилота
            if(get_var(f_ctr_throttle)==0){         //чтоб не подруливал рудером
               output_throttle=0.0;
            }
            if(output_throttle>prev_throttle){
                if((output_throttle-prev_throttle)>delta_throttle){ //если за одну итерацию значение поменялось больше чем delta_throttle
                    output_throttle=prev_throttle+delta_throttle;
                }
            } else {
                if(get_var(f_sw_sw1) && get_var(f_cmode_throvr)){             //работает только когда thr управляется мощьностью
                  if(abs(output_throttle-prev_throttle)>delta_throttle){
                    output_throttle=prev_throttle-delta_throttle;
                  }
                }
            }
        }

        prev_throttle=output_throttle;
        last_time_throttle=timeNow;
        //================

        ch_throttle=output_throttle;


        if((ch_throttle>0) && (air_brk==0)){
            c_brake=START_CURRENT_BRAKE;
            //setCurrent(ch_throttle*CURRENT_MAX);
            setDuty(ch_throttle * (-1.0));
        }


        if(((ch_throttle==0)&&(air_brk==0)) || (get_var(f_cmode_thrcut)>0) || ((air_brk>0)&&(get_var(f_airspeed)<8))){
            prev_throttle=0.0;
            if(c_brake<BRAKE_MAX){
                c_brake+=BRAKE_CHANGE_RATE;
            }
            setCurrentBrake(c_brake);       // Ток складывания винта
        }


        if((air_brk>0) && (get_var(f_airspeed)>8) && (get_var(f_cmode_thrcut)==0)){
            c_brake=START_CURRENT_BRAKE;

            if((get_var(f_rpm)<200)){
                output_throttle=0.03;
                ch_throttle=output_throttle*CURRENT_MAX;
                setCurrent(ch_throttle);
            }

            if(get_var(f_rpm)>400){
                setCurrentBrake(CURRENT_BRAKE);     // Рекуперативное торможение винтом
            }
        }

        if(air_brk>0) {
            prev_throttle = 0.03;                    //для поавного выхода из airbrk
        }


        set_var(f_ils_altitude, output_throttle, true);       //новое значение throttle в user2
        //set_var(f_user1, c_brake, true);                    //ток складывания винта

        if(tmp++==20) {
            getValues();
            tmp=0;
        }
    }

    telemetryProcess();
    return 5;
}

new Float: last_rpm = 0.0;
forward @OnSerial(cnt)
@OnSerial(cnt)
{
  if(cnt < PACKED_SIZE)
    return ;
  //printf("nBytes = %d\n", cnt);
  //printf("crc_error = %d\n", crc_error);
  new message_c[PACKED_SIZE];               // temp solution, change to copy function

  for(new i=0; i<serial_byte(1); i++){        // first 2 byte + last byte
    message_c[i] = serial_byte(i+2);
  }

  new crc_ch = calc_crc16(message_c,serial_byte(1));
  new crc_idx = serial_byte(1) + 2;

  if(crc_ch != (serial_byte(crc_idx) << 8) + serial_byte(crc_idx + 1)){
    crc_error++;
    all_error++;
    set_var(f_radar_dy, all_error, true);
    return ;
  }

  new msg_type = serial_byte(2);
  //printf("%d\n", msg_type);

  switch(msg_type){
    case GET_VALUES:{
        new param;
        //temp_mos = (((serial_byte(3) << 8) + serial_byte(4))/10.0);   //mc_interface_temp_fet_filtered
        param = (serial_byte(3) << 8) + serial_byte(4);                 //mc_interface_temp_fet_filtered
        if (serial_byte(3) & 0x80)
          param += (0xffff << 16);
        temp_mos = param/10.0;

        //temp_motor = (((serial_byte(5) << 8) + serial_byte(6))/10.0); //mc_interface_temp_motor_filtered
        param = (serial_byte(5) << 8) + serial_byte(6);                 //mc_interface_temp_fet_filtered
        if (serial_byte(5) & 0x80)
          param += (0xffff << 16);
        temp_motor = param/10.0;

        mot_current= ((serial_byte(7) << 24) + (serial_byte(8) << 16) + (serial_byte(9) << 8) + serial_byte(10))/100.0;// mc_interface_read_reset_avg_motor_current
        current_in = ((serial_byte(11) << 24) + (serial_byte(12) << 16) + (serial_byte(13) << 8) + serial_byte(14))/100.0;// mc_interface_read_reset_avg_input_current
        // 15,16,17,18 mc_interface_read_reset_avg_id
        // 19,20,21,22 mc_interface_read_reset_avg_iq

        param = (serial_byte(23) << 8) + serial_byte(24);               //mc_interface_get_duty_cycle_now
        if (serial_byte(23) & 0x80)
          param += (0xffff << 16);
        duty_motor=param/10.0;

        rpm = ((serial_byte(25) << 24) + (serial_byte(26) << 16) + (serial_byte(27) << 8) + serial_byte(28))/(-21.0); //mc_interface_get_rpm
        voltage_in = ((serial_byte(29) << 8) + serial_byte(30))/10.0;

        if(rpm>50 && (last_rpm==rpm)){
          rpm+=5;
        }
        last_rpm=rpm;
        if(rpm<30){
          rpm=0.0;
        }

        set_var(f_rpm,rpm,true);
        set_var(f_platform_Veast,voltage_in,true);
        set_var(f_user6,current_in,true);
        set_var(f_EGT,(temp_motor+127)*10.0, true);
        set_var(f_OP, (temp_mos+127)/10.0, true);
        set_var(f_ils_heading, duty_motor, true);

        //----------------------------------------------------------------
        fault_code = serial_byte(55);
        if(fault_code>0){all_error+=10;}
        set_var(f_radar_dy, all_error, true);
    }
  }
}

serializeInt(data{},index,value){

  for(new i = 0 ; i<4;i++){
    new shift = 8 * (4-i-1)
    data{index+i} = (value >> shift) & 0xFF;
  }
}

getValues(){
  new message{1+DATA};
  message{2} = GET_VALUES;
  sendMessage(message,1)
}

sendAlive()
{
  new message{1+DATA};
  message{2} = ALIVE;
  sendMessage(message,1)
}

setCurrent(Float: val)
{

  new message{5+DATA};
  new current = roundDouble(val * 1000);  // scale = 1e3
  message{2} = SET_CURRENT;
  serializeInt(message,3,current);
  sendMessage(message,5)
}

setCurrentBrake(Float:val)
{

  new message{5+DATA};
  new current = roundDouble(val * 1000);  // scale = 1e3
  message{2} = SET_CURRENT_BRAKE;
  serializeInt(message,3,current,floatint(roundDouble(val*1000)));
  sendMessage(message,5)
}

setDuty(Float:val)
{
  new message{5 + DATA};
  new current = roundDouble(val * 100000);  // scale = 1e5
  message{2} = SET_DUTY;
  serializeInt(message,3,current,floatint(roundDouble(val*100000)));
  sendMessage(message,5)
}

roundDouble(Float: x) {
    return x < 0.0 ? floatround(x - 0.5,floatround_ceil) : floatround(x + 0.5,floatround_floor);
}

sendMessage(message{},messageSize)
{
    if(messageSize <= 256){
      message{0} = 2;
      message{1} = messageSize;
    }else{
      // always < 256
    }

    new message_c[10];               // temp solution

    for(new i=0;i<messageSize;i++){
      message_c[i] = message{i+2};
    }

    new crc = calc_crc16(message_c,messageSize);
    message{messageSize+2} = crc >> 8;
    message{messageSize+3} = crc & 0xFF
    message{messageSize+4} = 3;
    serial_write(PORT_ID, message, messageSize + DATA, serialmode:NODE);
}


calc_crc16(message[],messageSize){
   new cksum = 0;
   for (new i = 0; i < messageSize; i++) {
        cksum = crc16_tab[(((cksum >> 8) ^ message[i]) & 0xFF)] ^ (cksum << 8);
    }
   return cksum & 0xFFFF;
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

  new Int: let_rpm = floatround(rpm, floatround_ceil);
  if(let_rpm < 0) let_rpm=0

  if(pack_len > 1) pack2BytesSFloat(1, current_in);             //Mandala
  if(pack_len > 3) pack2BytesSFloat(3, voltage_in);             //Mandala
  if(pack_len > 5) pack2Bytes(5, let_rpm);                      //Mandala
  if(pack_len > 7) packByte(7, fault_code);
  if(pack_len > 8) packByte(8, crc_error);
  if(pack_len > 9) pack2BytesSFloat(9, temp_mos);
  if(pack_len > 11) pack2BytesSFloat(11, temp_motor);           //Mandala
  if(pack_len > 13) pack2BytesSFloat(13, duty_motor);           //Mandala

  packByte(pack_len-1, calcTelemetryCrc());
}
//============================================================

