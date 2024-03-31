//Swiwin turbine control protocol version 1.0
//UART: 9600 8 N 2

#define PORT_ID 30

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


#define MSG_SIZE 7
new frame_length;

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

//---------------------------------------------------------------
new start_eng = false;
//---------------------------------------------------------------

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
/***************************************************************
** Set State for RC Switch
** Input: eSW_STOP, eSW_READY, eSW_START
***************************************************************/
set_switch(sw)
{
  m_ctr_switch = sw;
}
/***************************************************************
** Set Valve for RC Throttle
** Input: 0 ~ 1000
***************************************************************/
set_throttle(val)
{
    if (val > 1000) {
        val = 1000;
    }
    m_ctr_throttle = val;
}

EcuPrintError(id)
{
  switch(id)
  {
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
  switch(id)
  {
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

//---------------------------------------------------------------
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
  new  press;

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
  m_pump_ign_voltage = (m_rev_buf{3}) * 2; //0.10 ~  5.00v
  m_rpm_acc = m_rev_buf{4};  //10 ~ 70

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
//---------------------------------------------------------------
print_and_save_param()
{
  if(m_error_code != m_last_error_code) {
    EcuPrintError(m_error_code);
    m_last_error_code = m_error_code;
  }

  if(m_engine_status != m_last_engine_status) {
    EcuPrintStatus(m_engine_status);
    m_last_engine_status = m_engine_status;
  }

  set_var(f_rpm, m_engine_rpm / 10, true);
  set_var(f_EGT, m_engine_temp, true);
  set_var(f_user5, m_throttle, true);
  set_var(f_Vm, m_pwr_cur_voltage / 10.0, true);
}
//---------------------------------------------------------------
main()
{
  frame_length = 0;
  serial_listen(PORT_ID, "@OnSerial");
  return 0
}
//---------------------------------------------------------------
forward @OnSerial(cnt)
@OnSerial(cnt)
{
  frame_length = 0;

  if (cnt != MSG_SIZE) {
    return;
  }


  for(new i = 0; i < cnt; i++) {
    m_rev_buf{frame_length++} = serial_byte(i);
  }

  if(calc_crc8(m_rev_buf, 0, MSG_SIZE-1) == m_rev_buf{MSG_SIZE-1}) {
    state_comm();
    switch(m_rev_buf{0} & 0x0F)
    {
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
//---------------------------------------------------------------
get_send_buf()
{
    m_snd_buf{0} = 0xFF; //Header
    m_snd_buf{4} = 0xFF;

    if (m_set_ign_pump) {
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
    } else if (m_set_rpm_acc) {
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
    } else {

        //Send Control(Switch & Throttle)
        m_snd_buf{1} = eCMD_POWER << 4;
        m_snd_buf{1} |= (m_ctr_switch & 0x03) << 2;
        m_snd_buf{1} |= ((m_ctr_throttle & 0x300) >> 8);
        m_snd_buf{2} = ((m_ctr_throttle & 0x0FF) >> 0);
        m_snd_buf{3} = calc_crc8(m_snd_buf, 1, 2);

        if (m_update_rate != m_set_update_rate && m_version >= 1) {
            switch (m_set_update_rate) {
            case eUPDATE_RATE_20Hz:
                m_snd_buf{6} = eCMD_SEND_RATE_20Hz;
                //break;
            case eUPDATE_RATE_50Hz:
                m_snd_buf{6} = eCMD_SEND_RATE_50Hz;
                //break;
            case eUPDATE_RATE_100Hz:
                m_snd_buf{6} = eCMD_SEND_RATE_100Hz;
                //break;
            }
            m_snd_buf{5} = eCMD_SHORT << 4;
            m_snd_buf{7} = calc_crc8(m_snd_buf, 5, 2);
            m_cmd_param = 0;
            return 8;
        }
        //Send Short Command
        else if (m_cmd_param) {
            m_snd_buf{5} = eCMD_SHORT << 4;
            m_snd_buf{6} = m_cmd_param;
            m_snd_buf{7} = calc_crc8(m_snd_buf, 5, 2);
            m_cmd_param = 0;
            return 8;
        }
        return 4;
    }
}

//---------------------------------------------------------------
@OnTask()
{

  new on_power_ignition = get_var(f_power_ignition) > 0.0;

  //start
  if (on_power_ignition && get_var(f_sw_starter) > 0.0) {
    set_switch(eSW_START);
    set_var(f_sw_starter, 0.0, true);
    start_eng = true;
  }

  //ready
  if (on_power_ignition && m_switch != eSW_START && !start_eng) {
    set_switch(eSW_READY);
  }

  //stop and ctr_throttle
  if (on_power_ignition) {
    set_throttle(get_var(f_ctr_throttle) * 1000);
  } else {
    set_switch(eSW_STOP);
    start_eng = false;
  }

  //cooling
  if (m_engine_temp > 100.0 && !start_eng) {
    set_switch(eSW_READY);
    set_var(f_power_ignition, 1.0, true);
  }


  new len = get_send_buf();
  if (len) {
    serial_write(PORT_ID, m_snd_buf, len, serialmode:NODE);
  }

  return 200;
}
