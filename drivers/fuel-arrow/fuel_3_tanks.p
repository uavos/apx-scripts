#define PORT_RS485      240
#define ADR_SENS1       75
#define ADR_SENS2       76
#define ADR_SENS3       77 //???
#define PACK_SIZE       9

//editable
#define V_MAX           15.7    //liters

#define P_TANK_3        50.0    //%
#define V_BLOCK_PUMP    1.0     //%

#define KF1_RATIO       1.0
#define KF2_RATIO       3.0     //tank1 = 3 * tank2

#define TIME_HYST       1       //sec
#define TIME_SA         2       //sec
#define TIMER_MC        50      //sec

#define DELAY_MS        200

//read
new MANDALA_IGNITION = f_power_ignition;
new MANDALA_ERS = f_ctrb_ers;
new MANDALA_ALGORITM = f_userb_1;

//write
new MANDALA_PUMP1 = f_userb_2;
new MANDALA_PUMP2 = f_userb_3;
new MANDALA_PUMP3 = f_userb_4;
new MANDALA_FUEL = f_fuel;
new MANDALA_FUEL_V1 = f_user1;
new MANDALA_FUEL_V2 = f_user2;
new MANDALA_FUEL_V3 = f_user3;
//new MANDALA_FUEL_RATIO = f_user4;

new MANDALA_WARN_ANSWER1 = f_radar_Vy;
new MANDALA_WARN_ANSWER2 = f_radar_Vz;
new MANDALA_WARN_ANSWER3 = f_radar_Vx;

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
new pump_stage = -1;

new Float: kf_start = 0.0;
new Float: kf_stop = 0.0;
new Float: m_vFuelPersent1;
new Float: m_vFuelPersent2;
new Float: m_vFuelPersent3;
new Float: fuel_ratio;
new m_timeAns1;
new m_timeAns2;
new m_timeAns3;

main()
{
    m_ignitionOld = false;
    m_algoritmOld = false;
    m_start_pump1 = false;

    set_var(MANDALA_PUMP1, 0, true);
    set_var(MANDALA_PUMP2, 0, true);
    set_var(MANDALA_PUMP3, 0, true);

    // 0 - fuel auto control
    // 1 - fuel manual control
    //set_var(MANDALA_ALGORITM, 0.0, true);

    startTimerPumpON = time();

    serial_listen(PORT_RS485, "@sensorHandler");

    set_var(f_user1, 15.7, true);
    set_var(f_user2, 15.7, true);
    set_var(f_user3, 15.7, true);

    return 0;
}

/*
Float:interpolate(Float:val, Float:x_min, Float:x_max, Float:y_min, Float: y_max)
{
  // y = kx + b;
  new Float: k = (y_max - y_min) / (x_max - x_min);
  new Float: b = y_min - (x_min * k);
  return val * k + b;
}
*/
//---------------------------------------------------------------
fuel_auto_control()
{
    if(m_vFuelPersent3 > 50 && !m_pump3) // > 50% in tank 3
    {
      set_var(MANDALA_PUMP3, 1, true);
    }
    else if (m_vFuelPersent1 > 50)  // > 50 % in tank 1 AND > 5% in tank 3
    {
      //1 and 3 must work alternately
    }    
    else if (m_vFuelPersent1 < 50)  // > 50 % in tank 1 AND > 5% in tank 3
    {
      //1 and 2 must work alternately
    }
    else
    {
      //get fuel from tank 2
    }
  //todo: turn on pump3 if tank3 >5%

}

fuel_manual_control()
{
/*    //check state pump1
    if(m_pump1 && !m_start_pump1){
      m_start_pump1 = true;
      startTimerPumpON = time();
      printf("VM:FP1 manual ON...\n");
    }else if(!m_pump1 && m_start_pump1){
      m_start_pump1 = false;
      printf("VM:FP1 manual OFF...\n");
    }

    //check time pump ON
    if(((time()-startTimerPumpON) > TIMER_MC * 1000) && m_pump1){
      printf("VM:Timer STOP...\n");
      set_var(MANDALA_PUMP1, 0.0, true);
    } */
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

@OnTask()
{
    //test fuel sensor
    /*new Float: ctr_thr = get_var(f_rc_throttle);
    if(ctr_thr < 0.01)
      ctr_thr = 0.01;
    set_var(f_user2, get_var(f_user2) - 0.05 * ctr_thr, true);
    if(get_var(f_user2) < 0.1)
      set_var(f_user2, 0.1, true);
    if(m_pump1 && get_var(f_user1) > 0.01) {
      //set_var(f_user1, get_var(f_user1) - 0.1, true);
      vFuel1 -= 0.01;
      set_var(f_user2, get_var(f_user2) + 0.01, true);
    }*/

    check_answer_time();


    new Float: fl1 = get_var(MANDALA_FUEL_V1);
    new Float: fl2 = get_var(MANDALA_FUEL_V2);
    new Float: fl3 = get_var(MANDALA_FUEL_V3);
    set_var(MANDALA_FUEL, fl1 + fl2 + fl3, true);

    m_vFuelPersent1 = fl1 * 100 / V_MAX;
    m_vFuelPersent2 = fl2 * 100 / V_MAX;
    m_vFuelPersent3 = fl3 * 100 / V_MAX;

    //if(m_vFuelPersent2 < 1.0)
    //  m_vFuelPersent2 = 1.0;
    //fuel_ratio = m_vFuelPersent1 / m_vFuelPersent2;
    //set_var(MANDALA_FUEL_RATIO, fuel_ratio, true);

    m_ignition = get_var(MANDALA_IGNITION);
    m_algoritm = get_var(MANDALA_ALGORITM);
    m_ers = get_var(MANDALA_ERS);
    //m_pump1 = get_var(MANDALA_PUMP1);
    //m_pump2 = get_var(MANDALA_PUMP2);
    //m_pump3 = get_var(MANDALA_PUMP3);

    if (m_ers){
          set_var(MANDALA_PUMP1, 0, true);
          set_var(MANDALA_PUMP2, 0, true);
          set_var(MANDALA_PUMP3, 0, true);
    }else{

        if(m_ignition != m_ignitionOld){
            m_ignitionOld = m_ignition;

            /*if(!m_ignition && m_pump2){
              set_var(MANDALA_PUMP2, 0, true);
            } else if(m_ignition && !m_pump2) {
              set_var(MANDALA_PUMP2, 1, true);
            }*/
        }

        /*if(m_algoritm != m_algoritmOld){
            m_algoritmOld = m_algoritm;

            if(m_algoritm && m_pump1){
              set_var(MANDALA_PUMP1, 0, true);
            }
        } */

        if(!m_algoritm){
          fuel_auto_control();
        }else{
          //fuel_manual_control();
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

    return DELAY_MS;
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
                    new Float: m_vFuel1 = (data{4} | (data{5} << 8)) / 10.0;
                    set_var(MANDALA_FUEL_V1, m_vFuel1, true);
                    //printf("sens1: %f\r\n", m_vFuel1);
                }

                if(data{1} == ADR_SENS2) {
                    m_timeAns2 = time();
                    new Float: m_vFuel2 = (data{4} | (data{5} << 8)) / 10.0;
                    set_var(MANDALA_FUEL_V2, m_vFuel2, true);
                    //printf("sens2: %f\r\n", m_vFuel2);
                }

                if(data{1} == ADR_SENS3) {
                    m_timeAns3 = time();
                    new Float: m_vFuel3 = (data{4} | (data{5} << 8)) / 10.0;
                    set_var(MANDALA_FUEL_V3, m_vFuel3, true);
                    //printf("sens3: %f\r\n", m_vFuel3);
                }
            }
        }
    }
}

