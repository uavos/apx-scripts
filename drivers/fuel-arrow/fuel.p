//v1.6 fuel ballance script
#define PORT_RS485      240
#define ADR_SENS1       75
#define ADR_SENS2       76
#define PACK_SIZE       9

//editable
#define V_MAX           32.0    //liters

#define P1_TANK2        5.0     //%
#define P2_TANK2        16.0    //%
#define V_BLOCK_PUMP    1.0     //%

#define KF1_RATIO       1.0
#define KF2_RATIO       3.0     //tank1 = 3 * tank2

new Float: vFuel1 = V_MAX; //value from sensor
#define PUMP_SPEED      0.03722 // liters / sec
new bool: estimate_fuel = true;

#define TIME_HYST       1       //sec
#define TIME_SA         2       //sec
#define TIMER_MC        50      //sec

#define DELAY_MS        200
#define AVG_MS          15000
#define AVG_N           AVG_MS/DELAY_MS

new Float: avg_window[AVG_N];
new Float: avg_summ;

new MANDALA_MAVG = f_user4;

//read
new MANDALA_IGNITION = f_power_ignition;
new MANDALA_ERS = f_ctrb_ers;
new MANDALA_ALGORITM = f_userb_1;

//write
new MANDALA_PUMP1 = f_userb_2;
new MANDALA_PUMP2 = f_userb_3;
new MANDALA_FUEL = f_fuel;
new MANDALA_FUEL_V1_est = f_user1;
new MANDALA_FUEL_V2 = f_user2;
new MANDALA_FUEL_RATIO = f_user3;

new MANDALA_WARN_ANSWER1 = f_radar_Vy;
new MANDALA_WARN_ANSWER2 = f_radar_Vz;

new msg{PACK_SIZE};
new bool: m_sensRequest = false;    //false - sens1; true - sens2;

new m_ers;
new m_algoritm;   //0 - enable; 1 - disable
new m_algoritmOld;

new m_ignition;
new m_ignitionOld;

new m_pump1 = 0;
new m_pump2 = 0;
new pump_stage = -1;

new Float: kf_start = 0.0;
new Float: kf_stop = 0.0;
new Float: m_vFuelPersent1;
new Float: m_vFuelPersent2;
new Float: fuel_ratio;
new m_timeAns1;
new m_timeAns2;

new Float: m_timeDeltaPump1;
new m_timeDeltaPump1Active = 0;

new startTimerPumpON;
new bool:m_start_pump1;

main()
{
    m_ignitionOld = false;
    m_algoritmOld = false;
    m_start_pump1 = false;

    set_var(MANDALA_PUMP1, 0, true);
    set_var(MANDALA_PUMP2, 0, true);

    // 0 - fuel auto control
    // 1 - fuel manual control
    set_var(MANDALA_ALGORITM, 0.0, true);

    startTimerPumpON = time();

    serial_listen(PORT_RS485, "@sensorHandler");

    set_var(f_user1, 32, true);
    set_var(f_user2, 32, true);

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
    new localTime = time();

    if(m_vFuelPersent2 > P2_TANK2 && !m_pump1) {
      pump_stage = 1;
      kf_start = KF2_RATIO;
      kf_stop = kf_start - 0.5;
    } else if(m_vFuelPersent1 > P2_TANK2 && !m_pump1) {
      pump_stage = 2;
      //kf_start = interpolate(m_vFuelPersent2, P1_TANK2, P2_TANK2, KF1_RATIO, KF2_RATIO);
      kf_start = 0;//not used
      kf_stop = 0; //not used
    } else if(!m_pump1) {
      pump_stage = 3;
      kf_start = 1.1;
      kf_stop = 0.8;
    }


    //start pump1
    if(m_vFuelPersent1 > V_BLOCK_PUMP && !m_pump1) {

      new bool:start_pump = false;

      if(fuel_ratio > kf_start && (pump_stage == 1 || pump_stage == 3)) {
        start_pump = true;
      }

      if(pump_stage == 2 && m_vFuelPersent2 < P2_TANK2 * 0.9) {
        start_pump = true;
      }

      if(start_pump) {
        if(!m_timeDeltaPump1Active) {
          m_timeDeltaPump1Active = true;
          m_timeDeltaPump1 = localTime;
        } else if(m_timeDeltaPump1 + TIME_HYST*1000 < localTime) {
          m_timeDeltaPump1Active = false;
          printf("VM:FP1 start:%.2f...\n", kf_start);
          set_var(MANDALA_PUMP1, 1, true);
        }
      }
    } else {
      m_timeDeltaPump1Active = false;
    }


    //stop pump1
    if(m_pump1) {

      new bool:stop_pump = false;

      if((pump_stage == 1 || pump_stage == 3) && fuel_ratio < kf_stop) {
        stop_pump = true;
      }

      if(pump_stage == 2 && m_vFuelPersent2 > P2_TANK2 * 1.1) {
        stop_pump = true;
      }

      if((m_vFuelPersent2 > m_vFuelPersent1) && (pump_stage != 3)) {
        stop_pump = true;
      }

      if(stop_pump) {
        printf("VM:FP1 stop:%.2f...\n", kf_stop);
        set_var(MANDALA_PUMP1, 0, true);
      }
    }
}

fuel_manual_control()
{
    //check state pump1
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
      set_var(MANDALA_PUMP1, 0, true);
    }
}

moving_average()
{
    for(new i=0; i<AVG_N-1; i++){
      avg_window[i] = avg_window[i+1];
    }

    avg_window[AVG_N-1] = get_var(f_vspeed);

    avg_summ = 0.0;
    for(new i=0; i<AVG_N; i++){
      avg_summ += avg_window[i];
    }

    set_var(MANDALA_MAVG, avg_summ / float(AVG_N), true);
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

@OnTask()
{
    //estimate fuel volume in tank 1
    if(((get_var(MANDALA_MAVG) > -0.3) && (get_var(MANDALA_MAVG) < 0.3)) || estimate_fuel == false)
        set_var(MANDALA_FUEL_V1_est, vFuel1, true);
    else if(m_pump1 && get_var(MANDALA_FUEL_V1_est) > 0.1)
        set_var(MANDALA_FUEL_V1_est, get_var(MANDALA_FUEL_V1_est) - PUMP_SPEED * DELAY_MS / 1000.0, true);

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

    //check answer time from sensor
    if(m_timeAns1+TIME_SA*1000 < time())
      set_var(MANDALA_WARN_ANSWER1, 1, true);
    else
      set_var(MANDALA_WARN_ANSWER1, 0, true);

    if(m_timeAns2+TIME_SA*1000 < time())
      set_var(MANDALA_WARN_ANSWER2, 1, true);
    else
      set_var(MANDALA_WARN_ANSWER2, 0, true);

    moving_average();

    new Float: fl1 = get_var(MANDALA_FUEL_V1_est);
    new Float: fl2 = get_var(MANDALA_FUEL_V2);
    set_var(MANDALA_FUEL, fl1 + fl2, true);

    m_vFuelPersent1 = fl1 * 100 / V_MAX;
    m_vFuelPersent2 = fl2 * 100 / V_MAX;
    if(m_vFuelPersent2 < 1.0)
      m_vFuelPersent2 = 1.0;
    fuel_ratio = m_vFuelPersent1 / m_vFuelPersent2;
    set_var(MANDALA_FUEL_RATIO, fuel_ratio, true);

    m_ignition = get_var(MANDALA_IGNITION);
    m_ers = get_var(MANDALA_ERS);
    m_algoritm = get_var(MANDALA_ALGORITM);
    m_pump1 = get_var(MANDALA_PUMP1);
    m_pump2 = get_var(MANDALA_PUMP2);

    if (m_ers){
        if(m_pump1){
          set_var(MANDALA_PUMP1, 0, true);
        }
        if(m_pump2){
          set_var(MANDALA_PUMP2, 0, true);
        }
    }else{

        if(m_ignition != m_ignitionOld){
            m_ignitionOld = m_ignition;

            if(!m_ignition && m_pump2){
              set_var(MANDALA_PUMP2, 0, true);
            } else if(m_ignition && !m_pump2) {
              set_var(MANDALA_PUMP2, 1, true);
            }
        }

        if(m_algoritm != m_algoritmOld){
            m_algoritmOld = m_algoritm;

            if(m_algoritm && m_pump1){
              set_var(MANDALA_PUMP1, 0, true);
            }
        }

        if(!m_algoritm){
          fuel_auto_control();
        }else{
          fuel_manual_control();
        }
    }

    msg{0} = 0x31;
    msg{2} = 0x06;
    msg{1} = m_sensRequest ? ADR_SENS1 : ADR_SENS2;
    msg{3} = calcCrc(msg, 3);
    serial_write(PORT_RS485, msg, 4, serialmode:NODE);

    m_sensRequest = !m_sensRequest;

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
                    vFuel1 = (data{4} | (data{5} << 8)) / 10.0;
                    //printf("sens1: %f\r\n", m_vFuel1);
                }

                if(data{1} == ADR_SENS2) {
                    m_timeAns2 = time();
                    new Float: m_vFuel2 = (data{4} | (data{5} << 8)) / 10.0;
                    set_var(MANDALA_FUEL_V2, m_vFuel2, true);
                    //printf("sens2: %f\r\n", m_vFuel2);
                }
            }
        }
    }
}

forward @estfuel()
@estfuel()
{
    estimate_fuel = !estimate_fuel;
    if(estimate_fuel)
        printf("fuel1 estimation on");
    else
        printf("fuel1 estimation off");
}
