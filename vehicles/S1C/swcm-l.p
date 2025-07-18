#define PORT_ID_ECU 60
#define CAN_PACK_SIZE 13

const CAN_BASE_ADDR  = 1520;
const OFFSET_CAN_ID_CURRENT_THROTTLE = 71
const CAN_ID_CURRENT_THROTTLE = CAN_BASE_ADDR + OFFSET_CAN_ID_CURRENT_THROTTLE

new data{13} = "";

main()
{
  serial_listen(PORT_ID_ECU,"@OnSerialECU");
  setThrottleMS();
}

@OnTask()
{
  sweep_control();

  setThrottleMS();

  //Calc LAMBDA
  set_var(f_ils_HDG, get_var(f_VM1) * 14.7, true)

  return 200;
}

sweep_control()
{
  if(get_var(f_VM10) || get_var(f_VM11)) {
    set_var(f_VM10, 0.0, false);
    set_var(f_VM11, 0.0, false);
  }
  new Float: ctr_sweep = get_var(f_ctr_sweep);

  if(ctr_sweep < -0.5) {
    set_var(f_VM10, 1.0, false);
    set_var(f_ctr_sweep, 0.0, true);
  }
  if(ctr_sweep > 0.5) {
    set_var(f_VM11, 1.0, false);
    set_var(f_ctr_sweep, 0.0, true);
  }
}

setThrottleMS()
{
  data{0} = CAN_ID_CURRENT_THROTTLE & 0xFF;
  data{1} = (CAN_ID_CURRENT_THROTTLE >> 8 ) & 0xFF;
  data{2} = (CAN_ID_CURRENT_THROTTLE >> 16) & 0xFF;
  data{3} = (CAN_ID_CURRENT_THROTTLE >> 24) & 0xFF;
  data{4} = 5;

  new Float: ch_throttle = 0.0;
  //printf("thr:%f/n", ch_throttle);
  serializeFloat(data, 5, ch_throttle);
  data{9} = 1;
  serial_write(PORT_ID_ECU, data,  10, serialmode:NODE);
}

serializeFloat(data2{}, index ,Float: value)
{
  for(new i = 0; i < 4; i++)
  {
     new shift = 8 * i;
     data2{index + i} = (value >> shift) & 0xFF;
  }
}

forward @OnSerialECU(cnt)
@OnSerialECU(cnt)
{
  if((cnt > CAN_PACK_SIZE) || (cnt != (serial_byte(4) & 0x7F) + 5)) {
    return;
  }

  new can_id = serial_byte(0) + (serial_byte(1) << 8) + (serial_byte(2) << 16) + (serial_byte(3) << 24);
  //printf("can_id:%x\n", can_id);

  switch(can_id)
  {
    case CAN_BASE_ADDR+2:
    {
      new param;

      param = (serial_byte(9) << 8) + serial_byte(10);
      if (serial_byte(9) & 0x80)
        param += (0xffff << 16);
      set_var(f_OT,(float(param)/10.0 - 32.0)*5.0/9.0,true); // Oil temperature [deg]


      param = (serial_byte(11) << 8) + serial_byte(12);
      if (serial_byte(11) & 0x80)
        param += (0xffff << 16);
      set_var(f_ET,(float(param)/10.0 - 32.0)*5.0/9.0,true); // Coolant temperature [deg]
    }


    case CAN_BASE_ADDR+15:
    {
      new Float: param;
      param = (serial_byte(5) << 8) + serial_byte(6);
      if (serial_byte(5) & 0x80)
         param += (0xffff << 16);
      //set_var(f_OP,(float(param)*0.02822581)-3.27822581, true); //Oil pressure
      set_var(f_OP,(float(param)*0.02083333)-7.02083333, true);
    }

    case CAN_BASE_ADDR+22:
    {
      new Float:egt[4];
      for (new i = 0; i < 4; i++)
      {
        new idx = i*2+5;
        egt[i] = (serial_byte(idx) << 8) + serial_byte(idx+1); // EGT1 deg F s
        if (serial_byte(idx) & 0x80)
          egt[i] += (0xffff << 16);
      }

      set_var(f_radar_dx, egt[0]/10, true);     //EGT1
      set_var(f_radar_dy, egt[1]/10,true);      //EGT2
      set_var(f_user4, egt[2]/10, true);        //EGT3
      set_var(f_user5, egt[3]/10, true);        //EGT4
   }
  }
}
