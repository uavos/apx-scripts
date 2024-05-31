new Float: Vadc=0.0;
new Float: Vadc_PY=0.0;

new Float: Vbias=3.584; //25 degree
new Float: TMP_k=0.0;

//const Float:HEATING_RESIST = 15.6;
const Float:HEATING_RESIST = 34.0;
new g_tpStartHeating = 0;
new g_tpHeatingInterval = 0;
new Float:maxPower = 0.0;
new Float:bat_temp_cl = 0.0;
new Float:bat_temp_cr = 0.0;



//====auto power throttle====
new last_time_throttle=0;

new Float: pwr_out_mppt=0.0;

new Float: pwr_esc1=0.0;
new Float: pwr_esc2=0.0;
new Float: pwr_esc3=0.0;
new Float: pwr_esc=0.0;

new pwr_cmd=0;

new Float: new_pwr_throttle=0.0;
new new_thr_itr=0;
//===========================

new time_precharge=0;
new ok_precharge=false;
new on_power_payload=false;
//===========================



main()
{
    g_tpStartHeating = time();
    last_time_throttle = time();
    time_precharge = time();
}

@OnTask()
{
    new timeNow = time();
    if(timeNow - g_tpStartHeating > 1000)
    {
        Vadc_PY = (get_var(f_VM4))*3.51562500+0.06835938;
        set_var(f_turret_pitch,Vadc_PY*get_var(f_Vm),true);


        Vadc = get_var(f_VM1);
        if (Vadc >= Vbias)  TMP_k=0.007142
        else                TMP_k=0.010883;

        bat_temp_cr = (Vadc-Vbias)/TMP_k+25;

        set_var(f_ils_DME, 0, true); //no temperature sensor
        //set_var(f_ils_DME,bat_temp_cr, true);


        Vadc = get_var(f_VM2);
        if (Vadc >= Vbias)  TMP_k=0.007142
        else                TMP_k=0.010883;

        bat_temp_cl = (Vadc-Vbias)/TMP_k+25;
        set_var(f_radar_Vy, 0, true); //no temperature sensor
        //set_var(f_radar_Vy, bat_temp_cl, true);
        set_var(f_AT,bat_temp_cl, true);


        if(get_var(f_radar_Vx)>=50.0 || get_var(f_radar_Vy)>=50.0 || get_var(f_ils_DME)>=50.0 || get_var(f_radar_Vz)>=50.0){
        //if(get_var(f_radar_Vx)>=50.0 || get_var(f_radar_Vz)>=50.0){
          set_var(f_user4, 0, true);
        }
        if(get_var(f_user4)>0)
        {
            new Float:voltage = get_var(f_Vm);
            maxPower = voltage * voltage / HEATING_RESIST;
            new Float:k = get_var(f_user4) / maxPower;
            if(k > 1.0)
                set_var(f_platform_hmsl,maxPower,true);
            else
                set_var(f_platform_hmsl,get_var(f_user4),true);
            g_tpHeatingInterval = 1000 * k;
            set_var(f_userb_2,1,true);
            g_tpStartHeating = timeNow;
        } else {
          set_var(f_platform_hmsl,0.0,true);
        }
    }
    if(timeNow-g_tpStartHeating>g_tpHeatingInterval){
      set_var(f_userb_2,0,true);
    }


    new Float: d_time=(timeNow-last_time_throttle)/1000.0;        //мсек -> сек

    if(get_var(f_sw_sw1) && get_var(f_cmode_throvr)){
        new_pwr_throttle=calc_power_thr(d_time);
        if(new_thr_itr++>=20){
          new_thr_itr=0;
          set_var(f_rc_throttle, new_pwr_throttle, true);
        }
    }
    last_time_throttle=timeNow;

    if (get_var(f_sw_sw3) && !ok_precharge && !on_power_payload) {
      ok_precharge = true;
      on_power_payload = true;
      time_precharge = timeNow;
      set_var(f_VM31, 1.0, false);
      printf("VM:Precharge on...\n");
    }

    if((timeNow-time_precharge) > 3000 && ok_precharge) {
      ok_precharge = false;
      set_var(f_VM32, 1.0, false);
      wait(2000);
      set_var(f_VM31, 0.0, false);
      printf("VM:Precharge off...\n");
      printf("VM:Payload SoftBank on...\n");
    }

    if(on_power_payload && !get_var(f_sw_sw3)) {
      on_power_payload = false;
      set_var(f_VM32, 0.0, false);
      set_var(f_VM31, 0.0, false);
      printf("VM:Payload SoftBank off...\n");
    }

    return 5;
}


new Float: Q=0.01;
new Float: R=5.0;
new Float: F=1.0;
new Float: H=1.0;

new Float: X0=0.0;
new Float: P0=0.0;

new Float: State=0.0;
new Float: Covariance=1.0;
new Float: K=0.0;

Float: KalmanCorrect(Float:signal)
{
  X0=K*State;
  P0=F*Covariance*F+Q;

  K=H*P0/(H*P0*H+R);
  State=X0+K*(signal-H*X0);
  Covariance=(1.0-K*H)*P0;

  return State;
}


Float:TrimSignal(Float:signal, Float:min, Float:max)
{
  if(signal<min){
    return min;
  }

  if(signal>max){
    return max;
  }

  return signal;
}


new Float: kp=0.02;  //0.01
new Float: lp=20.0;
new Float: ki=0.01;  //0.005
new Float: li=100.0;
new Float: lo=100.0;
new Float: cur_in_summ=0.0;
new Float: prev_value_error=0.0;


new tmp1=0;
Float:calc_power_thr(Float:time_elapsed)
{
  //P=I*U
  pwr_esc1=get_var(f_user5)*get_var(f_platform_Vnorth);
  pwr_esc2=get_var(f_user6)*get_var(f_platform_Veast);
  pwr_esc3=get_var(f_range)*get_var(f_platform_Vdown);

  pwr_esc=pwr_esc1+pwr_esc2+pwr_esc3;
  if(get_var(f_sw_sw4)) {
    //P=P1+P2+P3+P_BAT+P_AP+P_Payload
    pwr_esc+=6*get_var(f_platform_hmsl)+25.0+get_var(f_turret_pitch);
  }

  //MPPT Power
  pwr_out_mppt=get_var(f_platform_hdg);

  //баланс, если pwr_cmd > 0, то зяряд аккумулятора
  pwr_cmd=get_var(f_user1);

  //calc power error
  new Float: pwr_error=pwr_out_mppt-pwr_esc-pwr_cmd;

  //use kalman filter
  if(get_var(f_userb_3)){
    pwr_error=KalmanCorrect(pwr_error);
  }

  new Float: output=0.0;
  //P
  output=TrimSignal(pwr_error*kp,-lp,lp); //P

  //I
  if(ki>0) {
    cur_in_summ+=((pwr_error+prev_value_error)/2.0*time_elapsed*ki);
    cur_in_summ=TrimSignal(cur_in_summ,0,li);
    output+=cur_in_summ;
  } else {
    cur_in_summ=0.0;
  }

  output=TrimSignal(output,0,lo);
  prev_value_error=pwr_error;


  //if(tmp1++>=100) {
     //tmp1=0;
     //printf("output_thr=%f\n", output/100.0);
     //printf("pwr_er=%f\n", pwr_error);
     //printf("output=%f\n", output);
     //printf("=========================");
  //}

  return output/100.0;
}
