@OnTask()
{
  //user6 - rpm
  new rpm = get_var(f_user6);

  if((get_var(f_ET)>95 || get_var(f_OT)>105) && (rpm>200)) {
    set_var(f_turret_sw1, 1.0, true);
  }

  if((get_var(f_ET)<92 && get_var(f_OT)<101 && get_var(f_mode)) || rpm<200) {
    set_var(f_turret_sw1, 0.0, true);
  }

  if((get_var(f_ET)>100 || get_var(f_OT)>99) && (rpm>200)) {
    set_var(f_turret_sw2, 1.0, true);
  }

  if((get_var(f_ET)<96 && get_var(f_OT)<96 && get_var(f_mode)) || rpm<200) {
    set_var(f_turret_sw2, 0.0, true);
  }

  return 3000;
}
