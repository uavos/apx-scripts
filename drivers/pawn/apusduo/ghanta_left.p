new Float: Vadc=0.0;
new Float: Vbias=3.113208; //25 degree
new Float: TMP_k=0.0;
new Float: bat_temp_left = 0.0;

#define PORT_ID 232
new frame_length;
new buffer{128} = {};

main()
{
  frame_length = 0;
  serial_listen(PORT_ID, "@OnSerial");
  return 0
}

@OnTask()
{
    Vadc = get_var(f_VM1);
    if (Vadc >= Vbias)  TMP_k=0.000987
    else                TMP_k=0.002165;
    bat_temp_left = (Vadc-Vbias)/TMP_k+24;
    set_var(f_radar_Vx, bat_temp_left, true);
    return(500);
}


forward @OnSerial(cnt)
@OnSerial(cnt)
{
  if(cnt >= 128) {
    return;
  }

  for (new i = 0; i < cnt; i++)
  {
    buffer{frame_length++} = serial_byte(i);
  }

  if (frame_length)
  {
    serial_write(PORT_ID, buffer, frame_length, serialmode:GCU);
    frame_length = 0;
  }
}
