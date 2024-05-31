new Float: Vadc=0.0;
new Float: Vbias=3.113208; //25 degree
new Float: TMP_k=0.0;
new Float: bat_temp_right = 0.0;

@OnTask()
{
    Vadc = get_var(f_VM1);
    if (Vadc >= Vbias)  TMP_k=0.000987
    else                TMP_k=0.002165;
    bat_temp_right = (Vadc-Vbias)/TMP_k+25;
    set_var(f_radar_Vz, bat_temp_right, true);
    return(500);
}






















