#define V_CC    3.297
#define R_PUP   3300.0

#define TABLE_SIZE 43

new Float:TableTemp[] = [
	-55.0, -50.0, -45.0, -40.0, -35.0,
	-30.0, -25.0, -20.0, -15.0, -10.0,
	-5.0,	  0.0,   5.0,  10.0,  15.0,
	20.0,   25.0,  30.0,  35.0,  40.0,
	45.0,   50.0,  55.0,  60.0,  65.0,
	70.0,   75.0,  80.0,  85.0,  90.0,
	95.0,  100.0, 105.0, 110.0, 115.0,
	120.0, 125.0, 130.0, 135.0, 140.0,
	145.0, 150.0, 155.0
];


new Float:TableRes[] = [
	963000.0, 670100.0, 471700.0, 336500.0, 242600.0,
	177000.0, 130400.0,  97070.0,  72930.0,  55330.0,
	 42320.0,  32650.0,  25390.0,  19900.0,  15710.0,
	 12490.0,  10000.0,   8057.0,   6531.0,   5327.0,
	  4369.0,   3603.0,   2986.0,   2488.0,   2083.0,
	  1752.0,   1481.0,   1258.0,   1072.0,    917.7,
	   788.5,    680.0,    588.6,    511.2,    445.4,
	   389.3,    341.7,    300.9,    265.4,    234.8,
	   208.3,    185.3,    165.3
];

new Float: v_t = 0.0;
new Float: r_t = 0.0;
new Float: i_t = 0.0;

new Float: temp = 0.0;
new Float: persent = 0.0;

#define TASK_DELAY_MS   100

Float:calc_temperature(idx_var)
{
    new indexL = 0;
    new indexH = 0;
    v_t = get_var(idx_var);
    i_t = (V_CC-v_t)/R_PUP;

    if((v_t/963000.0)<i_t){
        r_t = v_t/i_t
    }else{
        r_t = 963000;
    }

    for(new i = 1; i < TABLE_SIZE; i++){
        if(r_t <= TableRes[i-1] && r_t >= TableRes[i]){
            indexH = i-1;
            indexL = i;
            break;
        }
    }

    if(indexL != indexH){
        persent = (r_t - TableRes[indexL]) / (TableRes[indexH] - TableRes[indexL]);
        temp = TableTemp[indexL] - persent*(TableTemp[indexL] - TableTemp[indexH]);
    }

    return temp;
}

@OnTask()
{
    set_var(f_radar_Vx, calc_temperature(f_VM1), true);
    set_var(f_radar_Vy, calc_temperature(f_VM2), true);
    set_var(f_radar_Vz, calc_temperature(f_VM3), true);

    return TASK_DELAY_MS;
}
