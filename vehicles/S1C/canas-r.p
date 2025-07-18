#define DELAY_MS        100

#define PORT_RS485      91


#define ADR_SENS1       80
#define ADR_SENS2       81
#define ADR_SENS3       82

#define PACK_SIZE       9


new fuel_timer = 0;
const FUEL_TIMEOUT = 1000;

new Float: fuel_lavel[3];


main()
{
    new now = time();
    fuel_timer = now;

    serial_listen(PORT_RS485, "@fuelHandler");

    return 0;
}

@OnTask()
{
    new now = time();
    if (now - fuel_timer > FUEL_TIMEOUT) {
        fuel_timer = now;
        fuel_proc();
    }

    return DELAY_MS;
}

fuel_proc()
{
    new Float: value = fuel_lavel[0] + fuel_lavel[1] + fuel_lavel[2];
    set_var(f_fuel, value, true);
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

forward @fuelHandler(cnt);
@fuelHandler(cnt)
{
    if (cnt != PACK_SIZE) {
        return;
    }

    if (serial_byte(0) != 0x3E && serial_byte(2) != 0x06) {
        return;
    }

    new data{PACK_SIZE};
    for(new i = 0; i < PACK_SIZE; i++)
        data{i} = serial_byte(i);

    if(data{8} != calcCrc(data, 8)){
        return;
    }

    new Float: value = (data{4} | (data{5} << 8)) / 10.0;

    if(data{1} == ADR_SENS1) {
        fuel_lavel[0] = value;
    }

    if(data{1} == ADR_SENS2) {
        fuel_lavel[1] = value;
    }

    if(data{1} == ADR_SENS3) {
        fuel_lavel[2] = value;
    }
}

//vmexec ('@vm_fuel')    //Console
//@vm_fuel               //Waypoints->actions->script
forward @vm_fuel()
@vm_fuel()
{
    printf("fuel: %.1f : %.1f : %.1f\r\n", fuel_lavel[0], fuel_lavel[1], fuel_lavel[2]);
}
