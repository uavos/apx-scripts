const LOOKHERE_PORT_ID = 123;

const DATA_SIZE = 8;

new g_tp;

packFloat(data{}, index, Float:v)
{
    for(new i = 0; i < 4; i++)
      data{index + i} = (v >> (8 * i)) & 0xFF;
}

sendLookHereCommands()
{
    new data{DATA_SIZE};

    packFloat(data, 0, get_var(f_camctr_yaw));
    packFloat(data, 4, get_var(f_camctr_pitch));

    new bool:result = serial_write(LOOKHERE_PORT_ID, data, DATA_SIZE, serialmode:LAN);
    if(!result)
        printf("sendLookHereCommands error")
}

main()
{
    g_tp = time();
}

@OnTask()
{
    new now = time();

    if(now - g_tp >= 50) {
        g_tp = now;

        if(get_var(f_cam_mode == 4)) {
            sendLookHereCommands();
        }
    }

    return 10;
}