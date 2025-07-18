//======================AGL========================
#define AGL_CAN_ID 0x00090002
#define PORT_ID_AGL 80
#define PACK_SIZE 13

new idx_agl = f_agl;      // agl
//new idx_snr = f_user1;  // snr

//======================IRIDIUM========================
const PORT_ID_SAT = 19;
const SBDI_TIMEOUT = 1000 * 30;
const WAIT_RESPONSE_TIMEOUT = 1000 * 60;

const MAX_RESPONSE_SIZE = 100;
new g_response{MAX_RESPONSE_SIZE};
new g_responseSize = 0;

new bool:g_waitResponse = false;
new bool:g_needClearBuffer = false;
new g_timePoint = 0;

new g_state = 100;
/*
 * g_state codes:
 *
 * 0 - start AT+CSQ         : signal level
 * 1 - start AT+SBDLOE      : remaining time to SBDI
 * 2 - start AT+SBDI        : start session
 *
 * 10 - start AT+SBDRB      : read incoming message from 9602
 *
 * 20 - 1st stage AT+SBDWB  : prepare to send telemetry
 * 21 - 2nd stage AT+SBDWB  : send telemetry
 *
 * 30 - start AT+SBDD0      : clear MO buffer
 *
 * 100 - start AT+GSN       : check link with 9602, get serial number
 */

main()
{
  wait(1000);
  serial_listen(PORT_ID_AGL,"@OnAGLHandler");
  serial_listen(PORT_ID_SAT, "@OnSATHandler");
}

forward @OnAGLHandler(cnt)
@OnAGLHandler(cnt)
{
   if(cnt != PACK_SIZE) {
     return
  }

  new can_id = serial_byte(0) + (serial_byte(1)<<8) + (serial_byte(2)<<16) + (serial_byte(3)<<24);

  switch(can_id)
  {
    case AGL_CAN_ID:
    {
      new Float: altitude = ((serial_byte(5)<<8) + serial_byte(6)) / 100.0;
      if (altitude > 0.1 && altitude < 40.0)
        set_var(idx_agl,altitude, true);

      //new snr=(serial_byte(7)<<8)+serial_byte(8);
      //set_var(idx_snr, snr, true);
      return;
    }
  }
}


serializeFloat(arrayIndex, array{}, Float:v)
{
    for(new i = 0; i < 4; i++)
        array{arrayIndex + i} = (v >> (8 * i)) & 0xFF;
}

bool:parseResponse(okMark{}, okMarkSize)
{
    new okCounter = 0;
    for(new i = 0; i < g_responseSize; i++)
    {
        if(g_response{i} == okMark{okCounter})
            okCounter++;
        else
            okCounter = 0;
        if(okCounter == okMarkSize)
            return true;
    }
    return false;
}

parseCSQ()
{
    new pos = strfind(g_response, ":");
    if(pos < 0)
        return -1;
    new substr{1};
    strmid(substr, g_response, pos + 1, pos + 2);
    new value = strval(substr);
    return value;
}

bool:parseSBDLOE(result[2])
{
    new startPos = strfind(g_response, ":");
    if(startPos < 0)
        return false;

    startPos += 1;
    new statusStr{1};
    new timeStr{1};
    new endPos = strfind(g_response, ",", false, startPos);
    if(endPos < 0)
        return false;

    strmid(statusStr, g_response, startPos, endPos);
    startPos = endPos + 1;
    endPos = strfind(g_response, "\r", false, startPos);
    if(endPos < 0)
        return false;

    strmid(timeStr, g_response, startPos, endPos);

    result[0] = strval(statusStr);
    result[1] = strval(timeStr);
    return true;
}

bool:parseSBDI(result[2])
{
    new startPos = strfind(g_response, ":");
    if(startPos < 0)
        return false;
    startPos += 1;
    new sendOkStr{1};
    new receiveOkStr{1};
    for(new i = 0; i < 3; i++)
    {
        new endPos = strfind(g_response, ",", false, startPos);
        if(endPos < 0)
            return false;
        if(i == 0)
            strmid(sendOkStr, g_response, startPos, endPos);
        if(i == 2)
            strmid(receiveOkStr, g_response, startPos, endPos);
        startPos = endPos + 1;
    }
    result[0] = strval(sendOkStr);
    result[1] = strval(receiveOkStr);
    return true;
}

/*
 * return values:
 *
 * -1 - fail
 * 0 - all ok
 * 1 - need send telemetry
 */
parseSBDRB()
{
    if(g_responseSize < 13)
        return -1;
    new size = g_response{9} * 256 + g_response{10};
    if(size != 5)
        return -1;
    new cmd = g_response{11};
    new Float:value[1];
    new temp{4};
    temp{0} = g_response{15};
    temp{1} = g_response{14};
    temp{2} = g_response{13};
    temp{3} = g_response{12};
    memcpy(value, temp, 0, 4);
    new crc = g_response{16} * 256 + g_response{17};
    new calcCrc = 0;
    for(new i = 11; i < 16; i++)
        calcCrc += g_response{i};
    if(calcCrc != crc)
        return -1;
    if(cmd == 0)
    {
        set_var(f_cmd_altitude, value[0], true);
    }
    else if(cmd == 1)
    {
        set_var(f_mode, value[0], true);
    }
    else if(cmd == 2)
    {
        return 1;
    }
    else if(cmd == 3)
    {
        set_var(f_ctrb_ers, 1, true);
    }
    else if(cmd == 4)
    {
        set_var(f_mode, 0, true);
        set_var(f_rc_roll, 1, true);
        set_var(f_rc_pitch, 1, true);
        set_var(f_rc_yaw, 1, true);
    }
    return 0;
}

bool:parseGSN(serial{})
{
    new startPos = strfind(g_response, "\n");
    if(startPos < 0)
        return false;
    new endPos = strfind(g_response, "\r", false, startPos + 1);
    if(endPos < 0)
        return false;
    strmid(serial, g_response, startPos, endPos, 20);
    serial{endPos - startPos} = '\0';
    return true;
}

sendCommand(cmd{}, cnt)
{
    serial_write(PORT_ID_SAT, cmd, cnt, serialmode:NODE);
    g_waitResponse = true;
    g_timePoint = time();
}

sendTelemetry()
{
    new data{4 * 6}; //6 vars
    serializeFloat(0, data, get_var(f_gps_lat));
    serializeFloat(4, data, get_var(f_gps_lon));
    serializeFloat(8, data, get_var(f_airspeed));
    serializeFloat(12, data, get_var(f_altps));
    serializeFloat(16, data, get_var(f_Ve));
    serializeFloat(20, data, get_var(f_mode));
    new crc = 0;
    for(new i = 0; i < 24; i++)
    {
        crc += data{i};
    }
    serial_write(PORT_ID_SAT, data, 24, serialmode:NODE);
    new crcArray{2};
    crcArray{0} = (crc >> 8) & 0xFF;
    crcArray{1} = crc & 0xFF;
    serial_write(PORT_ID_SAT, crcArray, 2, serialmode:NODE);
}

forward @OnSATHandler(cnt)
@OnSATHandler(cnt)
{
    if(g_responseSize + cnt > MAX_RESPONSE_SIZE)
    {
      print("9602: buffer overflow");
      return;
    }
    for(new i = 0; i < cnt; i++)
    {
      g_response{i + g_responseSize} = serial_byte(i);
    }
    g_responseSize += cnt;
}

/*
@OnTask()
{
    if(g_state == 0 && g_needClearBuffer)
    {
        g_state = 30;
    }

    if(g_state == 0)
    {
        if(!g_waitResponse)
        {
            //print("9602: AT+CSQ");
            sendCommand("AT+CSQ\r", 7);
            return 1;
        }
        else if(g_responseSize)
        {
            if(parseResponse("OK\r\n", 4))
            {
                new signal = parseCSQ();
                //printf("9602: CSQ: %d\n", signal);
                if(signal >= 2)
                    g_state = 1;

                g_waitResponse = false;
                g_responseSize = 0;
                return 1;
            }
        }
    }
    else if(g_state == 1)
    {
        if(!g_waitResponse)
        {
            //print("9602: AT+SBDLOE");
            sendCommand("AT+SBDLOE\r", 10);
            return 1;
        }
        else if(g_responseSize)
        {
            if(parseResponse("OK\r\n", 4))
            {
                new returnValue = 1;
                new result[2];
                if(parseSBDLOE(result))
                {
                    //printf("9602: SBDLOE: %d %d\n", result[0], result[1]);
                    if(result[0] == 0) //status ok
                    {
                        if(result[1] == 0)
                        {
                            g_state = 2; //remaining time = 0, start session
                        }
                        else
                        {
                            g_state = 0; // wait for result[1] s and try again all steps
                            returnValue = result[1] * 1000;
                        }
                    }
                    else
                    {
                        g_state = 0;
                    }
                }

                g_waitResponse = false;
                g_responseSize = 0;
                return returnValue;
            }
        }
    }
    else if(g_state == 2)
    {
        if(!g_waitResponse)
        {
            //print("9602: AT+SBDI");
            sendCommand("AT+SBDI\r", 8);
            return 1;
        }
        else if(g_responseSize)
        {
            if(parseResponse("OK\r\n", 4))
            {
                new returnValue = 1;
                new result[2];
                if(parseSBDI(result))
                {
                    //printf("9602: SBDI: %d %d\n", result[0], result[1]);
                    if(result[0] == 2 || result[1] == 2) //session fail, try again all steps
                    {
                        g_state = 0;
                    }
                    else if(result[1] == 1) //have incoming message, need AT+SBDRB
                    {
                        g_state = 10;
                    }
                    else //all ok, no messages, next session after SBDI_TIMEOUT ms
                    {
                        g_state = 0;
                        returnValue = SBDI_TIMEOUT;
                    }

                    if(result[0] == 1) //we send msg from MO, need clear MO later
                    {
                        g_needClearBuffer = true;
                    }
                }
                else
                {
                    g_state = 0;
                }

                g_waitResponse = false;
                g_responseSize = 0;
                return returnValue;
            }
        }
    }
    else if(g_state == 10)
    {
        if(!g_waitResponse)
        {
            //print("9602: AT+SBDRB");
            sendCommand("AT+SBDRB\r", 9);
            return 1;
        }
        else if(g_responseSize)
        {
            if(parseResponse("OK\r\n", 4))
            {
                new returnValue = 1;
                new result = parseSBDRB();
                //printf("9602: SBDRB: %d\n", result);
                if(result == 1)
                    g_state = 20;
                else
                {
                    g_state = 0;
                    returnValue = SBDI_TIMEOUT;
                }

                g_waitResponse = false;
                g_responseSize = 0;
                return returnValue;
            }
        }
    }
    else if(g_state == 20)
    {
        if(!g_waitResponse)
        {
            //print("9602: AT+SBDWB1");
            sendCommand("AT+SBDWB=24\r", 12);
            return 1;
        }
        else if(g_responseSize)
        {
            if(parseResponse("READY\r\n", 7))
            {
                //print("9602: SBDWB1 ok");
                sendTelemetry();
                g_state = 21;

                g_responseSize = 0;
                return 1;
            }
        }
    }
    else if(g_state == 21)
    {
        if(g_responseSize)
        {
            if(parseResponse("0\r\n\r\nOK\r\n", 9))
            {
                //print("9602: SBDWB2 ok");
                g_state = 0;

                g_waitResponse = false;
                g_responseSize = 0;
                return SBDI_TIMEOUT;
            }
        }
    }
    else if(g_state == 30)
    {
        if(!g_waitResponse)
        {
            //print("9602: SBDD0");
            sendCommand("AT+SBDD0\r", 9);
            return 1;
        }
        else if(g_responseSize)
        {
            if(parseResponse("0\r\n\r\nOK\r\n", 9))
            {
                //print("9602: SBBD0 ok");
                g_state = 0;
                g_needClearBuffer = false;

                g_waitResponse = false;
                g_responseSize = 0;
                return 1;
            }
        }
    }
    else if(g_state == 100)
    {
        if(!g_waitResponse)
        {
            //print("9602: AT+GSN");
            sendCommand("AT+GSN\r", 7);
            return 1;
        }
        else if(g_responseSize)
        {
            if(parseResponse("OK\r\n", 4))
            {
                new serial{20};
                if(parseGSN(serial))
                {
                    print("9602: link ok");
                    print(serial);
                    g_state = 0;

                    g_waitResponse = false;
                    g_responseSize = 0;
                    return 1;
                }
            }
        }
    }
    else
        print("UNKNOWN STATE");

    if(time() - g_timePoint > WAIT_RESPONSE_TIMEOUT)
    {
        print("9602: no response");

        g_state = 0;
        g_waitResponse = false;
        g_responseSize = 0;
        g_timePoint = time();

    }

    return 1;
}
*/
