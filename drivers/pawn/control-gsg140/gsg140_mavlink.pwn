// FIRMWARE 2.72.3 REQUIRED
// Hardware/IMU Sensor Settings/Heading correction factor = 200
// External IMU/ROLL and PITCH angles                     = Compensate for linear accelerations only
// External IMU/YAW (heading) angle                       = Use as heading reference
// External IMY/Yaw target angle is absolute              = Yes
const PORT_ID = 30;
const LOOKHERE_PORT_ID = 123;
const LOOKHERE_DATA_SIZE = 8;

const MAVLINK_MAGIC = 0xFE;
const MAVLINK_HEADER_SIZE = 6;
const MAVLINK_CRC_SIZE = 2;

const IDX_MAGIC = 0;
const IDX_LEN = 1;
const IDX_SEQ = 2;
const IDX_SYSID = 3;
const IDX_COMPID = 4;
const IDX_MSGID = 5;

const HEARTBEAT_ID = 0;
const HEARTBEAT_MSG_SIZE = 9;
const HEARTBEAT_EXTRA_CRC = 50;
const ATTITUDE_ID = 30;
const ATTITUDE_MSG_SIZE = 28;
const ATTITUDE_EXTRA_CRC = 39;
const RDS_ID = 66;
const RDS_MSG_SIZE = 6;
const RDS_EXTRA_CRC = 148;
const GPI_ID = 33;
const GPI_MSG_SIZE = 28;
const GPI_EXTRA_CRC = 104;
const COMMAND_LONG_ID = 76;
const COMMAND_LONG_MSG_SIZE = 33;
const COMMAND_LONG_EXTRA_CRC = 152;

const Float:INPUT_TYPE_POS = 260.0;     //772 for precise agressive control
const Float:INPUT_TYPE_SPEED = 1.0;

const HEARTBEAT_INTERVAL = 1000;
new g_tpLastHeartbeat = 0;
const ATTITUDE_INTERVAL = 10;
new g_tpLastAttitude = 0;
const GPI_INTERVAL = 10;
new g_tpLastGpi = 0;
const COMMAND_LONG_INTERVAL = 20;
new g_tpLastCommandLong = 0;

const FEEDBACK_TIMEOUT = 5000;
new g_tpLastFeedbackReceived = 0;

const POS_YAW_VAR = f_radar_dy;         //gimbal yaw in global coords
const POS_PITCH_VAR = f_radar_dz;       //gimbal pitch in global coords
const POS_ROLL_VAR = f_radar_dx;        //gimbal roll in global coords

const FIXED_POS_YAW = 0;                //yaw pos in cam_mode=5
const FIXED_POS_PITCH = -45;            //pitch pos in cam_mode=5

new g_sequenceNumber = 0;
new g_lastCamMode = -1;

new Float:g_yawOffset = 0.0;            //offset to yaw from gimbal
new Float:g_pitchOffset = 0.0;          //offset to pitch from gimbal
new Float:g_rollOffset = 0.0;           //offset to roll from gimbal
new Float:g_yawPosReverse = -1.0;       //multiplier for yaw from gimbal
new Float:g_pitchPosReverse = 1.0;      //multiplier for pitch from gimbal
new Float:g_rollReverse = -1.0;         //multiplier for roll from gimbal
new Float:g_yawCmdReverse = -1.0;       //multiplier for yaw commands
new Float:g_pitchCmdReverse = -1.0;     //multiplier for pitch commands
new Float:g_camctr_yaw = 0.0;           //look here data from nav (yaw)
new Float:g_camctr_pitch = 0.0;         //look here data from nav (pitch)

Float:bound(Float:a)
{
    return a - floatround(a / 360.0 + 0.5, floatround_floor) * 360.0;
}

Float:bound360(Float:a)
{
    while (a < 0.0)
        a += 360.0;
    while (a >= 360.0)
        a -= 360.0;
    return a;
}

main()
{
    serial_listen(PORT_ID, "@OnSerial");
    serial_listen(LOOKHERE_PORT_ID, "@OnLookHereCommands");
    sendRds();
    g_tpLastFeedbackReceived = time();
}

crcAccumulate(data, crcAccum)
{
    /*Accumulate one byte of data into the CRC*/
    new tmp{1};

    tmp{0} = data ^ (crcAccum &0xff);
    tmp{0} ^= (tmp{0}<<4);
    crcAccum = (crcAccum>>8) ^ (tmp{0}<<8) ^ (tmp{0} <<3) ^ (tmp{0}>>4);
    return crcAccum;
}

crcInit()
{
    return 0xffff;
}

crcCalculate(pBuffer{}, length, offset)
{
    new crcTmp = crcInit();
    new i = offset;
    while(length--) {
        crcTmp = crcAccumulate(pBuffer{i}, crcTmp);
        i++;
    }
    return crcTmp;
}

crc_get_extra(msgid)
{
    if(msgid == HEARTBEAT_ID)
        return HEARTBEAT_EXTRA_CRC;
    else if(msgid == RDS_ID)
        return RDS_EXTRA_CRC;
    else if(msgid == ATTITUDE_ID)
        return ATTITUDE_EXTRA_CRC;
    else
        return 0;
}

packFloat(data{}, index, Float:v)
{
    for(new i = 0; i < 4; i++)
        data{index + i} = (v >> (8 * i)) & 0xFF;
}

packInt(data{}, index, v)
{
    for(new i = 0; i < 4; i++)
        data{index + i} = (v >> (8 * i)) & 0xFF;
}

packDegE7(data{}, index, Float:v)
{
    new degE7 = floatint(v * 10000000);
    packInt(data, index, degE7);
}

packInt2(data{}, index, v)
{
    data{index} = (v >> (0 * 8)) & 0xFF;
    data{index + 1} = (v >> (1 * 8)) & 0xFF;
}

Float:unpackFloat(data{}, index)
{
    new temp{4};
    new Float:value[1];
    temp{0} = data{index + 3};
    temp{1} = data{index + 2};
    temp{2} = data{index + 1};
    temp{3} = data{index};
    memcpy(value, temp, 0, 4);
    return value[0];
}

fillHeader(data{}, msgId, payloadSize)
{
    data{IDX_MAGIC} = MAVLINK_MAGIC;
    data{IDX_LEN} = payloadSize;
    data{IDX_SEQ} = g_sequenceNumber;
    data{IDX_SYSID} = 1;
    data{IDX_COMPID} = 1;
    data{IDX_MSGID} = msgId;
    g_sequenceNumber++;
}

sendHearbeat()
{
    const dataSize = MAVLINK_HEADER_SIZE + HEARTBEAT_MSG_SIZE + MAVLINK_CRC_SIZE;
    new data{dataSize};
    fillHeader(data, HEARTBEAT_ID, HEARTBEAT_MSG_SIZE);
    data{MAVLINK_HEADER_SIZE} = 0;          //type of the system
    data{MAVLINK_HEADER_SIZE + 1} = 0;      //autopilot type
    data{MAVLINK_HEADER_SIZE + 2} = 0;      //base mode
    data{MAVLINK_HEADER_SIZE + 3} = 0;      //custom mode
    data{MAVLINK_HEADER_SIZE + 4} = 0;
    data{MAVLINK_HEADER_SIZE + 5} = 0;
    data{MAVLINK_HEADER_SIZE + 6} = 0;
    data{MAVLINK_HEADER_SIZE + 7} = 4;      //system status, active
    data{MAVLINK_HEADER_SIZE + 8} = 1;      //mavlink version, v1
    new crc = crcCalculate(data, MAVLINK_HEADER_SIZE + HEARTBEAT_MSG_SIZE - 1, 1);
    crc = crcAccumulate(HEARTBEAT_EXTRA_CRC, crc);
    data{MAVLINK_HEADER_SIZE + 9} = crc & 0xFF;
    data{MAVLINK_HEADER_SIZE + 10} = crc >> 8;
    new bool:result = serial_write(PORT_ID, data, dataSize, serialmode:NODE);
    if(!result)
        print("Gimbal: sendHearbeat error");
}

sendAttitude()
{
    const dataSize = MAVLINK_HEADER_SIZE + ATTITUDE_MSG_SIZE + MAVLINK_CRC_SIZE;
    new data{dataSize};
    fillHeader(data, ATTITUDE_ID, ATTITUDE_MSG_SIZE);
    packInt(data, MAVLINK_HEADER_SIZE, time());                         //time since system boot
    packFloat(data, MAVLINK_HEADER_SIZE + 4, get_var(f_roll) * D2R);    //roll
    packFloat(data, MAVLINK_HEADER_SIZE + 8, get_var(f_pitch) * D2R);   //pitch
    packFloat(data, MAVLINK_HEADER_SIZE + 12, get_var(f_yaw) * D2R);    //yaw
    packFloat(data, MAVLINK_HEADER_SIZE + 16, 0.0);                     //rollspeed
    packFloat(data, MAVLINK_HEADER_SIZE + 20, 0.0);                     //pitchspeed
    packFloat(data, MAVLINK_HEADER_SIZE + 24, 0.0);                     //yawspeed
    new crc = crcCalculate(data, MAVLINK_HEADER_SIZE + ATTITUDE_MSG_SIZE - 1, 1);
    crc = crcAccumulate(ATTITUDE_EXTRA_CRC, crc);
    data{MAVLINK_HEADER_SIZE + 28} = crc & 0xFF;
    data{MAVLINK_HEADER_SIZE + 29} = crc >> 8;
    new bool:result = serial_write(PORT_ID, data, dataSize, serialmode:NODE);
    if(!result)
        print("Gimbal: sendAttitude error");
}

sendGpi()
{
    const dataSize = MAVLINK_HEADER_SIZE + GPI_MSG_SIZE + MAVLINK_CRC_SIZE;
    new data{dataSize};
    fillHeader(data, GPI_ID, GPI_MSG_SIZE);
    packInt(data, MAVLINK_HEADER_SIZE, time()); //time since system boot
    packDegE7(data, MAVLINK_HEADER_SIZE + 4, get_var(f_gps_lat));
    packDegE7(data, MAVLINK_HEADER_SIZE + 8, get_var(f_gps_lon));
    packInt(data, MAVLINK_HEADER_SIZE + 12, floatint(get_var(f_gps_hmsl) * 1000));
    packInt(data, MAVLINK_HEADER_SIZE + 16, floatint(get_var(f_altitude) * 1000));
    packInt2(data, MAVLINK_HEADER_SIZE + 20, floatint(get_var(f_gps_Vnorth) * 100));
    packInt2(data, MAVLINK_HEADER_SIZE + 22, floatint(get_var(f_gps_Veast) * 100));
    packInt2(data, MAVLINK_HEADER_SIZE + 24, floatint(get_var(f_gps_Vdown) * 100));
    packInt2(data, MAVLINK_HEADER_SIZE + 26, floatint(bound360(get_var(f_yaw)) * 100));
    new crc = crcCalculate(data, MAVLINK_HEADER_SIZE + GPI_MSG_SIZE - 1, 1);
    crc = crcAccumulate(GPI_EXTRA_CRC, crc);
    data{MAVLINK_HEADER_SIZE + 28} = crc & 0xFF;
    data{MAVLINK_HEADER_SIZE + 29} = crc >> 8;
    new bool:result = serial_write(PORT_ID, data, dataSize, serialmode:NODE);
    if(!result)
        print("Gimbal: sendGpi error");
}

sendCommandLongControl()
{
    new Float:yawCmd = 0.0
    new Float:pitchCmd = 0.0
    new Float:rollCmd = 0.0
    new camMode = get_var(f_cam_mode);
    if(camMode == 0) {
        //camoff
        //maintain stow position from Shiva/Regulator/Gimbal
        yawCmd = get_var(f_camctr_yaw) * 180.0 * g_yawCmdReverse;
        pitchCmd = get_var(f_camctr_pitch) * 180.0 * g_pitchCmdReverse;
    } else if(camMode == 2) {
        //position
        yawCmd = bound(get_var(f_yaw) + get_var(f_camcmd_yaw)) * g_yawCmdReverse;
        pitchCmd = (get_var(f_pitch) + get_var(f_camcmd_pitch)) * g_pitchCmdReverse;
    } else if(camMode == 3) {
        //speed
        yawCmd = get_var(f_camcmd_yaw) * g_yawCmdReverse;
        pitchCmd = get_var(f_camcmd_pitch) * g_pitchCmdReverse;
    } else if(camMode == 4) {
        //target
        yawCmd = bound(g_camctr_yaw * 180.0) * g_yawCmdReverse;
        pitchCmd = g_camctr_pitch * 180.0 * g_pitchCmdReverse;
    } else if(camMode == 5) {
        //fixed
        yawCmd = FIXED_POS_YAW * g_yawCmdReverse;
        pitchCmd = FIXED_POS_PITCH * g_pitchCmdReverse;
    } else if(camMode == 6) {
        yawCmd = - bound(get_var(f_cmd_course) - get_var(f_yaw));
        pitchCmd = FIXED_POS_PITCH * g_pitchCmdReverse;
    }

    const dataSize = MAVLINK_HEADER_SIZE + COMMAND_LONG_MSG_SIZE + MAVLINK_CRC_SIZE;
    new data{dataSize};
    fillHeader(data, COMMAND_LONG_ID, COMMAND_LONG_MSG_SIZE);
    packFloat(data, MAVLINK_HEADER_SIZE,     pitchCmd);                 //pitch
    packFloat(data, MAVLINK_HEADER_SIZE + 4, rollCmd);                  //roll
    packFloat(data, MAVLINK_HEADER_SIZE + 8, yawCmd);                   //yaw
    packFloat(data, MAVLINK_HEADER_SIZE + 12, 0.0);                     //altitude
    packDegE7(data, MAVLINK_HEADER_SIZE + 16, 0.0);                     //lat
    packDegE7(data, MAVLINK_HEADER_SIZE + 20, 0.0);                     //lon
    packFloat(data, MAVLINK_HEADER_SIZE + 24, 2.0);                     //mode
    packInt2(data, MAVLINK_HEADER_SIZE + 28, 205);                      //command id
    data{MAVLINK_HEADER_SIZE + 30} = 1;                                 //target system id
    data{MAVLINK_HEADER_SIZE + 31} = 154;                               //target component id
    data{MAVLINK_HEADER_SIZE + 32} = 0;                                 //confirmation
    new crc = crcCalculate(data, MAVLINK_HEADER_SIZE + COMMAND_LONG_MSG_SIZE - 1, 1);
    crc = crcAccumulate(COMMAND_LONG_EXTRA_CRC, crc);
    data{MAVLINK_HEADER_SIZE + 33} = crc & 0xFF;
    data{MAVLINK_HEADER_SIZE + 34} = crc >> 8;
    new bool:result = serial_write(PORT_ID, data, dataSize, serialmode:NODE);
    if(!result)
        print("Gimbal: sendCommandLongError error");
}

sendCommandLongConfigure(mode)
{
    const dataSize = MAVLINK_HEADER_SIZE + COMMAND_LONG_MSG_SIZE + MAVLINK_CRC_SIZE;
    new data{dataSize};
    fillHeader(data, COMMAND_LONG_ID, COMMAND_LONG_MSG_SIZE);
    packFloat(data, MAVLINK_HEADER_SIZE, 2.0);
    packFloat(data, MAVLINK_HEADER_SIZE + 4, 1.0);
    packFloat(data, MAVLINK_HEADER_SIZE + 8, 1.0);
    packFloat(data, MAVLINK_HEADER_SIZE + 12, 1.0);
    packFloat(data, MAVLINK_HEADER_SIZE + 16, INPUT_TYPE_SPEED);     //roll
    packFloat(data, MAVLINK_HEADER_SIZE + 20, mode);                 //pitch
    packFloat(data, MAVLINK_HEADER_SIZE + 24, mode);                 //yaw
    packInt2(data, MAVLINK_HEADER_SIZE + 28, 204);                   //command id
    data{MAVLINK_HEADER_SIZE + 30} = 1;                              //target system id
    data{MAVLINK_HEADER_SIZE + 31} = 154;                            //target component id
    data{MAVLINK_HEADER_SIZE + 32} = 0;                              //confirmation
    new crc = crcCalculate(data, MAVLINK_HEADER_SIZE + COMMAND_LONG_MSG_SIZE - 1, 1);
    crc = crcAccumulate(COMMAND_LONG_EXTRA_CRC, crc);
    data{MAVLINK_HEADER_SIZE + 33} = crc & 0xFF;
    data{MAVLINK_HEADER_SIZE + 34} = crc >> 8;
    new bool:result = serial_write(PORT_ID, data, dataSize, serialmode:NODE);
    if(!result)
        print("Gimbal: sendCommandLongConfigure error");
}

sendRds()
{
    const dataSize = MAVLINK_HEADER_SIZE + RDS_MSG_SIZE + MAVLINK_CRC_SIZE;
    new data{dataSize};
    fillHeader(data, RDS_ID, RDS_MSG_SIZE);
    packInt2(data, MAVLINK_HEADER_SIZE, 10);    //rate
    data{MAVLINK_HEADER_SIZE + 2} = 1;          //system id
    data{MAVLINK_HEADER_SIZE + 3} = 154;        //comp id
    data{MAVLINK_HEADER_SIZE + 4} = 4;          //stream id
    data{MAVLINK_HEADER_SIZE + 5} = 1;          //start/stop

    new crc = crcCalculate(data, MAVLINK_HEADER_SIZE + RDS_MSG_SIZE - 1, 1);
    crc = crcAccumulate(RDS_EXTRA_CRC, crc);
    data{MAVLINK_HEADER_SIZE + 6} = crc & 0xFF;
    data{MAVLINK_HEADER_SIZE + 7} = crc >> 8;
    new bool:result = serial_write(PORT_ID, data, dataSize, serialmode:NODE);
    if(!result)
        print("Gimbal: sendRds error");
}

@OnTask()
{
    new now = time();

    if(now - g_tpLastFeedbackReceived > FEEDBACK_TIMEOUT) {
        sendRds();
        g_tpLastFeedbackReceived = now;
        print("Gimbal: no telemetry detected, request...")
    }

    //default cam mode override
    new camMode = get_var(f_cam_mode);
    if(g_lastCamMode != camMode) {
        g_lastCamMode = camMode;

        //position
        if(camMode == 0) {          //camoff
            sendCommandLongConfigure(INPUT_TYPE_POS);
            sendCommandLongControl();
            print("Gimbal: mode switched to camoff");
        } else if(camMode == 2) {   //position
            set_var(f_camcmd_yaw, 0.0, true);
            set_var(f_camcmd_pitch, 0.0, true);
            sendCommandLongConfigure(INPUT_TYPE_POS);
            print("Gimbal: mode switched to position");
        } else if(camMode == 3) {   //speed
            set_var(f_camcmd_yaw, 0.0, true);
            set_var(f_camcmd_pitch, 0.0, true);
            sendCommandLongConfigure(INPUT_TYPE_SPEED);
            print("Gimbal: mode switched to speed");
        } else if(camMode == 4) {   //target
            sendCommandLongConfigure(INPUT_TYPE_POS);
            print("Gimbal: mode switched to look here");
        } else if(camMode == 5) {   //fixed
            sendCommandLongConfigure(INPUT_TYPE_POS);
            print("Gimbal: mode switched to fixed");
        } else if(camMode == 6) {
            sendCommandLongConfigure(INPUT_TYPE_POS);
            print("Gimbal: mode switched to course");
        } else {
            printf("Gimbal: unknown cam mode %d\n", camMode);
        }

        sendAttitude();
        g_tpLastAttitude = now;
    }

    if(now - g_tpLastHeartbeat > HEARTBEAT_INTERVAL) {
        sendHearbeat();
        g_tpLastHeartbeat = now;
    }
    if(now - g_tpLastAttitude > ATTITUDE_INTERVAL) {
        sendAttitude();
        g_tpLastAttitude = now;
    }
    if(now - g_tpLastGpi > GPI_INTERVAL) {
        sendGpi();
        g_tpLastGpi = now;
    }
    if(now - g_tpLastCommandLong > COMMAND_LONG_INTERVAL) {
        if(camMode >= 0 && camMode <= 6) {
            sendCommandLongControl();
        }
        g_tpLastCommandLong = now;
    }

    return 10;
}

forward @OnSerial(cnt)
@OnSerial(cnt)
{
    new data{200};
    for(new i = 0; i < cnt; i++) {
        data{i} = serial_byte(i);
    }
    new msgid = data{IDX_MSGID};
    new crc1 = crcCalculate(data, cnt - 3, 1);
    crc1 = crcAccumulate(crc_get_extra(msgid), crc1);
    new crc2 = (data{cnt - 1} << 8) | (data{cnt - 2});
    if(crc1 == crc2) {
        if(msgid == HEARTBEAT_ID) {
            //unused
        } else if(msgid == RDS_ID) {
            //unused
        } else if(msgid == ATTITUDE_ID) {
            new Float:rollPos = unpackFloat(data, MAVLINK_HEADER_SIZE + 4) * R2D;
            new Float:pitchPos = unpackFloat(data, MAVLINK_HEADER_SIZE + 8) * R2D;
            new Float:yawPos = unpackFloat(data, MAVLINK_HEADER_SIZE + 12) * R2D;

            yawPos = (yawPos + g_yawOffset) * g_yawPosReverse;
            pitchPos = (pitchPos + g_pitchOffset) * g_pitchPosReverse;
            rollPos = (rollPos + g_rollOffset) * g_rollReverse;

            set_var(POS_YAW_VAR, yawPos, true);
            set_var(POS_PITCH_VAR, pitchPos, true);
            set_var(POS_ROLL_VAR, rollPos, true);

            g_tpLastFeedbackReceived = time();
        } else {
            printf("Gimbal: unknown message: %d\n", msgid);
        }
    }
}

forward @OnLookHereCommands(cnt)
@OnLookHereCommands(cnt)
{
    if(cnt != LOOKHERE_DATA_SIZE)
        return;

    new buffer{LOOKHERE_DATA_SIZE};
    for(new i = 0; i < cnt; i++)
        buffer{i} = serial_byte(i);

    g_camctr_yaw = unpackFloat(buffer, 0);
    g_camctr_pitch = unpackFloat(buffer, 4);
}
