#include <apx.h>

#define NODE_LEFT TRUE //ifc-l
//#define NODE_RIGHT TRUE //ifc-r

#if defined NODE_LEFT
constexpr const char *txt_dev = "L";           //L
constexpr const port_id_t PORT_ID_CAN_AUX{20}; //L=20 //CAN   1Mb
using m_pump_rpm_raw = Mandala<mandala::sns::env::scr::s8>;
using m_pump_rpm = Mandala<mandala::est::env::usrw::w1>; //pump rpm left
#endif

#if defined NODE_RIGHT
constexpr const char *txt_dev = "R";           //R
constexpr const port_id_t PORT_ID_CAN_AUX{70}; //R=70 //CAN   1Mb
using m_pump_rpm_raw = Mandala<mandala::sns::env::scr::s9>;
using m_pump_rpm = Mandala<mandala::est::env::usrw::w2>; //pump rpm right
#endif

using m_pwr_eng = Mandala<mandala::ctr::env::pwr::eng>;
using m_pwr_satcom = Mandala<mandala::ctr::env::pwr::satcom>;

constexpr const uint8_t TASK_MAIN_MS{50}; //20Hz

//--------------------------pu-----------------------------
constexpr const uint16_t PU_ID{128};
constexpr const uint16_t PU_SHIFT{20};
constexpr const uint16_t PU_CMD_HEATER{PU_ID + 10};
constexpr const uint16_t PU_CMD_ON{PU_ID + 12};
constexpr const uint16_t PU_CMD_RB{PU_ID + 13};

//--------------------------sp-----------------------------
constexpr const uint16_t SP_ID{512};
constexpr const uint16_t SP_SHIFT{20};
constexpr const uint16_t SP_CMD_POWER{SP_ID + 10};
constexpr const uint16_t SP_CMD_HOLD{SP_ID + 13};
constexpr const uint16_t SP_CMD_MOVE{SP_ID + 14};
constexpr const uint16_t SP_CMD_HEAT{SP_ID + 15};
constexpr const uint16_t SP_CMD_STEP{SP_ID + 16};

//--------------------------pump---------------------------
uint32_t timer_changed_rpm{};
constexpr const uint16_t PUMP_RPM_TIMEOUT{2000};

//task("pu");    //GCS with terminal command `vmexec("pu", 0)` or `vmexec("pu", 1)`
//EXPORT void pu(int32_t val){}

/*
sp_x - command for SP
10 - off;
11 - on;
12 - hold v_in;
13 - hold dflt_Vmpp;
14 - down;
15 - up;
16 - heat on;
17 - heat off;
*/

//IRIDIUM
static const port_id_t PORT_ID = 19;

int main()
{
    m_pump_rpm_raw("on_pump_rpm_raw"); // subscribe `on changed` event
    m_pwr_eng("on_pwr_eng");           // subscribe `on changed` event
    m_pwr_satcom();

    task("pu_hold"); // 0/1
    task("pu_rb");   // 0..100

#if defined NODE_LEFT
    task("sp_1");
    task("sp_2");

    task("pu_h1"); // 0..30
    task("pu_h2"); // 0..30
#endif

#if defined NODE_RIGHT
    task("sp_3");
    task("sp_4");

    task("pu_h3"); // 0..30
    task("pu_h4"); // 0..30
#endif

    task("on_main", TASK_MAIN_MS); //20 Hz

    printf("IFC:%s Script ready...\n", txt_dev);

    //IRIDIUM
    sleep(1000);
    receive(PORT_ID, "serialHandler");
    task("onTask", 10);

    return 0;
}

EXPORT void on_main()
{
    if (time_ms() - timer_changed_rpm > PUMP_RPM_TIMEOUT) {
        m_pump_rpm::publish(0u);
    }
}

EXPORT void on_pump_rpm_raw()
{
    timer_changed_rpm = time_ms();
    m_pump_rpm::publish(m_pump_rpm_raw::value());
}

template<typename T>
T limit(T value, T min, T max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

void sendCmdToCan(const uint32_t &can_id, const uint8_t *data, const uint8_t &size)
{
    constexpr const uint8_t PACK_CAN_SIZE{12};

    uint8_t msg[PACK_CAN_SIZE] = {};

    msg[0] = (uint8_t) can_id;         //ID_0_7
    msg[1] = (uint8_t) (can_id >> 8);  //ID_8_15
    msg[2] = (uint8_t) (can_id >> 16); //ID_16_23
    msg[3] = (uint8_t) (can_id >> 24); //ID_24_31

    if (can_id > 0x7FF)
        msg[3] = msg[3] | 0x80;

    for (uint8_t i = 0; i < size; i++) {
        msg[4 + i] = data[i];
    }

    send(PORT_ID_CAN_AUX, msg, 4 + size, false);
    send(PORT_ID_CAN_AUX + 1, msg, 4 + size, true);
}

void pu_cmd_power_on(uint8_t val)
{
    uint8_t msg[1] = {val};

    sendCmdToCan(PU_CMD_ON, msg, 1);
    sendCmdToCan(PU_CMD_ON + PU_SHIFT, msg, 1);
}

void pu_cmd_rb(uint8_t val)
{
    uint8_t msg[1] = {val};

    sendCmdToCan(PU_CMD_RB, msg, 1);
    sendCmdToCan(PU_CMD_RB + PU_SHIFT, msg, 1);
}

void pu_cmd_heater(const uint8_t &dev_id, const uint8_t &value)
{
    const uint16_t cmd = value * 100;

    uint8_t msg[2] = {};
    msg[0] = (uint8_t) cmd;
    msg[1] = cmd >> 8;

    sendCmdToCan(PU_CMD_HEATER + dev_id * PU_SHIFT, msg, 2);
}

void sp_power(const uint8_t &dev_id, const uint8_t &value)
{
    uint8_t msg[1] = {value};
    sendCmdToCan(SP_CMD_POWER + dev_id * SP_SHIFT, msg, 1);
}

void sp_hold(const uint8_t &dev_id, const uint8_t &value)
{
    uint8_t msg[1] = {value};
    sendCmdToCan(SP_CMD_HOLD + dev_id * SP_SHIFT, msg, 1);
}

void sp_move(const uint8_t &dev_id, const uint8_t &value)
{
    uint8_t msg[1] = {value};
    sendCmdToCan(SP_CMD_MOVE + dev_id * SP_SHIFT, msg, 1);
}

void sp_step(const uint8_t &dev_id, const uint8_t &value)
{
    //variable step from -128 to 127 (-1.28V to 1.27V) divided by 100
    uint8_t msg[1] = {value};
    sendCmdToCan(SP_CMD_STEP + dev_id * SP_SHIFT, msg, 1);
}

void sp_heat(const uint8_t &dev_id, const uint8_t &value)
{
    uint8_t msg[1] = {value};
    sendCmdToCan(SP_CMD_HEAT + dev_id * SP_SHIFT, msg, 1);
}

EXPORT void on_pwr_eng()
{
    if (!(bool) m_pwr_eng::value()) {
        sp_power(0, 0);
        sp_power(1, 0);
    }
}

EXPORT void pu_hold(int32_t val)
{
    printf("IFC-%s, pu...\n", txt_dev);
    pu_cmd_power_on((uint8_t) val);
}

EXPORT void pu_rb(int32_t val)
{
    printf("IFC-%s, pu_rb...\n", txt_dev);
    val = limit(val, 0, 100);
    pu_cmd_rb((uint8_t) val);
}

EXPORT void sp(const uint8_t sp_id, int32_t val)
{
    if (val == 10) {
        sp_power(sp_id, 0); //power off
    } else if (val == 11) {
        sp_power(sp_id, 1); //power on
    } else if (val == 12) {
        sp_hold(sp_id, 0); //hold V_in
    } else if (val == 13) {
        sp_hold(sp_id, 1); //hold Vmpp
    } else if (val == 14) {
        sp_move(sp_id, 0); //move down
    } else if (val == 15) {
        sp_move(sp_id, 1); //move up
    } else if (val == 16) {
        sp_heat(sp_id, 1); //heat on
    } else if (val == 17) {
        sp_heat(sp_id, 0); //heat off
    }
}

#if defined NODE_LEFT
EXPORT void sp_1(int32_t val)
{
    printf("IFC-%s, sp_1...\n", txt_dev);
    printf("CMD-%u\n", val);
    sp(0, val);
}

EXPORT void sp_2(int32_t val)
{
    printf("IFC-%s, sp_2...\n", txt_dev);
    printf("CMD-%u\n", val);
    sp(1, val);
}

EXPORT void pu_h1(int32_t val)
{
    printf("IFC-%s, pu_h1...\n", txt_dev);
    pu_cmd_heater(0, (uint8_t) val);
}

EXPORT void pu_h2(int32_t val)
{
    printf("IFC-%s, pu_h2...\n", txt_dev);
    pu_cmd_heater(1, (uint8_t) val);
}
#endif

#if defined NODE_RIGHT
EXPORT void sp_3(int32_t val)
{
    printf("IFC-%s, sp_3...\n", txt_dev);
    printf("CMD-%u\n", val);
    sp(0, val);
}

EXPORT void sp_4(int32_t val)
{
    printf("IFC-%s, sp_4...\n", txt_dev);
    printf("CMD-%u\n", val);
    sp(1, val);
}

EXPORT void pu_h3(int32_t val)
{
    printf("IFC-%s, pu_h3...\n", txt_dev);
    pu_cmd_heater(0, (uint8_t) val);
}

EXPORT void pu_h4(int32_t val)
{
    printf("IFC-%s, pu_h4...\n", txt_dev);
    pu_cmd_heater(1, (uint8_t) val);
}
#endif

//IRIDIUM
static const uint32_t SBDI_TIMEOUT = 1000 * 30;
static const uint32_t WAIT_RESPONSE_TIMEOUT = 1000 * 60;

static const bool PRINT_DEBUG_MESSAGES = false;

static const size_t MAX_RESPONSE_SIZE = 100;
enum MessageType {
    mtUnknown = -1,
    mtVarRequest = 0,
    mtVarResponse = 1,
    mtVarCommand = 2
};

struct VarResponseMessage {
    mandala_uid_t uids[40];
    float values[40];
    size_t count = 0;
};

struct ModemResponse {
    char data[MAX_RESPONSE_SIZE];
    size_t size = 0;
};

struct LockoutState {
    uint32_t valid = 1;   //0 - timeout valid, 1 - timeout invalid
    uint32_t tp = 0;      //tp when lockout state was updated
    uint32_t timeout = 0; //in ms
};

struct IdleState {
    uint32_t tp = 0;       //tp when idle started
    uint32_t interval = 0; //in ms
};

static ModemResponse g_response;
static VarResponseMessage g_varResponseMessage;
static LockoutState g_lockoutState;
static IdleState g_idleState;

static bool g_waitResponse = false;
static bool g_needClearBuffer = false;
static uint32_t g_timePoint = 0;

int g_state = 100;

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
 *
 * 202 - start AT+SBDTC     : transfer MO to MT (debug only)
 * 220 - 1st stage AT+SBDWB : prepare to send debug data (debug only)
 * 221 - 2st stage AT+SBDWB : send debug data
 *
 */

bool parseResponse(const char *okMark)
{
    size_t okCounter = 0;
    size_t okMarkSize = strlen(okMark);
    for(size_t i = 0; i < g_response.size; i++) {
        if(g_response.data[i] == okMark[okCounter]) {
            okCounter++;
        } else {
            okCounter = 0;
        }
        if(okCounter == okMarkSize) {
            return true;
        }
    }
    return false;
}

int32_t parseCSQ()
{
    auto startPos = strstr(g_response.data, ":");
    if(startPos) {
        return (int32_t)strtol(startPos + 1, nullptr, 10);
    } else {
        return -1;
    }
}

bool parseSBDLOE()
{
    auto startPos = strstr(g_response.data, ":");
    if(startPos) {
        startPos += 1;
        g_lockoutState.valid = (uint32_t)strtol(startPos, nullptr, 10);
        startPos = strstr(startPos, ",");
        if(startPos) {
            startPos += 1;
            g_lockoutState.timeout = (uint32_t)strtol(startPos, nullptr, 10);
            g_lockoutState.tp = time_ms();
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}

bool parseSBDI(uint32_t &sendState, uint32_t &receiveState)
{
    auto startPos = strstr(g_response.data, ":");
    if(startPos) {
        sendState = (uint32_t)strtol(startPos + 1, nullptr, 10);
        receiveState = (uint32_t)strtol(startPos + 8, nullptr, 10);
        return true;
    } else {
        return false;
    }
}

MessageType parseSBDRB()
{
    auto size = g_response.data[9] * 256 + g_response.data[10];
    auto messageType = g_response.data[11];
    if(messageType == mtVarRequest && size >= 3) {
        size_t count = 0;
        for(int i = 12; i < 12 + size - 1; i += 2) {
            mandala_uid_t uid;
            memcpy(&uid, g_response.data + i, 2);
            auto handle = mbind(uid, nullptr);
            g_varResponseMessage.uids[count] = uid;
            g_varResponseMessage.values[count] = mget(handle);
            count++;
        }
        g_varResponseMessage.count = count;
        return mtVarRequest;
    } else if(messageType == mtVarCommand && size >= 6) {
        for(int i = 12; i < 12 + size - 1; i += 6) {
            mandala_uid_t uid;
            memcpy(&uid, g_response.data + i, 2);
            float value = 0;
            memcpy(&value, g_response.data + i + 2, 4);
            mset_f(uid, value);
        }
        return mtVarCommand;
    } else {
        return mtUnknown;
    }
}

bool parseGSN(char serial[20])
{
    auto startPos = strstr(g_response.data, "\n");
    if(startPos) {
        auto endPos = strstr(startPos + 1, "\r");
        *endPos = '\0';
        strcpy(serial, startPos + 1);
        return true;
    } else {
        return false;
    }
}

void sendCommand(const char *cmd)
{
    size_t size = strlen(cmd);
    send(PORT_ID, cmd, size);
    g_waitResponse = true;
    g_timePoint = time_ms();
}

void sendTelemetry()
{
    size_t dataSize = 1 + g_varResponseMessage.count * 6;
    uint8_t data[dataSize + 2]; //+2 for crc
    data[0] = mtVarResponse;
    for(size_t i = 0; i < g_varResponseMessage.count; i++) {
        memcpy(data + 1 + i * 6, g_varResponseMessage.uids + i, 2);
        memcpy(data + 1 + i * 6 + 2, g_varResponseMessage.values + i, 4);
    }
    uint16_t crc = 0;
    for(size_t i = 0; i < dataSize; i++) {
        crc += data[i];
    }
    data[dataSize] = (crc >> 8) & 0xFF;
    data[dataSize + 1] = crc & 0xFF;
    send(PORT_ID, data, dataSize + 2);
}

void sendDebugData()
{
    uint8_t data[11] = {0, 37, 4, 36, 4, 35, 4, 56, 4, 0, 0};
    uint16_t crc = 0;
    for(size_t i = 0; i < 9; i++) {
        crc += data[i];
    }
    data[9] = (crc >> 8) & 0xFF;
    data[10] = crc & 0xFF;
    send(PORT_ID, data, 11);
}

bool inLockout()
{
    auto futureTp = g_lockoutState.tp + g_lockoutState.timeout;
    if(g_lockoutState.valid == 0 && time_ms() < futureTp) {
        return true;
    } else {
        return false;
    }
}

bool inIdle()
{
    auto futureTp = g_idleState.tp + g_idleState.interval;
    return time_ms() < futureTp;
}

void makeIdle(uint32_t interval)
{
    g_idleState.tp = time_ms();
    g_idleState.interval = interval;
}

EXPORT void serialHandler(const uint8_t *data, size_t size)
{
    if(g_response.size + size > MAX_RESPONSE_SIZE) {
        print("iridium: buffer overflow");
        return;
    } else {
        memcpy(g_response.data + g_response.size, data, size);
    }
    g_response.size += size;
    g_response.data[g_response.size] = '\0';
}

EXPORT void onTask()
{

    if ((bool)m_pwr_satcom::value() == false) {
        return;
    }

    if(inLockout() || inIdle()) {
        return;
    }

    if(g_state == 0 && g_needClearBuffer) {
        g_state = 30;
    }

    if(g_state == 0) {
        if(!g_waitResponse) {
            if(PRINT_DEBUG_MESSAGES) {
                print("iridium: AT+CSQ");
            }
            sendCommand("AT+CSQ\r");
        } else if(g_response.size) {
            if(parseResponse("OK\r\n")) {
                auto signal = parseCSQ();
                if(PRINT_DEBUG_MESSAGES) {
                    printf("iridium: CSQ: %i\n", signal);
                }
                if(signal >= 2) {
                    g_state = 1;
                } else {
                    g_waitResponse = false;
                    g_response.size = 0;
                    makeIdle(10000);
                }
            }
        }
    } else if(g_state == 1) {
        if(!g_waitResponse) {
            if(PRINT_DEBUG_MESSAGES) {
                print("iridium: AT+SBDLOE");
            }
            sendCommand("AT+SBDLOE\r");
        } else if(g_response.size) {
            if(parseResponse("OK\r\n")) {
                if(parseSBDLOE()) {
                    if(PRINT_DEBUG_MESSAGES) {
                        printf("iridium: SBDLOE valid: %i\n", g_lockoutState.valid);
                        printf("iridium: SBDLOE timeout: %i\n", g_lockoutState.timeout);
                    }
                    if(inLockout()) { //no traffic management period
                        g_state = 0;
                    } else {
                        g_state = 2; //remaining time = 0, start session
                    }
                }

                g_waitResponse = false;
                g_response.size = 0;
            }
        }
    } else if(g_state == 2) {
        if(!g_waitResponse) {
            if(PRINT_DEBUG_MESSAGES) {
                print("iridium: AT+SBDI");
            }
            sendCommand("AT+SBDI\r");
        } else if(g_response.size) {
            if(parseResponse("OK\r\n")) {
                uint32_t sendState = 2;
                uint32_t receiveState = 2;
                if(parseSBDI(sendState, receiveState)) {
                    if(PRINT_DEBUG_MESSAGES) {
                        printf("iridium: SBDI: %i\n", sendState);
                        printf("iridium: SBDI: %i\n", receiveState);
                    }
                    if(sendState == 2 || receiveState == 2) { //session fail, try again all steps
                        g_state = 0;
                    } else if(receiveState == 1) { //have incoming message, need AT+SBDRB
                        g_state = 10;
                    } else { //all ok, no messages, next session after SBDI_TIMEOUT ms
                        g_state = 0;
                        makeIdle(SBDI_TIMEOUT);
                    }

                    if(sendState == 1) { //we send msg from MO, need clear MO later
                        g_needClearBuffer = true;
                    }
                } else {
                    g_state = 0;
                }

                g_waitResponse = false;
                g_response.size = 0;
            }
        }
    } else if(g_state == 10) {
        if(!g_waitResponse) {
            if(PRINT_DEBUG_MESSAGES) {
                print("iridium: AT+SBDRB");
            }
            sendCommand("AT+SBDRB\r");
        } else if(g_response.size) {
            if(parseResponse("OK\r\n")) {
                auto result = parseSBDRB();
                if(result == mtVarRequest) {
                    g_state = 20;
                    print("iridium: var request received");
                } else if(result == mtVarCommand) {
                    print("iridium: var command received");
                    g_state = 0;
                    makeIdle(SBDI_TIMEOUT);
                }

                g_waitResponse = false;
                g_response.size = 0;
            }
        }
    } else if(g_state == 20) {
        if(!g_waitResponse) {
            if(PRINT_DEBUG_MESSAGES) {
                print("iridium: AT+SBDWB1");
            }
            char cmd[12];
            snprintf(cmd, 12, "AT+SBDWB=%i\r", int32_t(1 + g_varResponseMessage.count * 6));
            sendCommand(cmd);
        } else if(g_response.size) {
            if(parseResponse("READY\r\n")) {
                if(PRINT_DEBUG_MESSAGES) {
                    print("iridium: SBDWB1 ok");
                }
                sendTelemetry();
                g_state = 21;

                g_response.size = 0;
            }
        }
    } else if(g_state == 21) {
        if(g_response.size) {
            if(parseResponse("0\r\n\r\nOK\r\n")) {
                if(PRINT_DEBUG_MESSAGES) {
                    print("iridium: SBDWB2 ok");
                }
                g_state = 0;
                // g_state = 220;

                g_waitResponse = false;
                g_response.size = 0;
                makeIdle(SBDI_TIMEOUT);
            }
        }
    } else if(g_state == 30) {
        if(!g_waitResponse) {
            if(PRINT_DEBUG_MESSAGES) {
                print("iridium: SBDD0");
            }
            sendCommand("AT+SBDD0\r");
        } else if(g_response.size) {
            if(parseResponse("0\r\n\r\nOK\r\n")) {
                if(PRINT_DEBUG_MESSAGES) {
                    print("iridium: SBBD0 ok");
                }
                g_state = 0;
                g_needClearBuffer = false;

                g_waitResponse = false;
                g_response.size = 0;
            }
        }
    } else if(g_state == 100) {
        if(!g_waitResponse) {
            if(PRINT_DEBUG_MESSAGES) {
                print("iridium: AT+GSN");
            }
            sendCommand("AT+GSN\r");
        } else if(g_response.size) {
            if(parseResponse("OK\r\n")) {
                char serial[20];
                if(parseGSN(serial)) {
                    printf("iridium found: %s", serial);
                    g_state = 0;
                    // g_state = 220;

                    g_waitResponse = false;
                    g_response.size = 0;
                }
            }
        }
    } else if(g_state == 202) {
        if(!g_waitResponse) {
            print("iridium: AT+SBDTC");
            sendCommand("AT+SBDTC\r");
        } else if(g_response.size) {
            if(parseResponse("OK\r\n")) {
                print("AT+SBDTC ok");
                g_state = 10;
                g_waitResponse = false;
                g_response.size = 0;
            }
        }
    } else if(g_state == 220) {
        if(!g_waitResponse) {
            print("iridium: debug AT+SBDWB1");
            char cmd[12];
            snprintf(cmd, 12, "AT+SBDWB=%i\r", int32_t(9));
            sendCommand(cmd);
        } else if(g_response.size) {
            if(parseResponse("READY\r\n")) {
                print("iridium: SBDWB1 ok");
                sendDebugData();
                g_state = 221;

                g_response.size = 0;
            }
        }
    } else if(g_state == 221) {
        if(g_response.size) {
            if(parseResponse("0\r\n\r\nOK\r\n")) {
                print("iridium: debug SBDWB2 ok");
                g_state = 202;

                g_waitResponse = false;
                g_response.size = 0;
                makeIdle(SBDI_TIMEOUT);
            }
        }
    } else {
        print("UNKNOWN STATE");
    }

    if(time_ms() - g_timePoint > WAIT_RESPONSE_TIMEOUT) {
        print("iridium: no response");

        g_state = 0;
        g_waitResponse = false;
        g_response.size = 0;
        g_timePoint = time_ms();
    }
}
