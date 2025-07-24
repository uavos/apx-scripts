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

int main()
{
    m_pump_rpm_raw("on_pump_rpm_raw"); // subscribe `on changed` event
    m_pwr_eng("on_pwr_eng");           // subscribe `on changed` event

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
