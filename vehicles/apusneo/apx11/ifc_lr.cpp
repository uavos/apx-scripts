#include <apx.h>

#define NODE_LEFT TRUE //ifc-l
//#define NODE_RIGHT TRUE //ifc-r

#if defined NODE_LEFT
constexpr const char *txt_dev = "L";                     //L
constexpr const port_id_t PORT_ID_CAN_AUX{20};           //L=20 //CAN   1Mb
using m_pump_rpm = Mandala<mandala::est::env::usrw::w1>; //pump rpm left
#endif

#if defined NODE_RIGHT
constexpr const char *txt_dev = "R";                     //R
constexpr const port_id_t PORT_ID_CAN_AUX{70};           //R=70 //CAN   1Mb
using m_pump_rpm = Mandala<mandala::est::env::usrw::w2>; //pump rpm right
#endif

constexpr const uint8_t TASK_MAIN_MS{50}; //20Hz
//constexpr const uint8_t TASK_SAVE_MANDALA_MS{100}; //100Hz

//--------------------------size---------------------------
constexpr const uint8_t PACK_CAN_SIZE{12};
//---------------------------------------------------------

//--------------------------pu-----------------------------
constexpr const uint16_t PU_ID{128};
constexpr const uint16_t PU_SHIFT{20};
//constexpr const uint16_t PU_CMD_HEATER{PU_ID + 10};
constexpr const uint16_t PU_CMD_ON{PU_ID + 12};
constexpr const uint16_t PU_CMD_RB{PU_ID + 13};
//---------------------------------------------------------

//--------------------------sp-----------------------------
//constexpr const uint16_t SP_ID{512};
//constexpr const uint16_t SP_SHIFT{20};
//constexpr const uint16_t SP_CMD_POWER_ON{SP_ID + 10};

//bool sp_state{false};
//---------------------------------------------------------

using m_pump_rpm_raw = Mandala<mandala::sns::env::scr::s5>; //pump rpm left/right
uint32_t timer_changed_rpm{};
constexpr const uint16_t PUMP_RPM_TIMEOUT{2000};

//--------------------------Mandala------------------------
//cmd
using m_sp_pwr_ign = Mandala<mandala::ctr::env::pwr::eng>; //sp

int main()
{
    m_sp_pwr_ign();
    m_pump_rpm_raw("on_pump_rpm"); // subscribe `on changed` event

    task("pu_on");  //GCS with terminal command `vmexec("pu_on")`
    task("pu_off"); //GCS with terminal command `vmexec("pu_off")`

    task("pu_rb");

    //task("sp_on");  //GCS with terminal command `vmexec("sp_on")`
    //task("sp_off"); //GCS with terminal command `vmexec("sp_off")`
    //m_pu_cmd_hpwr("on_cmd_hpwr");
    task("on_main", TASK_MAIN_MS); //20 Hz
    //task("on_save_mandala", TASK_SAVE_MANDALA_MS); //10 Hz

    printf("IFC:%s Script ready...\n", txt_dev);

    return 0;
}

EXPORT void on_pump_rpm()
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

EXPORT void sendCmdToCan(const uint32_t &can_id, const uint8_t *data, const uint8_t &size)
{
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

//void pu_cmd_heater(const uint8_t &dev_id, const uint8_t &value)
//{
//    const uint16_t cmd = value * 100;
//
//    uint8_t msg[2] = {};
//    msg[0] = (uint8_t) cmd;
//    msg[1] = cmd >> 8;
//
//    sendCmdToCan(PU_CMD_HEATER + dev_id * PU_SHIFT, msg, 2);
//}

//void sp_cmd_power_on(const uint8_t &value)
//{
//    uint8_t msg[1] = {value};
//
//    sendCmdToCan(SP_CMD_POWER_ON, msg, 1);
//    sendCmdToCan(SP_CMD_POWER_ON + SP_SHIFT, msg, 1);
//}

EXPORT void on_main()
{
    //    bool pwr_ign = (bool) m_sp_pwr_ign::value();
    //    if (sp_state != pwr_ign) {
    //        sp_cmd_power_on(pwr_ign);
    //        sp_state = pwr_ign;
    //    }

    if (time_ms() - timer_changed_rpm > PUMP_RPM_TIMEOUT) {
        m_pump_rpm::publish(0u);
    }
}

//EXPORT void on_cmd_hpwr()
//{
//    constexpr const uint8_t PU_MIN_HEATER{0};
//    constexpr const uint8_t PU_MAX_HEATER{30};
//    uint8_t cmd = limit((uint8_t) m_pu_cmd_hpwr::value(), PU_MIN_HEATER, PU_MAX_HEATER);
//    printf("cmd_hpwr:%u", cmd);
//    pu_cmd_heater(1, cmd);
//}

EXPORT void pu_on()
{
    printf("IFC-%s, pu_on...\n", txt_dev);
    pu_cmd_power_on(1u);
}

EXPORT void pu_off()
{
    printf("IFC-%s, pu_off...\n", txt_dev);
    pu_cmd_power_on(0u);
}

EXPORT void pu_rb(int32_t val)
{
    printf("IFC-%s, pu_rb...\n", txt_dev);
    val = limit(val, 0, 100);
    pu_cmd_rb((uint8_t) val);
}

//EXPORT void sp_on()
//{
//    printf("IFC-%s, sp_on...\n", txt_dev);
//    sp_cmd_power_on(1u);
//}

//EXPORT void sp_off()
//{
//    printf("IFC-%s, sp_off...\n", txt_dev);
//    sp_cmd_power_on(0u);
//}
