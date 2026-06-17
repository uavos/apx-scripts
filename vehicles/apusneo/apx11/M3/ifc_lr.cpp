#include <apx.h>

#define NODE_LEFT TRUE //ifc-l
//#define NODE_RIGHT TRUE //ifc-r

#if defined NODE_LEFT
constexpr const char *txt_dev = "L";           //L
constexpr const port_id_t PORT_ID_CAN_AUX{31}; //L=31 //CAN   1Mb
constexpr const port_id_t PORT_ID_AGL{33};
#endif

#if defined NODE_RIGHT
constexpr const char *txt_dev = "R";           //R
constexpr const port_id_t PORT_ID_CAN_AUX{41}; //R=41 //CAN   1Mb
#endif

//eng
using m_pwr_eng = Mandala<mandala::ctr::env::pwr::eng>;
using m_sns_rpm = Mandala<mandala::sns::env::eng::rpm>;
using m_w1_rpm = Mandala<mandala::est::env::usrw::w1>; //left rpm
using m_w2_rpm = Mandala<mandala::est::env::usrw::w2>; //right rpm

//agl
using m_power_agl = Mandala<mandala::ctr::env::pwr::agl>;
using m_agl_laser = Mandala<mandala::sns::nav::agl::laser>;

constexpr const uint8_t TASK_MAIN_MS{100}; //10Hz

//--------------------------pu-----------------------------
constexpr const uint16_t PU_ID{128};
constexpr const uint16_t PU_SHIFT{20};
constexpr const uint16_t PU_CMD_ON{PU_ID + 12};
constexpr const uint16_t PU_CMD_RB{PU_ID + 13};

//--------------------------sp-----------------------------
constexpr const uint16_t SP_ID{512};
constexpr const uint16_t SP_SHIFT{20};
constexpr const uint16_t SP_CMD_POWER{SP_ID + 10};

int main()
{
    m_power_agl();
    m_agl_laser();
    m_w1_rpm();
    m_w2_rpm();
    m_pwr_eng("on_pwr_eng"); // subscribe `on changed` event

    task("pu_hold"); // 0/1
    task("pu_rb");   // 0..100

    schedule_periodic(task("on_main"), TASK_MAIN_MS);

#if defined NODE_LEFT
    receive(PORT_ID_AGL, "agl_handler");
#endif

    printf("IFC:%s Script ready...\n", txt_dev);

    return 0;
}

EXPORT void on_main()
{
#if defined NODE_LEFT
    //agl
    send(PORT_ID_AGL, (const uint8_t *) "?LD\r\n", 5, true);

    m_sns_rpm::publish(m_w1_rpm::value());
#endif
}

//ASCII:ld,0:-0.66
//HEX:6c 64 2c 30 3a 2d 30 2e 36 36 20 0d 0a
EXPORT void agl_handler(const uint8_t *data, size_t size)
{
    if ((uint32_t) m_power_agl::value() == mandala::pwr_agl_off) {
        return;
    }

    for (uint32_t i = 0; i < size; i++) {
        if (data[i] == ':') {
            int sign = 1;
            float result = 0.0f;
            float divisor = 10.0f;
            bool decimal = false;
            i++;
            if (data[i] == '-') {
                sign = -1;
                i++;
            }
            for (; i < size; i++) {
                if (data[i] >= '0' && data[i] <= '9') {
                    if (decimal) {
                        result += (data[i] - '0') / divisor;
                        divisor *= 10.0f;
                    } else {
                        result = result * 10.0f + (data[i] - '0');
                    }
                } else if (data[i] == '.') {
                    decimal = true;
                } else {
                    break;
                }
            }
            float agl = (float) sign * result;

            //printf("agl:%.2f", agl);
            m_agl_laser::publish(agl);
        }
    }
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

template<typename T>
T limit(T value, T min, T max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
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

void sp_power(const uint8_t &dev_id, const uint8_t &value)
{
    uint8_t msg[1] = {value};
    sendCmdToCan(SP_CMD_POWER + dev_id * SP_SHIFT, msg, 1);
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
